#include "hdtSkyrimPhysicsWorld.h"
#include "PluginInterfaceImpl.h"
#include "WeatherManager.h"
#include "hdtPhysicsProfiler.h"

#include <LinearMath/btQuickprof.h>

namespace hdt
{
	static const float* timeStamp = (float*)0x12E355C;

	SkyrimPhysicsWorld::SkyrimPhysicsWorld(void)
	{
		gDisableDeactivation = true;
		setGravity(btVector3(0, 0, -9.8f * scaleSkyrim));

		// https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h

		getSolverInfo().m_friction = 0;

		// This should be enabled by default, but just for clarity I put it here too
		getSolverInfo().m_splitImpulse = true;

		// Set a very low threshold so even micro-penetrations use Split Impulse
		// Too low might cause weird visuals - default is -0.04f
		getSolverInfo().m_splitImpulsePenetrationThreshold = -0.01f;

		// Default ERP2 is 0.2
		// From Bullet: error reduction for non-contact constraints
		getSolverInfo().m_erp2 = 0.15f;

		// constraint force mixing for contacts and non-contacts
		// Adds "sponginess" to collisions to absorb the constant recalculations
		// Default is 0
		getSolverInfo().m_globalCfm = 0.001f;

		// Ignore Bounciness (Restitution) on slow micro-collisions
		// If objects are moving slower than this, they will not bounce at all.
		// The default is 0.2f, but putting this here since it's very noteworthy!
		getSolverInfo().m_restitutionVelocityThreshold = 0.2f;

		// Default is = SOLVER_USE_WARMSTARTING | SOLVER_SIMD;
		// But we don't even use warm starts since we delete the manifolds every frame
		// SOLVER_SIMD nets a small performance uplift
		// SOLVER_RANDMIZE_ORDER is also possible, but I clocked a pretty heavy performance hit. Maybe make it a config option
		getSolverInfo().m_solverMode = SOLVER_SIMD;
		getSolverInfo().m_leastSquaresResidualThreshold = 0.0001f;

		m_averageInterval = m_timeTick;
		m_accumulatedInterval = 0;
	}

	void SkyrimPhysicsWorld::setProfilerCapture(bool a_enabled, std::uint64_t a_sampleFrames, std::uint64_t a_printFrames)
	{
		auto simulationLock = lockSimulation();
		physicsprofiler::setCapture(a_enabled, a_sampleFrames, a_printFrames);
	}

	replay::SolverConfig SkyrimPhysicsWorld::buildReplaySolverConfig()
	{
		replay::SolverConfig s;
		s.gravity = replay::toVec3(getGravity());
		const auto& si = getSolverInfo();
		s.friction = si.m_friction;
		s.splitImpulse = si.m_splitImpulse ? 1 : 0;
		s.splitImpulsePenetrationThreshold = si.m_splitImpulsePenetrationThreshold;
		s.erp2 = si.m_erp2;
		s.globalCfm = si.m_globalCfm;
		s.restitutionVelocityThreshold = si.m_restitutionVelocityThreshold;
		s.solverMode = si.m_solverMode;
		s.leastSquaresResidualThreshold = si.m_leastSquaresResidualThreshold;
		s.timeTick = m_timeTick;
		s.maxSubSteps = m_maxSubSteps;
		s.enableWind = m_enableWind ? 1 : 0;
		s.windStrength = m_windStrength;
		s.distanceForNoWind = m_distanceForNoWind;
		s.distanceForMaxWind = m_distanceForMaxWind;
		s.disableDeactivation = gDisableDeactivation ? 1 : 0;
		return s;
	}

	replay::Snapshot SkyrimPhysicsWorld::buildReplaySnapshot(SkyrimSystem* system)
	{
		auto [it, inserted] = m_replaySystemIds.try_emplace(system, m_replayNextSystemId);
		if (inserted)
			++m_replayNextSystemId;
		replay::Snapshot snap = replay::buildSnapshot(*system, it->second);
		// Carry the in-game build/read cost (file read + XML parse + NIF skin extraction + construction,
		// timed in createOrUpdateSystem) into the capture so the benchmark can report it.
		snap.buildTimeMicros = system->m_buildTimeMicros;
		return snap;
	}

	void SkyrimPhysicsWorld::setReplayCapture(bool a_enabled, const std::string& a_path,
		std::uint32_t a_frameCap, std::size_t a_sizeCap, bool a_golden)
	{
		auto simulationLock = lockSimulation();

		if (a_enabled) {
			m_replayCapturePath = a_path;
			m_replayGolden = a_golden;
			m_replaySystemIds.clear();
			m_replayNextSystemId = 0;

			std::vector<replay::Snapshot> initial;
			initial.reserve(m_systems.size());
			for (auto& i : m_systems) {
				if (auto* s = dynamic_cast<SkyrimSystem*>(i.get()))
					initial.push_back(buildReplaySnapshot(s));
			}

			m_replayCapture.begin(buildReplaySolverConfig(), std::move(initial),
				std::string(Plugin::BUILD_INFO), 0, static_cast<std::uint32_t>(m_systems.size()),
				a_golden, a_frameCap, a_sizeCap);
			logger::info("Replay capture started -> {}", a_path);
		} else if (m_replayCapture.active() || m_replayCapture.capReached()) {
			if (!m_replayCapturePath.empty() && m_replayCapture.flushToFile(m_replayCapturePath))
				logger::info("Replay capture flushed -> {} ({} bytes)", m_replayCapturePath, m_replayCapture.approxSizeBytes());
			else
				logger::error("Replay capture flush failed for {}", m_replayCapturePath);
			m_replayCapture.reset();
			m_replaySystemIds.clear();
		}
	}

	void SkyrimPhysicsWorld::captureReplayFrame(float remainingTimeStep, float tick)
	{
		replay::Frame frame;
		frame.remainingTimeStep = remainingTimeStep;
		frame.tick = tick;
		frame.windSpeed = replay::toVec3(m_windSpeed);
		frame.reset = remainingTimeStep <= RESET_PHYSICS ? 1 : 0;

		for (auto& i : m_systems) {
			auto* s = dynamic_cast<SkyrimSystem*>(i.get());
			if (!s)
				continue;
			auto idIt = m_replaySystemIds.find(s);
			if (idIt == m_replaySystemIds.end())
				continue;  // a system added before capture started but not snapshotted; skip
			replay::captureFrameBones(*s, idIt->second, frame, m_replayGolden);
		}

		// addFrame auto-stops the buffer (without touching disk) once a frame/size cap is hit - that is
		// the RAM fail-safe. Flushing to disk is the caller's job: updateReplayRecording() for a console
		// recording, or the explicit SetReplayCapture(false) for the Papyrus path.
		m_replayCapture.addFrame(std::move(frame));
	}

	void SkyrimPhysicsWorld::requestReplayRecording(float durationSec, std::size_t sizeCapBytes, std::string path)
	{
		// Stage only - the capture begins in updateReplayRecording() once the console is closed.
		m_recordPath = std::move(path);
		m_recordDurationSec = durationSec;
		m_recordSizeCap = sizeCapBytes;
		m_recordPending = true;
	}

	void SkyrimPhysicsWorld::updateReplayRecording()
	{
		if (!m_recordPending && !m_recordActive)
			return;

		auto* ui = RE::UI::GetSingleton();
		const bool consoleOpen = ui && ui->IsMenuOpen(RE::Console::MENU_NAME);

		if (m_recordPending) {
			// Wait until the player has dismissed the console, then begin. setReplayCapture snapshots the
			// live scene under the simulation lock, so it must run here (not under m_lock).
			if (consoleOpen)
				return;
			setReplayCapture(true, m_recordPath, 0, m_recordSizeCap, false);
			m_recordStart = std::chrono::steady_clock::now();
			m_recordActive = true;
			m_recordPending = false;
			logger::info("smp record: started -> {} (<= {:.1f}s, <= {} bytes)",
				m_recordPath, m_recordDurationSec, m_recordSizeCap);
			if (auto* console = RE::ConsoleLog::GetSingleton())
				console->Print("[HDT-SMP] Replay recording started.");
			return;
		}

		// Active: stop on whichever limit is reached first - elapsed wall-clock time, or the size cap
		// (the buffer raises capReached() once it auto-stops on size).
		const float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - m_recordStart).count();
		const bool timeUp = m_recordDurationSec > 0.0f && elapsed >= m_recordDurationSec;
		const bool sizeUp = m_replayCapture.capReached();
		if (!timeUp && !sizeUp)
			return;

		setReplayCapture(false);  // flushes to m_recordPath and resets the buffer
		m_recordActive = false;
		const char* why = sizeUp ? "size cap" : "time limit";
		logger::info("smp record: stopped ({}) -> {}", why, m_recordPath);
		if (auto* console = RE::ConsoleLog::GetSingleton())
			console->Print("[HDT-SMP] Replay recording finished (%s). Saved to: %s", why, m_recordPath.c_str());
	}

	SkyrimPhysicsWorld::~SkyrimPhysicsWorld(void) noexcept
	{
	}

	//void hdtSkyrimPhysicsWorld::suspend()
	//{
	//	m_suspended++;
	//}

	//void hdtSkyrimPhysicsWorld::resume()
	//{
	//	--m_suspended;
	//}

	//void hdtSkyrimPhysicsWorld::switchToSeperateClock()
	//{
	//	m_lock.lock();
	//	m_useSeperatedClock = true;
	//	m_timeLastUpdate = clock()*0.001;
	//	m_lock.unlock();
	//}

	//void hdtSkyrimPhysicsWorld::switchToInternalClock()
	//{
	//	m_lock.lock();
	//	m_useSeperatedClock = false;
	//	m_timeLastUpdate = *timeStamp;
	//	m_lock.unlock();
	//}

	SkyrimPhysicsWorld* SkyrimPhysicsWorld::get()
	{
		static SkyrimPhysicsWorld g_World;
		return &g_World;
	}

	void SkyrimPhysicsWorld::doUpdate(float interval)
	{
		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

		// Time passed since last computation
		m_accumulatedInterval += interval;

		// Exponential average - becomes the tick; the tick equals the average interval when the interval is stable.
		m_averageInterval += (interval - m_averageInterval) * .125f;

		// No need to calculate physics if there is no active skeleton.
		if (!disabled && hdt::ActorManager::instance()->activeSkeletons) {
			// The tick is the given time for each computation substep. We set it to the average fps
			// to have one average computation each frame when everything is usual.
			// In case of poor fps, we set it to the configured minimum engine value (60 Hz),
			// to still allow a physics with max increments of 1/60s.
			const auto tick = std::min(m_averageInterval, m_timeTick);

			// No need to calculate physics when too little time has passed (time exceptionally short since last computation).
			// This magic value directly impacts the number of computations and the time cost of the mod...
			if (m_accumulatedInterval * 2.0f > tick) {
				// The interval is limited to a configurable number of substeps, by default 4.
				// Additional substeps happens when there is a very sudden slowdown, or when fps is lower than min-fps,
				// we have to compute for the passed time we haven't computed.
				// n substeps means that when instant fps is n times lower than usual current fps, we stop computing.
				// So, we guarantee no jitter for fps greater than min-fps / maxSubsteps.
				// For example, if min-fps = 60 and maxSubsteps = 4, we guarantee no jitter for 15+ fps,
				// at the cost of additional simulations.
				const auto remainingTimeStep = std::min(m_accumulatedInterval, tick * m_maxSubSteps);

				readTransform(remainingTimeStep);

				m_resetPc -= m_resetPc > 0;

				m_tasks.run([this, interval, tick, remainingTimeStep] { doUpdate2ndStep(interval, tick, remainingTimeStep); });
			}
		}
	}

	void SkyrimPhysicsWorld::doUpdate2ndStep(float, const float tick, const float remainingTimeStep)
	{
		if (m_suspended)
			return;

		std::lock_guard<decltype(m_lock)> l(m_lock);

		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

		LARGE_INTEGER ticks;
		int64_t startTime = 0;
		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			startTime = ticks.QuadPart;
		}

		g_pluginInterface.onPreStep({ getCollisionObjectArray(), remainingTimeStep });

		{
			BT_PROFILE("HDTSMP_doUpdate2ndStep");
			updateActiveState();
			auto offset = applyTranslationOffset();
			stepSimulation(remainingTimeStep, 0, tick);
			restoreTranslationOffset(offset);
			// Capture the per-frame record here (D9): kinematic bones still carry this frame's input
			// target in m_currentTransform, and dynamic bones carry their solved post-step pose in the
			// rigid body - writeTransform (FrameSyncEvent) has not yet run.
			if (m_replayCapture.active())
				captureReplayFrame(remainingTimeStep, tick);
			m_accumulatedInterval = 0;
			m_pendingTransformUpdate = true;
		}

		g_pluginInterface.onPostStep({ getCollisionObjectArray(), remainingTimeStep });

		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			int64_t endTime = ticks.QuadPart;
			QueryPerformanceFrequency(&ticks);
			// float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
			float lastProcessingTime = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
			m_2ndStepAverageProcessingTime = (m_2ndStepAverageProcessingTime + lastProcessingTime) * 0.5f;
		}

		physicsprofiler::advanceFrame();
	}

	std::unique_lock<std::mutex> SkyrimPhysicsWorld::lockSimulation()
	{
		m_tasks.wait();
		return std::unique_lock(m_lock);
	}

	btVector3 SkyrimPhysicsWorld::applyTranslationOffset()
	{
		btVector3 center;
		center.setZero();
		int count = 0;
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto rig = btRigidBody::upcast(m_collisionObjects[i]);
			if (rig) {
				center += rig->getWorldTransform().getOrigin();
				++count;
			}
		}

		if (count > 0) {
			center /= static_cast<btScalar>(count);
			for (int i = 0; i < m_collisionObjects.size(); ++i) {
				auto rig = btRigidBody::upcast(m_collisionObjects[i]);
				if (rig)
					rig->getWorldTransform().getOrigin() -= center;
			}
		}
		return center;
	}

	void SkyrimPhysicsWorld::restoreTranslationOffset(const btVector3& offset)
	{
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto rig = btRigidBody::upcast(m_collisionObjects[i]);
			if (rig) {
				rig->getWorldTransform().getOrigin() += offset;
			}
		}
	}

	void SkyrimPhysicsWorld::setWind(const RE::NiPoint3& a_point, float a_scale, uint32_t a_smoothingSamples)
	{
		if (a_smoothingSamples == 0) {
			logger::error("setWind a_smoothingSamples must be > 0; values ignored");
			return;
		}
		const auto oldValueWeight = a_smoothingSamples - 1;
		if (!btFuzzyZero((m_windSpeed - btVector3(a_point.x, a_point.y, a_point.z)).length())) {
			m_windSpeed.setValue((oldValueWeight * m_windSpeed.getX() + a_point.x * a_scale) / a_smoothingSamples, (oldValueWeight * m_windSpeed.getY() + a_point.y * a_scale) / a_smoothingSamples, (oldValueWeight * m_windSpeed.getZ() + a_point.z * a_scale) / a_smoothingSamples);
			logger::debug(
				"Wind Speed now ({:.2f}, {:.2f}, {:.2f}), target ({:.2f}, {:.2f}, {:.2f}) using {} samples.",
				m_windSpeed.getX(),
				m_windSpeed.getY(),
				m_windSpeed.getZ(),
				a_point.x * a_scale,
				a_point.y * a_scale,
				a_point.z * a_scale,
				a_smoothingSamples);
		}
	}

	void SkyrimPhysicsWorld::updateActiveState()
	{
		struct Group
		{
			std::unordered_set<RE::BSFixedString> tags;
			std::unordered_map<RE::BSFixedString, std::vector<SkyrimBody*>> list;
		};

		std::unordered_map<RE::NiNode*, Group> maps;

		RE::BSFixedString invalidString;
		for (auto& i : m_systems) {
			auto system = static_cast<SkyrimSystem*>(i.get());
			auto& map = maps[system->m_skeleton.get()];
			for (auto& j : system->meshes()) {
				auto shape = static_cast<SkyrimBody*>(j.get());
				if (!shape)
					continue;

				if (shape->m_disableTag == invalidString) {
					for (auto& k : shape->m_tags)
						map.tags.insert(k);
				} else {
					map.list[shape->m_disableTag].push_back(shape);
				}
			}
		}

		for (auto& i : maps) {
			for (auto& j : i.second.list) {
				if (i.second.tags.find(j.first) != i.second.tags.end()) {
					for (auto& k : j.second)
						k->m_disabled = true;
				} else if (j.second.size()) {
					std::sort(j.second.begin(), j.second.end(), [](SkyrimBody* a, SkyrimBody* b) {
						if (a->m_disablePriority != b->m_disablePriority)
							return a->m_disablePriority > b->m_disablePriority;
						return a < b;
					});

					for (auto& k : j.second)
						k->m_disabled = true;
					j.second[0]->m_disabled = false;
				}
			}
		}
	}

	void SkyrimPhysicsWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		auto s = dynamic_cast<SkyrimSystem*>(system);
		if (!s)
			return;

		s->m_initialized = false;
		SkinnedMeshWorld::addSkinnedMeshSystem(system);

		// Scene-log: record the new system as a full sub-scene serialize at this frame (D3/D4).
		if (m_replayCapture.active()) {
			replay::SceneEvent e;
			e.kind = replay::SceneEventKind::AddSystem;
			e.snapshot = buildReplaySnapshot(s);
			e.systemId = e.snapshot.systemId;
			m_replayCapture.addSceneEvent(std::move(e));
		}
	}

	void SkyrimPhysicsWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

		// Scene-log: record the removal (id + frame) before the system goes away (D3/D4).
		if (m_replayCapture.active()) {
			auto it = m_replaySystemIds.find(system);
			if (it != m_replaySystemIds.end()) {
				replay::SceneEvent e;
				e.kind = replay::SceneEventKind::RemoveSystem;
				e.systemId = it->second;
				m_replayCapture.addSceneEvent(std::move(e));
			}
		}

		SkinnedMeshWorld::removeSkinnedMeshSystem(system);
	}

	void SkyrimPhysicsWorld::removeSystemByNode(void* root)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

		for (int i = 0; i < m_systems.size();) {
			RE::BSTSmartPointer<SkyrimSystem> s = hdt::make_smart(dynamic_cast<SkyrimSystem*>(m_systems[i].get()));
			if (s && s->m_skeleton == root) {
				SkinnedMeshWorld::removeSkinnedMeshSystem(s.get());
			}

			else
				++i;
		}
	}

	void SkyrimPhysicsWorld::resetSystems()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		for (auto& i : m_systems)
			i->readTransform(i->prepareForRead(RESET_PHYSICS));
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*)
	{
		auto mm = RE::UI::GetSingleton();

		if ((e->gamePaused || mm->GameIsPaused()) && !m_suspended) {
			suspend();
		} else if (!(e->gamePaused || mm->GameIsPaused()) && m_suspended) {
			resume();
		}

		// Drive the console-armed 'smp record' lifecycle (deferred start once the console closes, then
		// time/size auto-stop). Must run outside m_lock - it may call setReplayCapture, which locks.
		updateReplayRecording();

		if (m_enableWind) {
			WeatherManager::runWeatherTick(RE::GetSecondsSinceLastFrame());
		}

		LARGE_INTEGER ticks;
		int64_t startTime = 0;
		int64_t endTime = 0;
		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			startTime = ticks.QuadPart;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);

		float interval = (m_useRealTime ? RE::BSTimer::GetSingleton()->realTimeDelta : RE::BSTimer::GetSingleton()->delta);

		if (interval > FLT_EPSILON && !m_suspended && !m_systems.empty()) {
			doUpdate(interval);
		} else if (m_suspended && !m_loading) {
			writeTransform();
		}

		if (m_doMetrics) {
			QueryPerformanceCounter(&ticks);
			endTime = ticks.QuadPart;
			QueryPerformanceFrequency(&ticks);
			// float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
			m_SMPProcessingTimeInMainLoop = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::FrameSyncEvent*, RE::BSTEventSource<Events::FrameSyncEvent>*)
	{
		if (m_doMetrics) {
			LARGE_INTEGER ticks, freq;
			QueryPerformanceCounter(&ticks);
			int64_t t0 = ticks.QuadPart;

			m_tasks.wait();

			QueryPerformanceCounter(&ticks);
			int64_t t1 = ticks.QuadPart;

			if (m_pendingTransformUpdate) {
				std::lock_guard<decltype(m_lock)> l(m_lock);
				writeTransform();
				m_pendingTransformUpdate = false;
			}

			QueryPerformanceCounter(&ticks);
			int64_t t2 = ticks.QuadPart;
			QueryPerformanceFrequency(&freq);
			float f = static_cast<float>(freq.QuadPart);

			float instWaitTime = (t1 - t0) / f * 1000.0f;
			float instWriteTime = (t2 - t1) / f * 1000.0f;
			float instSetupTime = m_SMPProcessingTimeInMainLoop;

			float instFpsImpact = instSetupTime + instWaitTime + instWriteTime;

			m_averageSMPProcessingTimeInMainLoop = (m_averageSMPProcessingTimeInMainLoop * (m_sampleSize - 1) + instFpsImpact) / m_sampleSize;

			// Smooth the individual components for logging so the math adds up perfectly visually
			static float avgSetupTime = 0.0f;
			static float avgWaitTime = 0.0f;
			static float avgWriteTime = 0.0f;

			avgSetupTime = (avgSetupTime * (m_sampleSize - 1) + instSetupTime) / m_sampleSize;
			avgWaitTime = (avgWaitTime * (m_sampleSize - 1) + instWaitTime) / m_sampleSize;
			avgWriteTime = (avgWriteTime * (m_sampleSize - 1) + instWriteTime) / m_sampleSize;

			// The background thread's math time (this is already smoothed in doUpdate2ndStep)
			float avgBackgroundCalcTime = m_2ndStepAverageProcessingTime;

			// How much of that background math was successfully hidden?
			float avgHiddenTime = std::max(0.0f, avgBackgroundCalcTime - avgWaitTime);

			float avgTotalCpuWork = avgSetupTime + avgBackgroundCalcTime + avgWriteTime;

			logger::info(
				"[SMP Metrics] Avg Frame-time Impact: {:.2f}ms (Setup: {:.2f}, Wait: {:.2f}, Apply: {:.2f}) | Avg Hidden Time: {:.2f}ms | Avg Total CPU Work: {:.2f}ms",
				m_averageSMPProcessingTimeInMainLoop,  // This will exactly equal Setup + Wait + Apply
				avgSetupTime,
				avgWaitTime,
				avgWriteTime,
				avgHiddenTime,
				avgTotalCpuWork);
		} else {
			m_tasks.wait();
			if (m_pendingTransformUpdate) {
				std::lock_guard<decltype(m_lock)> l(m_lock);
				writeTransform();
				m_pendingTransformUpdate = false;
			}
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*)
	{
		while (m_systems.size()) {
			SkinnedMeshWorld::removeSkinnedMeshSystem(m_systems.back().get());
		}

		m_tasks.wait();

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const SKSE::CameraEvent* evn, RE::BSTEventSource<SKSE::CameraEvent>*)
	{
		if (evn && evn->oldState && evn->newState) {
			if (evn->oldState->id == RE::CameraState::kFirstPerson && evn->newState->id == RE::CameraState::kThirdPerson) {
				m_resetPc = 3;
			}
		}

		return RE::BSEventNotifyControl::kContinue;
	}
}
