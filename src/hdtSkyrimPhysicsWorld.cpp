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

		// Exponential average of the frame interval. Kept for diagnostics only; with D1 the step size
		// is fixed (m_timeTick), no longer the average frame interval.
		m_averageInterval += (interval - m_averageInterval) * .125f;

		// No need to calculate physics if there is no active skeleton.
		if (!disabled && hdt::ActorManager::instance()->activeSkeletons) {
			// D1 — rate-decoupled fixed timestep ("fix your timestep").
			// The simulation advances in fixed steps of m_timeTick (= 1/min_fps, default 1/60 s),
			// decoupled from the render rate, instead of one render-rate-sized step every frame.
			// Above the base rate we step only every few frames and carry the leftover time, so a
			// 144 fps user no longer pays for ~144 steps/s when 60 give a visually identical result.
			// It also makes cloth behaviour fps-independent: every frame rate now integrates at the
			// same dt, where before high fps used a smaller (stiffer-feeling) dt than 60 fps.
			const auto fixedStep = m_timeTick;

			// Step only once at least one fixed step of real time has accumulated.
			if (m_accumulatedInterval >= fixedStep) {
				// Consume whole fixed steps, capped at maxSubSteps to bound catch-up work after a
				// hitch or when fps < base rate (default cap 4 ⇒ no jitter for 15+ fps at min-fps 60).
				const int steps = std::min(static_cast<int>(m_accumulatedInterval / fixedStep), m_maxSubSteps);
				const auto remainingTimeStep = steps * fixedStep;

				readTransform(remainingTimeStep);

				m_resetPc -= m_resetPc > 0;

				m_tasks.run([this, interval, fixedStep, remainingTimeStep] { doUpdate2ndStep(interval, fixedStep, remainingTimeStep); });

				// Carry the unspent remainder into the next frame. If we hit the maxSubSteps cap we
				// are behind real time and drop the surplus to avoid a spiral of death.
				if (steps >= m_maxSubSteps)
					m_accumulatedInterval = 0.0f;
				else
					m_accumulatedInterval -= remainingTimeStep;
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
			// D1: snapshot the previous solved transforms (in world space, before the translation
			// offset is applied) so render frames between steps can interpolate up to the new result.
			snapshotInterpolation();
			auto offset = applyTranslationOffset();
			stepSimulation(remainingTimeStep, 0, tick);
			restoreTranslationOffset(offset);
			// m_accumulatedInterval is now carried/reset on the main thread in doUpdate (D1) — the
			// background step no longer touches it, keeping it single-writer from the frame thread.
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

	void SkyrimPhysicsWorld::applyInterpolatedTransform()
	{
		// D1 "Apply" phase, run every render frame: write the simulated cloth back to the skeleton,
		// blended between the last two solved steps so motion stays smooth above the fixed physics
		// rate. When physics is idle/suspended we leave the skeleton untouched, as before.
		if (disabled || m_suspended || !hdt::ActorManager::instance()->activeSkeletons)
			return;

		std::lock_guard<decltype(m_lock)> l(m_lock);
		// alpha = fraction of a fixed step elapsed since the last solved state (carried remainder).
		const float alpha = std::clamp(m_accumulatedInterval / m_timeTick, 0.0f, 1.0f);
		writeTransform(alpha);
		m_pendingTransformUpdate = false;
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
		BT_PROFILE("updateActiveState");

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
	}

	void SkyrimPhysicsWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

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

			applyInterpolatedTransform();

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
			applyInterpolatedTransform();
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
