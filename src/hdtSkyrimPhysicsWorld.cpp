#include "hdtSkyrimPhysicsWorld.h"
#include "PluginInterfaceImpl.h"

namespace hdt
{
	static const float* timeStamp = (float*)0x12E355C;

	SkyrimPhysicsWorld::SkyrimPhysicsWorld(void)
	{
		gDisableDeactivation = true;
		setGravity(btVector3(0, 0, -9.8f * scaleSkyrim));

		getSolverInfo().m_friction = 0;
		m_averageInterval = m_timeTick;
		m_accumulatedInterval = 0;
	}

	SkyrimPhysicsWorld::~SkyrimPhysicsWorld(void)
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
		if (!disabled && hdt::ActorManager::instance()->activeSkeletons)
		{
			// The tick is the given time for each computation substep. We set it to the average fps
			// to have one average computation each frame when everything is usual.
			// In case of poor fps, we set it to the configured minimum engine value (60 Hz),
			// to still allow a physics with max increments of 1/60s.
			const auto tick = std::min(m_averageInterval, m_timeTick);

			// No need to calculate physics when too little time has passed (time exceptionally short since last computation).
			// This magic value directly impacts the number of computations and the time cost of the mod...
			if (m_accumulatedInterval * 2.0f > tick)
			{
				// The interval is limited to a configurable number of substeps, by default 4.
				// Additional substeps happens when there is a very sudden slowdown, or when fps is lower than min-fps,
				// we have to compute for the passed time we haven't computed.
				// n substeps means that when instant fps is n times lower than usual current fps, we stop computing.
				// So, we guarantee no jitter for fps greater than min-fps / maxSubsteps.
				// For example, if min-fps = 60 and maxSubsteps = 4, we guarantee no jitter for 15+ fps,
				// at the cost of additional simulations.
				const auto remainingTimeStep = std::min(m_accumulatedInterval, tick * m_maxSubSteps);

				readTransform(remainingTimeStep);

				m_tasks.run([this, interval, tick, remainingTimeStep] { doUpdate2ndStep(interval, tick, remainingTimeStep); });
			}
		}
	}

	void SkyrimPhysicsWorld::doUpdate2ndStep(float, const float tick, const float remainingTimeStep)
	{
		if (m_suspended || m_isStasis)
			return;

		std::lock_guard<decltype(m_lock)> l(m_lock);

		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

		LARGE_INTEGER ticks;
		int64_t startTime = 0;
		if (m_doMetrics)
		{
			QueryPerformanceCounter(&ticks);
			startTime = ticks.QuadPart;
		}

		g_pluginInterface.onPreStep({ getCollisionObjectArray(), remainingTimeStep });

		updateActiveState();
		auto offset = applyTranslationOffset();
		stepSimulation(remainingTimeStep, 0, tick);
		restoreTranslationOffset(offset);
		m_accumulatedInterval = 0;

		g_pluginInterface.onPostStep({ getCollisionObjectArray(), remainingTimeStep });

		writeTransform();

		if (m_doMetrics)
		{
			QueryPerformanceCounter(&ticks);
			int64_t endTime = ticks.QuadPart;
			QueryPerformanceFrequency(&ticks);
			// float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
			float lastProcessingTime = (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
			m_2ndStepAverageProcessingTime = (m_2ndStepAverageProcessingTime + lastProcessingTime) * 0.5f;
		}
	}

	void SkyrimPhysicsWorld::suspendSimulationUntilFinished(std::function<void(void)> process)
	{
		this->m_isStasis = true;
		process();
		this->m_isStasis = false;
	}

	btVector3 SkyrimPhysicsWorld::applyTranslationOffset()
	{
		btVector3 center;
		center.setZero();
		int count = 0;
		for (int i = 0; i < m_collisionObjects.size(); ++i)
		{
			auto rig = btRigidBody::upcast(m_collisionObjects[i]);
			if (rig)
			{
				center += rig->getWorldTransform().getOrigin();
				++count;
			}
		}

		if (count > 0)
		{
			center /= static_cast<btScalar>(count);
			for (int i = 0; i < m_collisionObjects.size(); ++i)
			{
				auto rig = btRigidBody::upcast(m_collisionObjects[i]);
				if (rig) rig->getWorldTransform().getOrigin() -= center;
			}
		}
		return center;
	}

	void SkyrimPhysicsWorld::restoreTranslationOffset(const btVector3& offset)
	{
		for (int i = 0; i < m_collisionObjects.size(); ++i)
		{
			auto rig = btRigidBody::upcast(m_collisionObjects[i]);
			if (rig)
			{
				rig->getWorldTransform().getOrigin() += offset;
			}
		}
	}

	void SkyrimPhysicsWorld::setWind(const RE::NiPoint3& a_point, float a_scale, uint32_t a_smoothingSamples)
	{
		if (a_smoothingSamples == 0) 
		{
			logger::error("setWind a_smoothingSamples must be > 0; values ignored");
			return;
		}
		const auto oldValueWeight = a_smoothingSamples - 1;
		if (!btFuzzyZero((m_windSpeed - btVector3(a_point.x, a_point.y, a_point.z)).length())) 
		{
			m_windSpeed.setValue((oldValueWeight * m_windSpeed.getX() + a_point.x * a_scale) / a_smoothingSamples, (oldValueWeight * m_windSpeed.getY() + a_point.y * a_scale) / a_smoothingSamples, (oldValueWeight * m_windSpeed.getZ() + a_point.z * a_scale) / a_smoothingSamples);
			logger::debug
			(
				"Wind Speed now ({:.2f}, {:.2f}, {:.2f}), target ({:.2f}, {:.2f}, {:.2f}) using {} samples.", 
				m_windSpeed.getX(), 
				m_windSpeed.getY(), 
				m_windSpeed.getZ(), 
				a_point.x * a_scale, 
				a_point.y * a_scale, 
				a_point.z * a_scale, 
				a_smoothingSamples
			);
		}
	}

	void SkyrimPhysicsWorld::updateActiveState()
	{
		struct Group
		{
			std::unordered_set<IDStr> tags;
			std::unordered_map<IDStr, std::vector<SkyrimBody*>> list;
		};

		std::unordered_map<RE::NiNode*, Group> maps;

		IDStr invalidString;
		for (auto& i : m_systems)
		{
			auto system = static_cast<SkyrimSystem*>(i.get());
			auto& map = maps[system->m_skeleton.get()];
			for (auto& j : system->meshes())
			{
				auto shape = static_cast<SkyrimBody*>(j.get());
				if (!shape) continue;

				if (shape->m_disableTag == invalidString)
				{
					for (auto& k : shape->m_tags)
						map.tags.insert(k);
				}
				else
				{
					map.list[shape->m_disableTag].push_back(shape);
				}
			}
		}

		for (auto& i : maps)
		{
			for (auto& j : i.second.list)
			{
				if (i.second.tags.find(j.first) != i.second.tags.end())
				{
					for (auto& k : j.second)
						k->m_disabled = true;
				}
				else if (j.second.size())
				{
					std::sort(j.second.begin(), j.second.end(), [](SkyrimBody* a, SkyrimBody* b)
						{
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
		if (!s) return;

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

		for (int i = 0; i < m_systems.size();)
		{
			RE::BSTSmartPointer<SkyrimSystem> s = hdt::make_smart(dynamic_cast<SkyrimSystem*>(m_systems[i].get()));
			if (s && s->m_skeleton == root)
			{
				SkinnedMeshWorld::removeSkinnedMeshSystem(s.get());
			}
			
			else ++i;
		}
	}

	void SkyrimPhysicsWorld::resetTransformsToOriginal()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		SkinnedMeshWorld::resetTransformsToOriginal();
	}

	void SkyrimPhysicsWorld::resetSystems()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		for (auto& i : m_systems)
			i->readTransform(RESET_PHYSICS);
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*)
	{
		auto mm = RE::UI::GetSingleton();

		if ((e->gamePaused || mm->GameIsPaused()) && !m_suspended)
		{
			suspend();
		}
		else if (!(e->gamePaused || mm->GameIsPaused()) && m_suspended)
		{
			resume();
		}

		LARGE_INTEGER ticks;
		int64_t startTime = 0;
		int64_t endTime = 0;
		if (m_doMetrics)
		{
			QueryPerformanceCounter(&ticks);
			startTime = ticks.QuadPart;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);

		float interval = (m_useRealTime ? RE::BSTimer::GetSingleton()->realTimeDelta : RE::BSTimer::GetSingleton()->delta);

		if (interval > FLT_EPSILON && !m_suspended && !m_isStasis && !m_systems.empty())
		{
			doUpdate(interval);
		}
		else if (m_isStasis || (m_suspended && !m_loading))
		{
			writeTransform();
		}

		if (m_doMetrics)
		{
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
		if (m_doMetrics)
		{
			LARGE_INTEGER ticks;
			QueryPerformanceCounter(&ticks);
			int64_t startTime = ticks.QuadPart;

			m_tasks.wait();

			QueryPerformanceCounter(&ticks);
			int64_t endTime = ticks.QuadPart;
			QueryPerformanceFrequency(&ticks);
			// float ticks_per_ms = static_cast<float>(ticks.QuadPart) * 1e-3;
			m_SMPProcessingTimeInMainLoop += (endTime - startTime) / static_cast<float>(ticks.QuadPart) * 1e3f;
			m_averageSMPProcessingTimeInMainLoop = (m_averageSMPProcessingTimeInMainLoop * (m_sampleSize - 1) + m_SMPProcessingTimeInMainLoop) / m_sampleSize;
			float totalSMPTime = m_averageSMPProcessingTimeInMainLoop + m_2ndStepAverageProcessingTime;
			logger::info
			(
				"smp cost in main loop (msecs): {:.2f}, cost outside main loop: {:.2f}, percentage outside vs total: {:.2f}", 
				m_averageSMPProcessingTimeInMainLoop, m_2ndStepAverageProcessingTime, 100. * m_2ndStepAverageProcessingTime / totalSMPTime
			);
		}
		else
		{
			m_tasks.wait();
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*)
	{
		while (m_systems.size())
		{
			SkinnedMeshWorld::removeSkinnedMeshSystem(m_systems.back().get());
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl SkyrimPhysicsWorld::ProcessEvent(const SKSE::CameraEvent* evn, RE::BSTEventSource<SKSE::CameraEvent>*)
	{
		if (evn && evn->oldState && evn->newState)
		{
			if (evn->oldState->id == RE::CameraState::kFirstPerson && evn->newState->id == RE::CameraState::kThirdPerson)
			{
				m_resetPc = 3;
			}
		}

		return RE::BSEventNotifyControl::kContinue;
	}
}
