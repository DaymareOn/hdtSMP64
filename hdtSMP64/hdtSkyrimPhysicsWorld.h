#pragma once

#include "hdtSkyrimSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
#include "IEventListener.h"
#include "HookEvents.h"

#include <atomic>
#include "ActorManager.h"
#include "skse64/PapyrusEvents.h"

namespace hdt
{
	constexpr float RESET_PHYSICS = -10.0f;

	class SkyrimPhysicsWorld : protected SkinnedMeshWorld, public IEventListener<FrameEvent>,
		public IEventListener<ShutdownEvent>, public BSTEventSink<SKSECameraEvent>, public IEventListener<FrameSyncEvent>
	{
	public:

		static SkyrimPhysicsWorld* get();

		void doUpdate(float delta);
		void doUpdate2ndStep(float delta, const float tick, const float remainingTimeStep);
		void updateActiveState();

		void addSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSystemByNode(void* root);

		void resetTransformsToOriginal();
		void resetSystems();

		void onEvent(const FrameEvent& e) override;
		void onEvent(const FrameSyncEvent& e) override;
		void onEvent(const ShutdownEvent& e) override;

		EventResult ReceiveEvent(SKSECameraEvent* evn, EventDispatcher<SKSECameraEvent>* dispatcher) override;

		bool isSuspended() { return m_suspended; }

		void suspend(bool loading = false)
		{
			m_suspended = true;
			m_loading = loading;
		}

		void resume()
		{
			m_suspended = false;
			if (m_loading)
			{
				resetSystems();
				m_loading = false;
			}
		}

		void suspendSimulationUntilFinished(std::function<void(void)> process);
		std::atomic_bool m_isStasis = false;

		btVector3 applyTranslationOffset();
		void restoreTranslationOffset(const btVector3&);

		btContactSolverInfo& getSolverInfo() { return btDiscreteDynamicsWorld::getSolverInfo(); }

		// @brief setWind force value for the world
		// @param a_direction wind direction
		// @a_scale Amount to scale the windForce. Defaults to scaleSkyrim
		// @a_smoothingSamples How many samples to smooth. Defaults to 8. Must be greater than 0. Value of 1 means no smoothing
		void setWind(NiPoint3* a_direction, float a_scale = scaleSkyrim, uint32_t a_smoothingSamples = 8);

		concurrency::task_group m_tasks;

		bool m_useRealTime = false;
		int min_fps = 60;
		int m_percentageOfFrameTime = 300; // percentage of time per frame doing hdt. Profiler shows 30% is reasonable. Out of 1000.
		float m_timeTick = 1 / 60.f;
		int m_maxSubSteps = 4;
		bool m_clampRotations = true;
		// @brief rotation speed limit of the PC in radians per second. Must be positive.
		float m_rotationSpeedLimit = 10.f;
		bool m_unclampedResets = true;
		float m_unclampedResetAngle = 120.0f;
		float m_2ndStepAverageProcessingTime = 0;
		float m_averageSMPProcessingTimeInMainLoop = 0;
		bool disabled = false;
		uint8_t m_resetPc;
		bool m_doMetrics = false;
		int m_sampleSize = 5; // how many samples (each sample taken every second) for determining average time per activeSkeleton.

		//wind settings
		bool m_enableWind = true;
		float m_windStrength = 2.0f; // compare to gravity acceleration of 9.8
		float m_distanceForNoWind = 50.0f; // how close to wind obstruction to fully block wind
		float m_distanceForMaxWind = 3000.0f; // how far to wind obstruction to not block wind

	private:

		SkyrimPhysicsWorld(void);
		~SkyrimPhysicsWorld(void);

		std::mutex m_lock;

		std::atomic_bool m_suspended;
		std::atomic_bool m_loading;
		float m_accumulatedInterval;
		float m_averageInterval;
		float m_SMPProcessingTimeInMainLoop = 0;
	};
}
