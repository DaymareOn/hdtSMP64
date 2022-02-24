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
		public IEventListener<ShutdownEvent>, public BSTEventSink<SKSECameraEvent>
	{
	public:

		static SkyrimPhysicsWorld* get();

		void doUpdate(float delta);
		void updateActiveState();

		void addSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSystemByNode(void* root);

		void resetTransformsToOriginal();
		void resetSystems();

		void onEvent(const FrameEvent& e) override;
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

		btVector3 applyTranslationOffset();
		void restoreTranslationOffset(const btVector3&);

		btContactSolverInfo& getSolverInfo() { return btDiscreteDynamicsWorld::getSolverInfo(); }

		int min_fps = 60;
		int m_percentageOfFrameTime = 300; // percentage of time per frame doing hdt. Profiler shows 30% is reasonable. Out of 1000.
		float m_timeTick = 1 / 60.f;
		bool m_clampRotations = true;
		bool m_unclampedResets = true;
		float m_unclampedResetAngle = 120.0f;
		float m_averageProcessingTime = 0;
		bool disabled = false;
		uint8_t m_resetPc;

	private:

		SkyrimPhysicsWorld(void);
		~SkyrimPhysicsWorld(void);

		std::mutex m_lock;

		std::atomic_bool m_suspended;
		std::atomic_bool m_loading;
		float m_accumulatedInterval;
		float m_averageInterval;
	};
}
