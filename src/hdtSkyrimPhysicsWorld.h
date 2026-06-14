#pragma once

#include "ActorManager.h"
#include "Events.h"
#include "Replay/hdtReplayCapture.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
#include "hdtSkyrimSystem.h"

#include <chrono>
#include <unordered_map>

namespace hdt
{
	// RESET_PHYSICS now lives on the core (hdtSkinnedMeshWorld.h, included transitively above).

	class SkyrimPhysicsWorld :
		protected SkinnedMeshWorld,
		public RE::BSTEventSink<Events::FrameEvent>,
		public RE::BSTEventSink<Events::ShutdownEvent>,
		public RE::BSTEventSink<SKSE::CameraEvent>,
		public RE::BSTEventSink<Events::FrameSyncEvent>
	{
	public:
		static SkyrimPhysicsWorld* get();

		void doUpdate(float delta);
		void doUpdate2ndStep(float delta, const float tick, const float remainingTimeStep);
		void updateActiveState();
		void setProfilerCapture(bool a_enabled, std::uint64_t a_sampleFrames = 240, std::uint64_t a_printFrames = 240);

		// Replay capture toggle (D9), mirrors setProfilerCapture. On enable, snapshots the current
		// scene under the simulation lock and begins buffering per-frame records + scene-log events in
		// RAM; on disable, flushes the whole capture to `a_path`. frameCap/sizeCap auto-stop capture as
		// a fail-safe (0 = unlimited); golden records dynamic-bone outputs for the D8 parity fixture.
		void setReplayCapture(bool a_enabled, const std::string& a_path = "",
			std::uint32_t a_frameCap = 0, std::size_t a_sizeCap = 0, bool a_golden = false);

		// Arms a console-driven recording (the `smp record` command). We only *stage* the request here;
		// recording actually begins later, once the player has closed the console, and stops on its own
		// after durationSec seconds or once the buffer hits sizeCapBytes - whichever comes first. The
		// deferral + auto-stop are driven by updateReplayRecording(), polled each frame.
		void requestReplayRecording(float durationSec, std::size_t sizeCapBytes, std::string path);

		void addSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSkinnedMeshSystem(SkinnedMeshSystem* system) override;
		void removeSystemByNode(void* root);
		using SkinnedMeshWorld::updateConstraintsForBone;
		// m_enableWind now lives on the core SkinnedMeshWorld; re-publicize it so existing callers
		// (config.cpp, ActorManager.cpp) keep using SkyrimPhysicsWorld::get()->m_enableWind unchanged.
		using SkinnedMeshWorld::m_enableWind;

		void resetSystems();

		RE::BSEventNotifyControl ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*) override;
		RE::BSEventNotifyControl ProcessEvent(const Events::FrameSyncEvent* e, RE::BSTEventSource<Events::FrameSyncEvent>*) override;
		RE::BSEventNotifyControl ProcessEvent(const Events::ShutdownEvent* e, RE::BSTEventSource<Events::ShutdownEvent>*) override;
		RE::BSEventNotifyControl ProcessEvent(const SKSE::CameraEvent* evn, RE::BSTEventSource<SKSE::CameraEvent>* dispatcher) override;

		bool isSuspended() { return m_suspended; }

		void suspend(bool loading = false)
		{
			m_suspended = true;
			m_loading = loading;
		}

		void resume()
		{
			m_suspended = false;
			if (m_loading) {
				resetSystems();
				m_loading = false;
			}
		}

		// This is used for when you want to mutate some physics objects without causing problems
		// Bullet is VERY sensitive to changes during simulation!
		std::unique_lock<std::mutex> lockSimulation();

		btVector3 applyTranslationOffset();
		void restoreTranslationOffset(const btVector3&);

		btContactSolverInfo& getSolverInfo() { return btDiscreteDynamicsWorld::getSolverInfo(); }

		// @brief setWind force value for the world
		// @param a_direction wind direction
		// @a_scale Amount to scale the windForce. Defaults to scaleSkyrim
		// @a_smoothingSamples How many samples to smooth. Defaults to 8. Must be greater than 0. Value of 1 means no smoothing
		void setWind(const RE::NiPoint3& a_direction, float a_scale = scaleSkyrim, uint32_t a_smoothingSamples = 8);

		tbb::task_group m_tasks;

		bool m_pendingTransformUpdate = false;
		bool m_useRealTime = false;
		int min_fps = 60;
		float m_budgetMs = 3.5f;
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
		int m_sampleSize = 5;  // how many samples (each sample taken every second) for determining average time per activeSkeleton.

		//wind settings (m_enableWind is inherited from SkinnedMeshWorld and re-published above)
		float m_windStrength = 2.0f;           // compare to gravity acceleration of 9.8
		float m_distanceForNoWind = 50.0f;     // how close to wind obstruction to fully block wind
		float m_distanceForMaxWind = 3000.0f;  // how far to wind obstruction to not block wind

	private:
		SkyrimPhysicsWorld(void);
		~SkyrimPhysicsWorld(void) noexcept;

		// --- replay capture state (D9). All touched only under m_lock / the simulation lock. ---
		replay::SolverConfig buildReplaySolverConfig();
		replay::Snapshot buildReplaySnapshot(SkyrimSystem* system);  // assigns/looks up a stable id
		void captureReplayFrame(float remainingTimeStep, float tick);

		// Drives the console-armed recording: starts the capture once the console closes, then stops it
		// when the duration elapses or the size cap is hit. Polled once per frame on the main thread
		// (from ProcessEvent(FrameEvent)), never under m_lock - it calls setReplayCapture, which takes
		// the simulation lock itself.
		void updateReplayRecording();

		replay::CaptureBuffer m_replayCapture;
		std::string m_replayCapturePath;
		bool m_replayGolden = false;
		std::uint32_t m_replayNextSystemId = 0;
		std::unordered_map<SkinnedMeshSystem*, std::uint32_t> m_replaySystemIds;

		// console-armed 'smp record' controller
		bool m_recordPending = false;  // armed, waiting for the console to close
		bool m_recordActive = false;   // capture is running
		float m_recordDurationSec = 0.0f;
		std::size_t m_recordSizeCap = 0;
		std::string m_recordPath;
		std::chrono::steady_clock::time_point m_recordStart;

		std::mutex m_lock;

		std::atomic_bool m_suspended;
		std::atomic_bool m_loading;
		float m_accumulatedInterval;
		float m_averageInterval;
		float m_SMPProcessingTimeInMainLoop = 0;
	};
}
