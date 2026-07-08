#pragma once

#include "hdtSkinnedMeshSystem.h"
#include "hdtSkyrimSystem.h"
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>

namespace hdt
{
	class SkinnedMeshWorld : protected btDiscreteDynamicsWorldMt
	{
	public:
		SkinnedMeshWorld();
		~SkinnedMeshWorld();

		virtual void addSkinnedMeshSystem(SkinnedMeshSystem* system);
		virtual void removeSkinnedMeshSystem(SkinnedMeshSystem* system);

		void updateConstraintsForBone(SkinnedMeshBone* bone);

		int stepSimulation(btScalar remainingTimeStep, int maxSubSteps = 1,
			btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) override;

		btVector3& getWind() { return m_windSpeed; }
		const btVector3& getWind() const { return m_windSpeed; }

	protected:
		std::vector<float> m_timeSteps;

		void readTransform(float timeStep)
		{
			const size_t n = m_systems.size();
			if (n == 0)
				return;

			// Main-thread "Setup" phase: pull game bone transforms into the rigid bodies. Already
			// totalled by the Setup metric; this scopes it in the profile tree too.
			BT_PROFILE("readTransform");

			m_timeSteps.resize(n);

			// processSkeletonRoot must be ran synchronously to avoid race issues
			for (size_t i = 0; i < n; ++i)
				m_timeSteps[i] = m_systems[i]->prepareForRead(timeStep);

			tbb::parallel_for(size_t{ 0 }, n, [this](size_t i) {
				m_systems[i]->readTransform(m_timeSteps[i]);
			});
		}

		void writeTransform(float alpha = 1.0f)
		{
			// Main-thread "Apply" phase: write simulated bone transforms back to the game skeleton.
			// alpha < 1 blends toward the previous solved step for smooth rendering above the
			// fixed physics rate (D1); alpha == 1 applies the latest solved state verbatim.
			BT_PROFILE("writeTransform");
			for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->writeTransform(alpha);
		}

		// Capture the current solved state as the interpolation start point, before stepping again.
		void snapshotInterpolation()
		{
			for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->snapshotInterpolation();
		}

		void applyGravity() override;
		void applyWind(btScalar timeStep);

		void predictUnconstraintMotion(btScalar timeStep) override;
		void integrateTransforms(btScalar timeStep) override;
		void performDiscreteCollisionDetection() override;
		void calculateSimulationIslands() override;
		void solveConstraints(btContactSolverInfo& solverInfo) override;

		std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> m_systems;

		btVector3 m_windSpeed;       // world windspeed
		btScalar m_windTime = 0.0f;  // wind simulation clock

	private:
		std::vector<SkinnedMeshBody*> _bodies;
		std::vector<SkinnedMeshShape*> _shapes;
	};
}
