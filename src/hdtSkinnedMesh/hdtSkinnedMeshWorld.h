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

		void resetTransformsToOriginal()
		{
			for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->resetTransformsToOriginal();
		}

		void readTransform(float timeStep)
		{
			const size_t n = m_systems.size();
			if (n == 0)
				return;

			m_timeSteps.resize(n);

			// processSkeletonRoot must be ran synchronously to avoid race issues
			for (size_t i = 0; i < n; ++i)
				m_timeSteps[i] = m_systems[i]->prepareForRead(timeStep);

			concurrency::parallel_for(size_t{ 0 }, n, [this](size_t i) {
				m_systems[i]->readTransform(m_timeSteps[i]);
			});
		}

		void writeTransform()
		{
			for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->writeTransform();
		}

		void applyGravity() override;
		void applyWind();

		void predictUnconstraintMotion(btScalar timeStep) override;
		void integrateTransforms(btScalar timeStep) override;
		void performDiscreteCollisionDetection() override;
		void calculateSimulationIslands() override;
		void solveConstraints(btContactSolverInfo& solverInfo) override;

		std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> m_systems;

		btVector3 m_windSpeed;  // world windspeed

	private:
		std::vector<SkinnedMeshBody*> _bodies;
		std::vector<SkinnedMeshShape*> _shapes;
	};
}
