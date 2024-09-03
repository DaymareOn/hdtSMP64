#pragma once

#include "hdtGroupConstraintSolver.h"

namespace hdt
{
	class SkinnedMeshWorld : protected btDiscreteDynamicsWorldMt
	{
	public:

		SkinnedMeshWorld();
		~SkinnedMeshWorld();

		virtual void addSkinnedMeshSystem(SkinnedMeshSystem* system);
		virtual void removeSkinnedMeshSystem(SkinnedMeshSystem* system);

		int stepSimulation(btScalar remainingTimeStep, int maxSubSteps = 1,
		                   btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) override;

		btVector3& getWind() { return m_windSpeed; }
		const btVector3& getWind() const { return m_windSpeed; }

	protected:

		void resetTransformsToOriginal()
		{
			for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->resetTransformsToOriginal();
		}

		void readTransform(float timeStep)
		{
			for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->readTransform(timeStep);
		}

		void writeTransform() { for (int i = 0; i < m_systems.size(); ++i) m_systems[i]->writeTransform(); }

		void applyGravity() override;
		void applyWind();

		void predictUnconstraintMotion(btScalar timeStep) override;
		void integrateTransforms(btScalar timeStep) override;
		void performDiscreteCollisionDetection() override;
		void solveConstraints(btContactSolverInfo& solverInfo) override;

		std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> m_systems;

		btVector3 m_windSpeed; // world windspeed

	private:

		std::vector<SkinnedMeshBody*> _bodies;
		std::vector<SkinnedMeshShape*> _shapes;
		btConstraintSolverPoolMt* m_solverPool;
		GroupConstraintSolver m_constraintSolver;
	};
}
