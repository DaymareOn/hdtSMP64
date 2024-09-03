#pragma once

#include "hdtBulletHelper.h"
#include "hdtBoneScaleConstraint.h"
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include "hdtLCP.h"

namespace hdt
{
	class ConstraintGroup : 
		public RE::BSIntrusiveRefCounted,
		public btMLCPSolver
	{
	public:
		using btMLCPSolver::operator new;
		using btMLCPSolver::operator delete;
		using btMLCPSolver::operator new[];
		using btMLCPSolver::operator delete[];
	public:
		ConstraintGroup();
		
		void scaleConstraint()
		{
			for (auto& i : m_constraints)
				i->scaleConstraint();
		}

		std::vector<RE::BSTSmartPointer<BoneScaleConstraint>> m_constraints;

		virtual void setup(btAlignedObjectArray<btSolverBody>* solverBodies, const btContactSolverInfo& info);
		virtual void iteration(btCollisionObject** bodies, size_t numBodies, const btContactSolverInfo& info);

		static int MaxIterations;
		static bool EnableMLCP;

	protected:

		int getOrInitSolverBody(btCollisionObject& body, btScalar timeStep);

		void createMLCPFast(const btContactSolverInfo& infoGlobal) override;
		btScalar solveSingleIteration(int iteration, btCollisionObject** /*bodies */, int /*numBodies*/,
		                              btPersistentManifold** /*manifoldPtr*/, int /*numManifolds*/,
		                              btTypedConstraint** constraints, int numConstraints,
		                              const btContactSolverInfo& infoGlobal, btIDebugDraw* /*debugDrawer*/) override;
		btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies, int numBodies,
		                                           btPersistentManifold** manifoldPtr, int numManifolds,
		                                           btTypedConstraint** constraints, int numConstraints,
		                                           const btContactSolverInfo& infoGlobal,
		                                           btIDebugDraw* debugDrawer) override;

		std::vector<btTypedConstraint*> m_btConstraints;
		btAlignedObjectArray<btSolverBody>* m_solverBodies;

		btDantzigSolver m_mlcpSolver;

		std::vector<int> m_all2mlcp;
		std::vector<int> m_mlcp2all;
	};
}
