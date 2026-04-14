#include "hdtSkinnedMeshWorld.h"
#include "hdtBoneScaleConstraint.h"
#include "hdtDispatcher.h"
#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtSkyrimSystem.h"
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace hdt
{
	SkinnedMeshWorld::SkinnedMeshWorld() :
		btDiscreteDynamicsWorldMt(
			nullptr,
			nullptr,
			// Pool of regular sequential solvers one per hardware thread.
			// Each island gets dispatched to a free solver on any thread.
			new btConstraintSolverPoolMt(
				std::max(1, static_cast<int>(std::thread::hardware_concurrency()))),
			nullptr,  // no Mt solver, avoids btBatchedConstraints entirely (we are not designed for that yet)
			nullptr)
	{
		btSetTaskScheduler(btGetPPLTaskScheduler());

		m_windSpeed = _mm_setzero_ps();

		auto collisionConfiguration = new btDefaultCollisionConfiguration;
		auto collisionDispatcher = new CollisionDispatcher(collisionConfiguration);

		m_dispatcher1 = collisionDispatcher;
		m_broadphasePairCache = new btDbvtBroadphase();
	}

	SkinnedMeshWorld::~SkinnedMeshWorld()
	{
		for (auto system : m_systems) {
			for (int i = 0; i < system->m_meshes.size(); ++i)
				removeCollisionObject(system->m_meshes[i].get());

			for (int i = 0; i < system->m_constraints.size(); ++i)
				if (system->m_constraints[i]->m_constraint)
					removeConstraint(system->m_constraints[i]->m_constraint);

			for (int i = 0; i < system->m_bones.size(); ++i)
				removeRigidBody(&system->m_bones[i]->m_rig);

			for (auto i : system->m_constraintGroups)
				for (auto j : i->m_constraints)
					if (j->m_constraint)
						removeConstraint(j->m_constraint);
		}

		m_systems.clear();

		auto solver = m_constraintSolver;
		m_constraintSolver = nullptr;
		delete solver;
	}

	void SkinnedMeshWorld::addSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		if (std::find(m_systems.begin(), m_systems.end(), system) != m_systems.end()) {
			return;
		}

		m_systems.push_back(hdt::make_smart(system));
		for (int i = 0; i < system->m_meshes.size(); ++i) {
			addCollisionObject(system->m_meshes[i].get(), 1, 1);
		}

		for (int i = 0; i < system->m_bones.size(); ++i) {
			system->m_bones[i]->m_rig.setActivationState(DISABLE_DEACTIVATION);
			// 0,0 mask disables the collision of this object on Bullet.
			addRigidBody(&system->m_bones[i]->m_rig, 0, 0);
		}

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				addConstraint(j->m_constraint, true);

		for (int i = 0; i < system->m_constraints.size(); ++i)
			addConstraint(system->m_constraints[i]->m_constraint, true);

		// -10 allows RESET_PHYSICS down the calls. But equality with a float?...
		system->readTransform(system->prepareForRead(RESET_PHYSICS));

		system->m_world = this;
	}

	void SkinnedMeshWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* system)
	{
		auto idx = std::find(m_systems.begin(), m_systems.end(), system);
		if (idx == m_systems.end())
			return;

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				if (j->m_constraint)
					removeConstraint(j->m_constraint);

		for (int i = 0; i < system->m_meshes.size(); ++i)
			removeCollisionObject(system->m_meshes[i].get());
		for (int i = 0; i < system->m_constraints.size(); ++i)
			if (system->m_constraints[i]->m_constraint)
				removeConstraint(system->m_constraints[i]->m_constraint);
		for (int i = 0; i < system->m_bones.size(); ++i)
			removeRigidBody(&system->m_bones[i]->m_rig);

		std::swap(*idx, m_systems.back());
		m_systems.pop_back();

		system->m_world = nullptr;
	}

	void SkinnedMeshWorld::updateConstraintsForBone(SkinnedMeshBone* bone)
	{
		if (!bone)
			return;

		int numConstraints = int(m_constraints.size());
		for (int i = 0; i < numConstraints; i++) {
			btTypedConstraint* constraint = m_constraints[i];

			if (&constraint->getRigidBodyA() == &bone->m_rig ||
				&constraint->getRigidBodyB() == &bone->m_rig) {
				bool bothKinematic = constraint->getRigidBodyA().isStaticOrKinematicObject() &&
				                     constraint->getRigidBodyB().isStaticOrKinematicObject();

				constraint->setEnabled(!bothKinematic);
			}
		}
	}

	int SkinnedMeshWorld::stepSimulation(btScalar remainingTimeStep, int, btScalar fixedTimeStep)
	{
		applyGravity();
		if (hdt::SkyrimPhysicsWorld::get()->m_enableWind)
			applyWind();

		while (remainingTimeStep > fixedTimeStep) {
			internalSingleStepSimulation(fixedTimeStep);
			remainingTimeStep -= fixedTimeStep;
		}

		// For the sake of the bullet library, we don't manage a step that would be lower than a 300Hz frame.
		// Review this when (screens / Skyrim) will allow 300Hz+.
		// Note: We are taking a final variable-sized step for the remaining time.
		// Because Bullet's constraint solvers (ERP/CFM) are sensitive to delta-time,
		// this variable tick can cause constraints to behave a bit differently
		// (appearing more stiff or damping differently at various framerates).
		constexpr auto minPossiblePeriod = 1.0f / 300.0f;
		if (remainingTimeStep > minPossiblePeriod)
			internalSingleStepSimulation(remainingTimeStep);
		clearForces();

		_bodies.clear();
		_shapes.clear();

		return 0;
	}

	// --Todo: This, and the systems related to it, can be optimized a bit more. I WILL BE BACK...
	// This optimizes Bullet's broadphase by removing tons of redudent work. We don't use persistent manifolds,
	// we don't have static objects, etc..
	// HOWEVER: If we ever add non-skinned Bullet collision objects, or make (0,0) bodies participate in
	// broadphase queries/collision, this optimization must be revisited.
	void SkinnedMeshWorld::performDiscreteCollisionDetection()
	{
		for (auto& system : m_systems) {
			system->internalUpdate();
		}

		btDispatcherInfo& dispatchInfo = getDispatchInfo();

		for (int i = 0; i < m_collisionObjects.size(); i++) {
			btCollisionObject* colObj = m_collisionObjects[i];
			btBroadphaseProxy* proxy = colObj->getBroadphaseHandle();

			if (proxy->m_collisionFilterGroup == 0 && proxy->m_collisionFilterMask == 0)
				continue;

			btVector3 minAabb, maxAabb;
			colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb, maxAabb);

			m_broadphasePairCache->setAabb(proxy, minAabb, maxAabb, m_dispatcher1);
		}

		m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);

		if (m_dispatcher1) {
			m_dispatcher1->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(), dispatchInfo, m_dispatcher1);
		}
	}

	void SkinnedMeshWorld::applyGravity()
	{
		for (auto& i : m_systems) {
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (!body->isStaticOrKinematicObject() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
					body->setGravity(m_gravity * j->m_gravityFactor);
				}
			}
		}

		btDiscreteDynamicsWorldMt::applyGravity();
	}

	void SkinnedMeshWorld::applyWind()
	{
		for (auto& i : m_systems) {
			auto system = static_cast<SkyrimSystem*>(i.get());
			if (btFuzzyZero(system->m_windFactor))  // skip any systems that aren't affected by wind
				continue;
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (!body->isStaticOrKinematicObject() && j->m_windFactor != 0.0f && (rand() % 5))  // apply randomly 80% of the time to desync wind across bones
				{
					body->applyCentralForce(m_windSpeed * j->m_windFactor * system->m_windFactor);
				}
			}
		}
	}

	void SkinnedMeshWorld::predictUnconstraintMotion(btScalar timeStep)
	{
		struct UpdaterPredictUnconstraintMotion : public btIParallelForBody
		{
			btScalar timeStep;
			btRigidBody** rigidBodies;

			void forLoop(int iBegin, int iEnd) const BT_OVERRIDE
			{
				for (int i = iBegin; i < iEnd; ++i) {
					btRigidBody* body = rigidBodies[i];

					// not realistic, just an approximate
					if (!body->isStaticOrKinematicObject())
						body->applyDamping(timeStep);

					body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
				}
			}
		};

		if (m_nonStaticRigidBodies.size() > 0) {
			UpdaterPredictUnconstraintMotion update;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];

			const int grainSize = 100;
			btParallelFor(0, m_nonStaticRigidBodies.size(), grainSize, update);
		}
	}

	void SkinnedMeshWorld::integrateTransforms(btScalar timeStep)
	{
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto body = m_collisionObjects[i];
			if (body->isKinematicObject()) {
				btTransformUtil::integrateTransform(
					body->getWorldTransform(),
					body->getInterpolationLinearVelocity(),
					body->getInterpolationAngularVelocity(),
					timeStep,
					body->getInterpolationWorldTransform());
				body->setWorldTransform(body->getInterpolationWorldTransform());
			}
		}

		btVector3 limitMin(-1e+9f, -1e+9f, -1e+9f);
		btVector3 limitMax(1e+9f, 1e+9f, 1e+9f);
		for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
			btRigidBody* body = m_nonStaticRigidBodies[i];
			auto lv = body->getLinearVelocity();
			lv.setMax(limitMin);
			lv.setMin(limitMax);
			body->setLinearVelocity(lv);

			auto av = body->getAngularVelocity();
			av.setMax(limitMin);
			av.setMin(limitMax);
			body->setAngularVelocity(av);
		}

		btDiscreteDynamicsWorldMt::integrateTransforms(timeStep);
	}

	void SkinnedMeshWorld::calculateSimulationIslands()
	{
		BT_PROFILE("calculateSimulationIslands");
		getSimulationIslandManager()->updateActivationState(getCollisionWorld(), getCollisionWorld()->getDispatcher());

		auto unionFind = &getSimulationIslandManager()->getUnionFind();

		for (int i = 0; i < m_predictiveManifolds.size(); i++) {
			btPersistentManifold* manifold = m_predictiveManifolds[i];
			const btCollisionObject* colObj0 = manifold->getBody0();
			const btCollisionObject* colObj1 = manifold->getBody1();
			if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
				((colObj1) && (!(colObj1)->isStaticOrKinematicObject()))) {
				unionFind->unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
			}
		}

		int numConstraints = int(m_constraints.size());
		for (int i = 0; i < numConstraints; i++) {
			btTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled()) {
				const btRigidBody* colObj0 = &constraint->getRigidBodyA();
				const btRigidBody* colObj1 = &constraint->getRigidBodyB();
				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject()))) {
					unionFind->unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
				}
			}
		}

		// If two dynamic bones are colliding, they MUST be processed by the same island! Without this the solver will cause an
		// EXCEPTION_ACCESS_VIOLATION reading m_tmpSolverBodyPool :(
		btDispatcher* dispatcher = getCollisionWorld()->getDispatcher();
		int numManifolds = dispatcher->getNumManifolds();
		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

			// Only unite if they are actively generating contact points
			if (manifold->getNumContacts() > 0) {
				const btCollisionObject* colObj0 = static_cast<const btCollisionObject*>(manifold->getBody0());
				const btCollisionObject* colObj1 = static_cast<const btCollisionObject*>(manifold->getBody1());

				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject()))) {
					unionFind->unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
				}
			}
		}

		getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());
	}

	// Island-based constraint solving...
	// btDiscreteDynamicsWorldMt::solveConstraints decomposes the world into independent
	// simulation islands and dispatches each to a solver from the pool on separate threads.
	void SkinnedMeshWorld::solveConstraints(btContactSolverInfo& solverInfo)
	{
		BT_PROFILE("solveConstraints");
		if (!m_collisionObjects.size())
			return;

		// Kinematic objects should never be able to collide, or move. Because: They're kinematic?
		// This avoids broken configs, API misuse - etc. Better to check it every cycle for safety
		for (int i = 0; i < m_constraints.size(); i++) {
			btTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled()) {
				if (constraint->getRigidBodyA().isStaticOrKinematicObject() &&
					constraint->getRigidBodyB().isStaticOrKinematicObject()) {
					constraint->setEnabled(false);
				}
			}
		}

		btDiscreteDynamicsWorldMt::solveConstraints(solverInfo);

		// the HDT manifolds are still recreated every frame, clear to prevent stale data.
		static_cast<CollisionDispatcher*>(m_dispatcher1)->clearAllManifold();
	}
}
