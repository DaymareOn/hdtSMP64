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

		SkinnedMeshAlgorithm::registerAlgorithm(collisionDispatcher);

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

		// Just a note, this was created by us and the base constructor wont delete it.
		// This is just a hacky workaround that also avoids a dangling pointer..
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
			addRigidBody(&system->m_bones[i]->m_rig, 0, 0);
		}

		// Safety: validate constraints before adding.
		// This shouldn't be needed, but exists just in case.
		// Was added due to crashing with the batch solver, but didn't fix that either.
		// Exists purely as a means of debugging for now, but should probably be removed on official release
		auto isBodyInWorld = [this](const btRigidBody& body) -> bool {
			for (int k = 0; k < m_collisionObjects.size(); ++k) {
				if (m_collisionObjects[k] == &body)
					return true;
			}
			return false;
		};

		int skippedConstraintCount = 0;

		auto addValidatedConstraint = [&](btTypedConstraint* constraint) {
			if (!constraint)
				return;
			if (!isBodyInWorld(constraint->getRigidBodyA()) ||
				!isBodyInWorld(constraint->getRigidBodyB())) {
				skippedConstraintCount++;
				return;
			}
			addConstraint(constraint, true);
		};

		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				addValidatedConstraint(j->m_constraint);

		for (int i = 0; i < system->m_constraints.size(); ++i)
			addValidatedConstraint(system->m_constraints[i]->m_constraint);

		if (skippedConstraintCount > 0) {
			logger::warn("[MeshWorld] Skipped {} constraints with missing bodies (bones without collision shapes?)",
				skippedConstraintCount);
		}

		// -10 allows RESET_PHYSICS down the calls. But equality with a float?...
		system->readTransform(RESET_PHYSICS);

		system->m_world = this;

		// Diagnostics, TODO: Remove this or put it under a compile rule
		int totalConstraints = 0;
		int skippedConstraints = 0;
		for (auto i : system->m_constraintGroups)
			for (auto j : i->m_constraints)
				totalConstraints++;
		totalConstraints += static_cast<int>(system->m_constraints.size());

		int worldConstraints = static_cast<int>(m_constraints.size());

		logger::info(
			"[MeshWorld] Added system: bones={}, meshes={}, total_constraints_attempted={}, "
			"world_constraints_after={}, collision_objects={}",
			system->m_bones.size(),
			system->m_meshes.size(),
			totalConstraints,
			worldConstraints,
			m_collisionObjects.size());
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
		constexpr auto minPossiblePeriod = 1.0f / 300.0f;
		if (remainingTimeStep > minPossiblePeriod)
			internalSingleStepSimulation(remainingTimeStep);
		clearForces();

		_bodies.clear();
		_shapes.clear();

		return 0;
	}

	void SkinnedMeshWorld::performDiscreteCollisionDetection()
	{
		for (auto& system : m_systems)
			system->internalUpdate();

		btDiscreteDynamicsWorldMt::performDiscreteCollisionDetection();
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
			if (btFuzzyZero(system->m_windFactor))
				continue;
			for (auto& j : i->m_bones) {
				auto body = &j->m_rig;
				if (!body->isStaticOrKinematicObject() && (rand() % 5)) {
					body->applyCentralForce(m_windSpeed * j->m_windFactor * system->m_windFactor);
				}
			}
		}
	}

	// Perf test: Parallelize unconstrained motion prediction
	// Todo: Uhh profile and see if this is just adding latency/overhead
	void SkinnedMeshWorld::predictUnconstraintMotion(btScalar timeStep)
	{
		BT_PROFILE("predictUnconstraintMotion");
		if (m_nonStaticRigidBodies.size() == 0)
			return;

		struct Updater : public btIParallelForBody
		{
			btScalar timeStep;
			btRigidBody** rigidBodies;

			void forLoop(int iBegin, int iEnd) const BT_OVERRIDE
			{
				for (int i = iBegin; i < iEnd; ++i) {
					btRigidBody* body = rigidBodies[i];
					if (!body->isStaticOrKinematicObject()) {
						body->applyDamping(timeStep);
					}
					body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
				}
			}
		};

		Updater update;
		update.timeStep = timeStep;
		update.rigidBodies = &m_nonStaticRigidBodies[0];
		btParallelFor(0, m_nonStaticRigidBodies.size(), 50, update);
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

	void SkinnedMeshWorld::logIslandDiagnostics()
	{
		static int frameCounter = 0;
		static constexpr int LOG_INTERVAL = 300;
		frameCounter++;
		if (frameCounter % LOG_INTERVAL != 0)
			return;

		auto* im = getSimulationIslandManager();
		if (!im)
			return;

		auto& unionFind = im->getUnionFind();
		int numElements = unionFind.getNumElements();

		// Count distinct island roots
		std::unordered_set<int> islandIds;
		int sleepingBodies = 0;
		int activeBodies = 0;

		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			auto* obj = m_collisionObjects[i];
			int tag = obj->getIslandTag();
			if (tag >= 0) {
				islandIds.insert(unionFind.find(tag));
				if (obj->isActive())
					activeBodies++;
				else
					sleepingBodies++;
			}
		}

		int numManifolds = m_dispatcher1 ? m_dispatcher1->getNumManifolds() : 0;
		int numConstraints = static_cast<int>(m_constraints.size());
		int numSystems = static_cast<int>(m_systems.size());

		// Count bodies per island for distribution info
		std::unordered_map<int, int> bodiesPerIsland;
		for (int i = 0; i < m_collisionObjects.size(); ++i) {
			int tag = m_collisionObjects[i]->getIslandTag();
			if (tag >= 0)
				bodiesPerIsland[unionFind.find(tag)]++;
		}

		int largestIsland = 0;
		int smallestIsland = INT_MAX;
		for (auto& [id, count] : bodiesPerIsland) {
			largestIsland = std::max(largestIsland, count);
			smallestIsland = std::min(smallestIsland, count);
		}
		if (bodiesPerIsland.empty())
			smallestIsland = 0;

		logger::info("[MeshWorld Diagnostics] Frame {}", frameCounter);
		logger::info("  Systems: {}", numSystems);
		logger::info("  Collision objects: {} (active: {}, sleeping: {})",
			m_collisionObjects.size(), activeBodies, sleepingBodies);
		logger::info("  Non-static rigid bodies: {}", m_nonStaticRigidBodies.size());
		logger::info("  Constraints: {}", numConstraints);
		logger::info("  Contact manifolds: {}", numManifolds);
		logger::info("  Simulation islands: {}", islandIds.size());
		logger::info("  Island sizes: smallest={}, largest={}", smallestIsland, largestIsland);
	}

	// Island-based constraint solving...
	// btDiscreteDynamicsWorldMt::solveConstraints decomposes the world into independent
	// simulation islands and dispatches each to a solver from the pool on separate threads.
	void SkinnedMeshWorld::solveConstraints(btContactSolverInfo& solverInfo)
	{
		BT_PROFILE("solveConstraints");
		if (!m_collisionObjects.size())
			return;

		// Diagnostics — logs island count periodically to verify parallel dispatch
		logIslandDiagnostics();

		btDiscreteDynamicsWorldMt::solveConstraints(solverInfo);

		// the HDT manifolds are still recreated every frame, clear to prevent stale data.
		static_cast<CollisionDispatcher*>(m_dispatcher1)->clearAllManifold();
	}
}
