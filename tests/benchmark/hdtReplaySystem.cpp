#include "hdtReplaySystem.h"

#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
#include "hdtSkinnedMesh/hdtStiffSpringConstraint.h"

#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <unordered_map>

namespace hdt
{
	using namespace replay;

	// ---------------------------------------------------------------- ReplayBone

	ReplayBone::ReplayBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci) :
		SkinnedMeshBone(name, ci)
	{
	}

	void ReplayBone::readTransform(float timeStep)
	{
		// Driver bones replay their captured target through the shared core apply-math (D6); dynamic
		// bones have no target and are left to the solver.
		if (m_hasTarget)
			applyKinematicTarget(m_target, timeStep, m_reset);
	}

	void ReplayBone::writeTransform()
	{
		// Mirror of SkyrimBone::writeTransform minus the NiNode write: publish the post-step pose so
		// the harness can read it for the golden/parity comparison (D8).
		auto transform = m_rig.getWorldTransform() * m_rigToLocal;
		m_currentTransform.setBasis(transform.getBasis());
		m_currentTransform.setOrigin(transform.getOrigin());
	}

	void ReplayBone::setFrameTarget(const btQsTransform& target, bool reset)
	{
		m_target = target;
		m_reset = reset;
		m_hasTarget = true;
	}

	// ---------------------------------------------------------------- shape rebuild

	namespace
	{
		/// Shared empty collision shape for bones that carry no collider (shapeIndex < 0).
		btEmptyShape* emptyShape()
		{
			static btEmptyShape s_empty;
			return &s_empty;
		}

		/// Builds one non-compound collision shape. Compounds are built in a second pass (below) since
		/// they reference already-built children by index.
		std::shared_ptr<btCollisionShape> makeLeafShape(const CollisionShape& s)
		{
			std::shared_ptr<btCollisionShape> shape;
			switch (s.type) {
			case ShapeType::Box:
				shape = std::make_shared<btBoxShape>(toBt(s.halfExtents));
				break;
			case ShapeType::Sphere:
				shape = std::make_shared<btSphereShape>(s.radius);
				break;
			case ShapeType::Capsule:
				shape = std::make_shared<btCapsuleShape>(s.radius, s.height);
				break;
			case ShapeType::Cylinder:
				shape = std::make_shared<btCylinderShape>(toBt(s.halfExtents));
				break;
			case ShapeType::ConvexHull: {
				auto hull = std::make_shared<btConvexHullShape>();
				for (const auto& p : s.hullPoints)
					hull->addPoint(toBt(p), false);
				hull->recalcLocalAabb();
				shape = hull;
				break;
			}
			case ShapeType::Compound:
				return nullptr;  // handled in the second pass
			}
			if (shape) {
				shape->setMargin(s.margin);
				shape->setLocalScaling(toBt(s.localScaling));
			}
			return shape;
		}
	}

	// ---------------------------------------------------------------- ReplaySystem

	RE::BSTSmartPointer<ReplaySystem> ReplaySystem::build(const Snapshot& snap)
	{
		auto sysPtr = hdt::make_smart(new ReplaySystem());
		ReplaySystem* sys = sysPtr.get();
		sys->m_systemId = snap.systemId;

		// --- shapes (two passes so compounds can reference their children) ---
		std::vector<std::shared_ptr<btCollisionShape>> shapes(snap.shapes.size());
		for (std::size_t i = 0; i < snap.shapes.size(); ++i)
			shapes[i] = makeLeafShape(snap.shapes[i]);
		for (std::size_t i = 0; i < snap.shapes.size(); ++i) {
			const auto& s = snap.shapes[i];
			if (s.type != ShapeType::Compound)
				continue;
			auto compound = std::make_shared<btCompoundShape>();
			for (const auto& child : s.children) {
				if (child.shapeIndex < shapes.size() && shapes[child.shapeIndex])
					compound->addChildShape(toBtTransform(child.localTransform), shapes[child.shapeIndex].get());
			}
			compound->setMargin(s.margin);
			compound->setLocalScaling(toBt(s.localScaling));
			shapes[i] = compound;
		}
		sys->m_shapeRefs = shapes;  // keep alive for the system's lifetime

		// --- bones ---
		for (const auto& b : snap.bones) {
			btCollisionShape* shape = (b.shapeIndex >= 0 && static_cast<std::size_t>(b.shapeIndex) < shapes.size() && shapes[b.shapeIndex])
			                              ? shapes[b.shapeIndex].get()
			                              : static_cast<btCollisionShape*>(emptyShape());

			btRigidBody::btRigidBodyConstructionInfo ci(b.mass, nullptr, shape, btVector3(0, 0, 0));
			auto* bone = new ReplayBone(RE::BSFixedString(b.name.c_str()), ci);

			// Reproduce the captured mass properties exactly (the game derives these from the NIF).
			bone->m_rig.setMassProps(b.mass, btVector3(0, 0, 0));
			bone->m_rig.setInvInertiaDiagLocal(toBt(b.invInertiaDiagLocal));
			bone->m_rig.updateInertiaTensor();

			const auto flags = bone->m_rig.getCollisionFlags();
			if (b.kinematic)
				bone->m_rig.setCollisionFlags(flags | btCollisionObject::CF_KINEMATIC_OBJECT);
			else
				bone->m_rig.setCollisionFlags(flags & ~btCollisionObject::CF_KINEMATIC_OBJECT);
			bone->m_isDriver = b.kinematic != 0;

			bone->m_localToRig = toBtTransform(b.localToRig);
			bone->m_rigToLocal = toBtTransform(b.rigToLocal);
			bone->m_marginMultipler = b.marginMultiplier;
			bone->m_boudingSphereMultipler = b.boundingSphereMultiplier;
			bone->m_gravityFactor = b.gravityFactor;
			bone->m_windFactor = b.windFactor;
			for (const auto& n : b.canCollideWithBone)
				bone->m_canCollideWithBone.emplace_back(n.c_str());
			for (const auto& n : b.noCollideWithBone)
				bone->m_noCollideWithBone.emplace_back(n.c_str());

			// Place the body at its captured initial pose.
			bone->m_currentTransform = toBtQs(b.initialWorldTransform);
			auto dest = bone->m_currentTransform.asTransform() * bone->m_localToRig;
			bone->m_rig.setWorldTransform(dest);
			bone->m_rig.setInterpolationWorldTransform(dest);
			static const btVector3 zero(0, 0, 0);
			bone->m_rig.setLinearVelocity(zero);
			bone->m_rig.setAngularVelocity(zero);
			bone->m_rig.setInterpolationLinearVelocity(zero);
			bone->m_rig.setInterpolationAngularVelocity(zero);

			sys->m_bones.push_back(hdt::make_smart<SkinnedMeshBone>(static_cast<SkinnedMeshBone*>(bone)));
		}

		// --- mesh bodies (collision geometry; inverse of replay::buildSnapshot's body capture) ---
		std::unordered_map<std::string, SkinnedMeshBone*> byName;
		for (auto& bp : sys->m_bones)
			if (bp)
				byName[bp->m_name.c_str()] = bp.get();

		for (const auto& mb : snap.bodies) {
			auto bodyPtr = hdt::make_smart(new SkinnedMeshBody());
			SkinnedMeshBody* body = bodyPtr.get();
			body->m_name = RE::BSFixedString(mb.name.c_str());

			for (const auto& sb : mb.skinnedBones) {
				ReplayBone* bn = sys->bone(sb.boneIndex);
				if (!bn)
					continue;
				int idx = body->addBone(bn, toBtQs(sb.vertexToBone),
					BoundingSphere(toBt(sb.localBoundingSphere.center), sb.localBoundingSphere.radius));
				body->m_skinnedBones[idx].weightThreshold = sb.weightThreshold;
			}

			body->m_vertices.resize(mb.vertices.size());
			for (std::size_t i = 0; i < mb.vertices.size(); ++i) {
				Vertex& dst = body->m_vertices[i];
				const BodyVertex& src = mb.vertices[i];
				dst.m_skinPos.setValue(src.skinPos.x, src.skinPos.y, src.skinPos.z);
				for (int k = 0; k < 4; ++k) {
					dst.m_weight[k] = src.weight[k];
					dst.m_boneIdx[k] = src.boneIdx[k];
				}
			}

			for (const auto& t : mb.tags)
				body->m_tags.emplace_back(t.c_str());
			for (const auto& t : mb.canCollideWithTags)
				body->m_canCollideWithTags.insert(RE::BSFixedString(t.c_str()));
			for (const auto& t : mb.noCollideWithTags)
				body->m_noCollideWithTags.insert(RE::BSFixedString(t.c_str()));
			for (const auto& n : mb.canCollideWithBones) {
				auto it = byName.find(n);
				if (it != byName.end())
					body->m_canCollideWithBones.insert(it->second);
			}
			for (const auto& n : mb.noCollideWithBones) {
				auto it = byName.find(n);
				if (it != byName.end())
					body->m_noCollideWithBones.insert(it->second);
			}

			if (mb.shapeKind == BodyShapeKind::PerVertex) {
				auto shape = RE::make_smart<PerVertexShape>(body);  // ctor wires body->m_shape
				shape->m_shapeProp.margin = mb.margin;
				shape->autoGen();
				body->finishBuild();
			} else {
				auto shape = RE::make_smart<PerTriangleShape>(body);
				shape->m_shapeProp.margin = mb.margin;
				shape->m_shapeProp.penetration = mb.penetration;
				for (std::size_t t = 0; t + 2 < mb.triangleIndices.size(); t += 3)
					shape->addTriangle(static_cast<int>(mb.triangleIndices[t]),
						static_cast<int>(mb.triangleIndices[t + 1]),
						static_cast<int>(mb.triangleIndices[t + 2]));
				body->finishBuild();
			}

			sys->m_meshes.push_back(bodyPtr);
		}

		// --- constraints ---
		auto pushConstraint = [&](hdt::BoneScaleConstraint* con) {
			sys->m_constraints.push_back(hdt::make_smart<hdt::BoneScaleConstraint>(con));
		};

		for (const auto& c : snap.constraints) {
			ReplayBone* a = sys->bone(c.boneA);
			ReplayBone* b = sys->bone(c.boneB);
			if (!a || !b)
				continue;

			switch (c.type) {
			case ConstraintType::Generic6Dof: {
				auto* con = new Generic6DofConstraint(a, b, toBtTransform(c.frameInA), toBtTransform(c.frameInB));
				con->setLinearLowerLimit(toBt(c.linearLowerLimit));
				con->setLinearUpperLimit(toBt(c.linearUpperLimit));
				con->setAngularLowerLimit(toBt(c.angularLowerLimit));
				con->setAngularUpperLimit(toBt(c.angularUpperLimit));
				for (int i = 0; i < 3; ++i) {
					con->setStiffness(i, (&c.linearStiffness.x)[i]);
					con->setDamping(i, (&c.linearDamping.x)[i]);
					con->setEquilibriumPoint(i, (&c.linearEquilibrium.x)[i]);
					con->setStiffness(i + 3, (&c.angularStiffness.x)[i]);
					con->setDamping(i + 3, (&c.angularDamping.x)[i]);
					con->setEquilibriumPoint(i + 3, (&c.angularEquilibrium.x)[i]);
				}
				pushConstraint(con);
				break;
			}
			case ConstraintType::ConeTwist: {
				auto* con = new ConeTwistConstraint(a, b, toBtTransform(c.frameInA), toBtTransform(c.frameInB));
				con->setLimit(c.swingSpan1, c.swingSpan2, c.twistSpan, c.limitSoftness, c.biasFactor, c.relaxationFactor);
				pushConstraint(con);
				break;
			}
			case ConstraintType::StiffSpring: {
				auto* con = new StiffSpringConstraint(a, b);
				con->m_minDistance = c.minDistance;
				con->m_maxDistance = c.maxDistance;
				con->m_stiffness = c.stiffness;
				con->m_damping = c.damping;
				con->m_equilibriumPoint = c.equilibrium;
				pushConstraint(con);
				break;
			}
			}
		}

		return sysPtr;
	}

	void ReplaySystem::readTransform(float timeStep)
	{
		// Drive every bone; ReplayBone applies its captured target only if it is a driver.
		for (auto& b : m_bones)
			b->readTransform(timeStep);
	}

	// ---------------------------------------------------------------- ReplayWorld

	btVector3 ReplayWorld::applyTranslationOffset()
	{
		btVector3 center;
		center.setZero();
		int count = 0;
		auto& objs = getCollisionObjectArray();
		for (int i = 0; i < objs.size(); ++i) {
			auto rig = btRigidBody::upcast(objs[i]);
			if (rig) {
				center += rig->getWorldTransform().getOrigin();
				++count;
			}
		}
		if (count > 0) {
			center /= static_cast<btScalar>(count);
			for (int i = 0; i < objs.size(); ++i) {
				auto rig = btRigidBody::upcast(objs[i]);
				if (rig)
					rig->getWorldTransform().getOrigin() -= center;
			}
		}
		return center;
	}

	void ReplayWorld::restoreTranslationOffset(const btVector3& offset)
	{
		auto& objs = getCollisionObjectArray();
		for (int i = 0; i < objs.size(); ++i) {
			auto rig = btRigidBody::upcast(objs[i]);
			if (rig)
				rig->getWorldTransform().getOrigin() += offset;
		}
	}
}
