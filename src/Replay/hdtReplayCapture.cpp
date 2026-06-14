#include "hdtReplayCapture.h"

#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "hdtSkinnedMesh/hdtStiffSpringConstraint.h"

#include <fstream>
#include <functional>
#include <unordered_map>

namespace hdt::replay
{
	namespace
	{
		// Cheap serialized-size estimate for one frame, used only for the size cap. We serialize the
		// frame into a throwaway writer; this is O(frame) and the size cap path is not hot.
		size_t estimateFrameSize(const Frame& f)
		{
			ByteWriter w;
			writeFrame(w, f);
			return w.data().size();
		}

		size_t estimateSnapshotSize(const Snapshot& s)
		{
			ByteWriter w;
			writeSnapshot(w, s);
			return w.data().size();
		}
	}

	Snapshot buildSnapshot(SkinnedMeshSystem& system, uint32_t systemId)
	{
		Snapshot snap;
		snap.systemId = systemId;

		// --- collision shapes (deduped by pointer; compounds recurse, children first) ---
		std::unordered_map<const btCollisionShape*, int> shapeIdx;
		std::function<int(const btCollisionShape*)> internShape = [&](const btCollisionShape* s) -> int {
			if (!s || s->getShapeType() == EMPTY_SHAPE_PROXYTYPE)
				return -1;
			auto found = shapeIdx.find(s);
			if (found != shapeIdx.end())
				return found->second;

			CollisionShape cs;
			cs.margin = s->getMargin();
			cs.localScaling = toVec3(s->getLocalScaling());
			switch (s->getShapeType()) {
			case BOX_SHAPE_PROXYTYPE:
				cs.type = ShapeType::Box;
				cs.halfExtents = toVec3(static_cast<const btBoxShape*>(s)->getHalfExtentsWithoutMargin());
				break;
			case SPHERE_SHAPE_PROXYTYPE:
				cs.type = ShapeType::Sphere;
				cs.radius = static_cast<const btSphereShape*>(s)->getRadius();
				break;
			case CAPSULE_SHAPE_PROXYTYPE: {
				auto* c = static_cast<const btCapsuleShape*>(s);
				cs.type = ShapeType::Capsule;
				cs.radius = c->getRadius();
				cs.height = 2.0f * c->getHalfHeight();
				break;
			}
			case CYLINDER_SHAPE_PROXYTYPE:
				cs.type = ShapeType::Cylinder;
				cs.halfExtents = toVec3(static_cast<const btCylinderShape*>(s)->getHalfExtentsWithoutMargin());
				break;
			case CONVEX_HULL_SHAPE_PROXYTYPE: {
				auto* h = static_cast<const btConvexHullShape*>(s);
				cs.type = ShapeType::ConvexHull;
				const btVector3* pts = h->getUnscaledPoints();
				for (int i = 0; i < h->getNumPoints(); ++i)
					cs.hullPoints.push_back(toVec3(pts[i]));
				break;
			}
			case COMPOUND_SHAPE_PROXYTYPE: {
				auto* cp = static_cast<const btCompoundShape*>(s);
				cs.type = ShapeType::Compound;
				for (int i = 0; i < cp->getNumChildShapes(); ++i) {
					int childIdx = internShape(cp->getChildShape(i));
					if (childIdx >= 0)
						cs.children.push_back(CompoundChild{ toTransform(cp->getChildTransform(i)), static_cast<uint32_t>(childIdx) });
				}
				break;
			}
			default:
				return -1;  // unsupported shape: leave the bone shape-less rather than guess
			}

			int id = static_cast<int>(snap.shapes.size());
			snap.shapes.push_back(std::move(cs));
			shapeIdx[s] = id;
			return id;
		};

		// --- bones ---
		std::unordered_map<const SkinnedMeshBone*, uint32_t> boneIdx;
		const auto& bones = system.bonesView();
		for (uint32_t i = 0; i < bones.size(); ++i) {
			SkinnedMeshBone* bone = bones[i].get();
			if (!bone)
				continue;
			boneIdx[bone] = static_cast<uint32_t>(snap.bones.size());

			Bone b;
			b.name = bone->m_name.c_str();
			const float invMass = bone->m_rig.getInvMass();
			b.invMass = invMass;
			b.mass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
			b.invInertiaDiagLocal = toVec3(bone->m_rig.getInvInertiaDiagLocal());
			b.localToRig = toTransform(bone->m_localToRig);
			b.rigToLocal = toTransform(bone->m_rigToLocal);
			b.marginMultiplier = bone->m_marginMultipler;
			b.boundingSphereMultiplier = bone->m_boudingSphereMultipler;
			b.gravityFactor = bone->m_gravityFactor;
			b.windFactor = bone->m_windFactor;
			b.kinematic = bone->m_rig.isStaticOrKinematicObject() ? 1 : 0;
			b.shapeIndex = internShape(bone->m_rig.getCollisionShape());
			b.initialWorldTransform = toQs(bone->m_currentTransform);
			for (const auto& n : bone->m_canCollideWithBone)
				b.canCollideWithBone.emplace_back(n.c_str());
			for (const auto& n : bone->m_noCollideWithBone)
				b.noCollideWithBone.emplace_back(n.c_str());
			snap.bones.push_back(std::move(b));
		}

		// --- mesh bodies (collision geometry: vertices + skinning + collider shape) ---
		// vertexToBone is stored on the live body as a btMatrix4x3T; qsFromMatrix4x3T recovers the
		// originating btQsTransform so ReplaySystem::build can re-feed it through addBone unchanged.
		for (const auto& bodyPtr : system.meshesView()) {
			SkinnedMeshBody* body = bodyPtr.get();
			if (!body || !body->m_shape)
				continue;

			MeshBody mb;
			mb.name = body->m_name.c_str();

			SkinnedMeshShape* shape = body->m_shape.get();
			if (PerTriangleShape* pt = shape->asPerTriangleShape()) {
				mb.shapeKind = BodyShapeKind::PerTriangle;
				mb.margin = pt->m_shapeProp.margin;
				mb.penetration = pt->m_shapeProp.penetration;
				mb.triangleIndices.reserve(pt->m_colliders.size() * 3);
				for (const auto& col : pt->m_colliders) {
					mb.triangleIndices.push_back(col.vertices[0]);
					mb.triangleIndices.push_back(col.vertices[1]);
					mb.triangleIndices.push_back(col.vertices[2]);
				}
			} else {
				mb.shapeKind = BodyShapeKind::PerVertex;
				PerVertexShape* pv = shape->asPerVertexShape();
				mb.margin = pv ? pv->m_shapeProp.margin : 1.0f;
			}

			mb.vertices.reserve(body->m_vertices.size());
			for (const auto& v : body->m_vertices) {
				BodyVertex bv;
				bv.skinPos = Vec4{ v.m_skinPos.x(), v.m_skinPos.y(), v.m_skinPos.z(), 0.0f };
				for (int k = 0; k < 4; ++k) {
					bv.weight[k] = v.m_weight[k];
					bv.boneIdx[k] = v.m_boneIdx[k];  // index into this body's skinnedBones
				}
				mb.vertices.push_back(bv);
			}

			mb.skinnedBones.reserve(body->m_skinnedBones.size());
			for (const auto& sb : body->m_skinnedBones) {
				BodySkinnedBone out;
				auto it = boneIdx.find(sb.ptr);
				out.boneIndex = it != boneIdx.end() ? it->second : 0;
				out.vertexToBone = qsFromMatrix4x3T(sb.vertexToBone);
				out.localBoundingSphere = SphereVolume{ toVec3(sb.localBoundingSphere.center()), sb.localBoundingSphere.radius() };
				out.weightThreshold = sb.weightThreshold;
				out.isKinematic = sb.isKinematic ? 1 : 0;
				mb.skinnedBones.push_back(out);
			}

			for (const auto& t : body->m_tags)
				mb.tags.emplace_back(t.c_str());
			for (const auto& t : body->m_canCollideWithTags)
				mb.canCollideWithTags.emplace_back(t.c_str());
			for (const auto& t : body->m_noCollideWithTags)
				mb.noCollideWithTags.emplace_back(t.c_str());
			for (auto* bp : body->m_canCollideWithBones)
				if (bp)
					mb.canCollideWithBones.emplace_back(bp->m_name.c_str());
			for (auto* bp : body->m_noCollideWithBones)
				if (bp)
					mb.noCollideWithBones.emplace_back(bp->m_name.c_str());

			snap.bodies.push_back(std::move(mb));
		}

		// --- constraints (frames recovered to the captured frameInA/B convention) ---
		for (const auto& bscPtr : system.constraintsView()) {
			hdt::BoneScaleConstraint* bsc = bscPtr.get();
			if (!bsc)
				continue;
			auto ia = boneIdx.find(bsc->m_boneA);
			auto ib = boneIdx.find(bsc->m_boneB);
			if (ia == boneIdx.end() || ib == boneIdx.end())
				continue;

			Constraint c;
			c.boneA = ia->second;
			c.boneB = ib->second;

			if (auto* g = dynamic_cast<Generic6DofConstraint*>(bsc)) {
				c.type = ConstraintType::Generic6Dof;
				c.frameInA = toTransform(bsc->m_boneA->m_localToRig * g->getFrameOffsetA());
				c.frameInB = toTransform(bsc->m_boneB->m_localToRig * g->getFrameOffsetB());
				const auto* lin = g->getTranslationalLimitMotor();
				c.linearLowerLimit = toVec3(lin->m_lowerLimit);
				c.linearUpperLimit = toVec3(lin->m_upperLimit);
				c.linearStiffness = toVec3(lin->m_springStiffness);
				c.linearDamping = toVec3(lin->m_springDamping);
				c.linearEquilibrium = toVec3(lin->m_equilibriumPoint);
				for (int i = 0; i < 3; ++i) {
					const auto* rot = g->getRotationalLimitMotor(i);
					(&c.angularLowerLimit.x)[i] = rot->m_loLimit;
					(&c.angularUpperLimit.x)[i] = rot->m_hiLimit;
					(&c.angularStiffness.x)[i] = rot->m_springStiffness;
					(&c.angularDamping.x)[i] = rot->m_springDamping;
					(&c.angularEquilibrium.x)[i] = rot->m_equilibriumPoint;
				}
			} else if (auto* ct = dynamic_cast<ConeTwistConstraint*>(bsc)) {
				c.type = ConstraintType::ConeTwist;
				c.frameInA = toTransform(bsc->m_boneA->m_localToRig * ct->getAFrame());
				c.frameInB = toTransform(bsc->m_boneB->m_localToRig * ct->getBFrame());
				c.swingSpan1 = ct->getSwingSpan1();
				c.swingSpan2 = ct->getSwingSpan2();
				c.twistSpan = ct->getTwistSpan();
			} else if (auto* ss = dynamic_cast<StiffSpringConstraint*>(bsc)) {
				c.type = ConstraintType::StiffSpring;
				c.minDistance = ss->m_minDistance;
				c.maxDistance = ss->m_maxDistance;
				c.stiffness = ss->m_stiffness;
				c.damping = ss->m_damping;
				c.equilibrium = ss->m_equilibriumPoint;
			} else {
				continue;
			}
			snap.constraints.push_back(std::move(c));
		}

		return snap;
	}

	void captureFrameBones(const SkinnedMeshSystem& system, uint32_t systemId, Frame& frame, bool golden)
	{
		// Intended call site: in doUpdate2ndStep, immediately after stepSimulation and before
		// writeTransform. At that point a kinematic (driver) bone's m_currentTransform still holds the
		// frame's input target (set in readTransform), while a dynamic bone's solved pose is the rigid
		// body's post-step world transform (writeTransform has not yet folded it back into
		// m_currentTransform), so we derive the golden output straight from the body - the same math
		// SkinnedMeshBone/SkyrimBone::writeTransform uses.
		const auto& bones = system.bonesView();
		for (uint32_t i = 0; i < bones.size(); ++i) {
			SkinnedMeshBone* bone = bones[i].get();
			if (!bone)
				continue;
			if (bone->m_rig.isStaticOrKinematicObject()) {
				frame.kinematicTargets.push_back(BoneTarget{ systemId, i, toQs(bone->m_currentTransform) });
			} else if (golden) {
				const btTransform t = bone->m_rig.getWorldTransform() * bone->m_rigToLocal;
				QsTransform out;
				out.basis = toQuat(t.getRotation());
				out.origin = toVec3(t.getOrigin());
				out.scale = bone->m_currentTransform.getScale();
				frame.golden.push_back(BoneOutput{ systemId, i, out });
			}
		}
	}

	void CaptureBuffer::begin(const SolverConfig& solver,
		std::vector<Snapshot> initialSystems,
		std::string gitSha,
		uint32_t configHash,
		uint32_t threadCountHint,
		bool golden,
		uint32_t frameCap,
		size_t sizeCap)
	{
		std::lock_guard<std::mutex> l(m_lock);
		m_doc = Document{};
		m_doc.header.formatVersion = kReplayFormatVersion;
		m_doc.header.gitSha = std::move(gitSha);
		m_doc.header.configHash = configHash;
		m_doc.header.threadCountHint = threadCountHint;
		m_doc.header.hasGolden = golden ? 1 : 0;
		m_doc.solver = solver;
		m_doc.initialSystems = std::move(initialSystems);

		m_approxSize = 0;
		for (const auto& s : m_doc.initialSystems)
			m_approxSize += estimateSnapshotSize(s);

		m_active = true;
		m_capReached = false;
		m_frameCap = frameCap;
		m_sizeCap = sizeCap;
	}

	void CaptureBuffer::addSceneEvent(SceneEvent event)
	{
		std::lock_guard<std::mutex> l(m_lock);
		if (!m_active)
			return;
		event.frame = static_cast<uint32_t>(m_doc.frames.size());
		if (event.kind == SceneEventKind::AddSystem)
			m_approxSize += estimateSnapshotSize(event.snapshot);
		m_doc.sceneLog.push_back(std::move(event));
	}

	bool CaptureBuffer::addFrame(Frame frame)
	{
		std::lock_guard<std::mutex> l(m_lock);
		if (!m_active)
			return false;

		m_approxSize += estimateFrameSize(frame);
		m_doc.frames.push_back(std::move(frame));

		const bool frameCapHit = m_frameCap != 0 && m_doc.frames.size() >= m_frameCap;
		const bool sizeCapHit = m_sizeCap != 0 && m_approxSize >= m_sizeCap;
		if (frameCapHit || sizeCapHit) {
			m_capReached = true;
			m_active = false;  // auto-stop (D9 fail-safe)
			return false;
		}
		return true;
	}

	bool CaptureBuffer::flushToFile(const std::string& path) const
	{
		std::vector<uint8_t> bytes;
		{
			std::lock_guard<std::mutex> l(m_lock);
			bytes = replay::serialize(m_doc);
		}

		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		if (!out)
			return false;
		out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
		return static_cast<bool>(out);
	}

	std::vector<uint8_t> CaptureBuffer::serialize() const
	{
		std::lock_guard<std::mutex> l(m_lock);
		return replay::serialize(m_doc);
	}

	void CaptureBuffer::reset()
	{
		std::lock_guard<std::mutex> l(m_lock);
		m_doc = Document{};
		m_active = false;
		m_capReached = false;
		m_approxSize = 0;
	}
}
