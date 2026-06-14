// Unit tests for the on-disk replay format (P0). These exercise hdtReplayFormat.h only and pull in
// no Bullet / CommonLibSSE / shim code, so they are the cheapest layer of the suite. Coverage:
//   - round-trip: a fully-populated Document survives serialize -> deserialize field-equal (§15)
//   - trust-boundary (mandatory): bad magic / wrong version / truncated / oversized count are all
//     rejected with a clean ReplayFormatError, never a crash or silent accept (global input rule).

#include "Replay/hdtReplayFormat.h"

#include <doctest/doctest.h>

using namespace hdt::replay;

namespace
{
	// Builds a Document touching every container and scalar so the round-trip test is meaningful.
	Document makeRichDocument()
	{
		Document doc;
		doc.header.gitSha = "0123456789abcdef0123456789abcdef01234567";
		doc.header.configHash = 0xDEADBEEF;
		doc.header.threadCountHint = 8;
		doc.header.hasGolden = 1;

		doc.solver.gravity = { 0.1f, 0.2f, -9.8f };
		doc.solver.friction = 0.0f;
		doc.solver.splitImpulse = 1;
		doc.solver.splitImpulsePenetrationThreshold = -0.01f;
		doc.solver.erp2 = 0.15f;
		doc.solver.globalCfm = 0.001f;
		doc.solver.restitutionVelocityThreshold = 0.2f;
		doc.solver.solverMode = 0x104;
		doc.solver.leastSquaresResidualThreshold = 0.0001f;
		doc.solver.timeTick = 1.0f / 60.0f;
		doc.solver.maxSubSteps = 4;
		doc.solver.enableWind = 1;
		doc.solver.windStrength = 2.0f;
		doc.solver.distanceForNoWind = 50.0f;
		doc.solver.distanceForMaxWind = 3000.0f;
		doc.solver.disableDeactivation = 1;

		Snapshot snap;
		snap.systemId = 7;
		snap.skeletonId = 70;
		snap.buildTimeMicros = 4500;

		CollisionShape box;
		box.type = ShapeType::Box;
		box.margin = 0.04f;
		box.halfExtents = { 1, 2, 3 };
		snap.shapes.push_back(box);

		CollisionShape hull;
		hull.type = ShapeType::ConvexHull;
		hull.hullPoints = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
		snap.shapes.push_back(hull);

		CollisionShape compound;
		compound.type = ShapeType::Compound;
		compound.children.push_back(CompoundChild{ Transform{ { 0, 0, 0, 1 }, { 5, 6, 7 } }, 0 });
		snap.shapes.push_back(compound);

		Bone b0;
		b0.name = "NPC Spine";
		b0.mass = 0.0f;
		b0.invMass = 0.0f;
		b0.shapeIndex = -1;
		b0.kinematic = 1;
		b0.initialWorldTransform.origin = { 1, 1, 1 };
		snap.bones.push_back(b0);

		Bone b1;
		b1.name = "Breast";
		b1.mass = 1.0f;
		b1.invMass = 1.0f;
		b1.invInertiaDiagLocal = { 0.5f, 0.5f, 0.5f };
		b1.shapeIndex = 0;
		b1.gravityFactor = 0.8f;
		b1.windFactor = 1.2f;
		b1.collisionFilter = 3;
		b1.canCollideWithBone = { "Belly" };
		b1.noCollideWithBone = { "NPC Spine", "Butt" };
		snap.bones.push_back(b1);

		MeshBody body;
		body.name = "BreastBody";
		body.shapeKind = BodyShapeKind::PerVertex;
		body.margin = 1.0f;
		BodyVertex v;
		v.skinPos = { 1, 2, 3, 1 };
		v.weight[0] = 1.0f;
		v.boneIdx[0] = 1;
		body.vertices.push_back(v);
		BodySkinnedBone sb;
		sb.boneIndex = 1;
		sb.vertexToBone.scale = 1.0f;
		sb.localBoundingSphere = SphereVolume{ { 0, 0, 0 }, 2.5f };
		sb.weightThreshold = 0.1f;
		body.skinnedBones.push_back(sb);
		body.tags = { "breast" };
		body.disableTag = "torso";
		body.disablePriority = 5;
		snap.bodies.push_back(body);

		Constraint c;
		c.type = ConstraintType::Generic6Dof;
		c.boneA = 0;
		c.boneB = 1;
		c.linearLowerLimit = { -1, -1, -1 };
		c.linearUpperLimit = { 1, 1, 1 };
		c.angularStiffness = { 10, 10, 10 };
		snap.constraints.push_back(c);

		ConstraintGroup g;
		g.constraintIndices = { 0 };
		snap.constraintGroups.push_back(g);

		snap.boneScaleConstraints.push_back(BoneScaleConstraint{ 1, 1.5f });

		doc.initialSystems.push_back(snap);

		// scene-log: add a second system at frame 1, remove it at frame 2
		SceneEvent add;
		add.kind = SceneEventKind::AddSystem;
		add.frame = 1;
		Snapshot snap2 = snap;
		snap2.systemId = 8;
		add.systemId = 8;
		add.snapshot = snap2;
		doc.sceneLog.push_back(add);

		SceneEvent rem;
		rem.kind = SceneEventKind::RemoveSystem;
		rem.frame = 2;
		rem.systemId = 8;
		doc.sceneLog.push_back(rem);

		// two frames
		Frame f0;
		f0.remainingTimeStep = 1.0f / 60.0f;
		f0.tick = 1.0f / 60.0f;
		f0.windSpeed = { 100, 0, 0 };
		f0.reset = 0;
		f0.kinematicTargets.push_back(BoneTarget{ 7, 0, QsTransform{ { 0, 0, 0, 1 }, { 1, 1, 1 }, 1.0f } });
		f0.disabled.push_back(BodyDisabled{ 7, 0, 0 });
		f0.golden.push_back(BoneOutput{ 7, 1, QsTransform{ { 0, 0, 0, 1 }, { 1.01f, 1, 1 }, 1.0f } });
		doc.frames.push_back(f0);

		Frame f1 = f0;
		f1.reset = 1;
		doc.frames.push_back(f1);

		return doc;
	}
}

TEST_CASE("round-trip preserves a fully-populated document")
{
	Document original = makeRichDocument();
	std::vector<uint8_t> bytes = serialize(original);
	Document restored = deserialize(bytes);
	CHECK(original == restored);
}

TEST_CASE("round-trip of an empty document")
{
	Document original;  // defaults only
	std::vector<uint8_t> bytes = serialize(original);
	Document restored = deserialize(bytes);
	CHECK(original == restored);
}

TEST_CASE("trust boundary: bad magic is rejected")
{
	Document doc = makeRichDocument();
	std::vector<uint8_t> bytes = serialize(doc);
	bytes[0] = 'X';  // corrupt the first magic byte
	CHECK_THROWS_AS(deserialize(bytes), ReplayFormatError);
}

TEST_CASE("trust boundary: wrong version is rejected")
{
	Document doc = makeRichDocument();
	std::vector<uint8_t> bytes = serialize(doc);
	// version u32 sits right after the 4 magic bytes; bump it past kReplayFormatVersion
	bytes[4] = static_cast<uint8_t>(kReplayFormatVersion + 1);
	CHECK_THROWS_AS(deserialize(bytes), ReplayFormatError);
}

TEST_CASE("trust boundary: truncated file is rejected")
{
	Document doc = makeRichDocument();
	std::vector<uint8_t> bytes = serialize(doc);
	bytes.resize(bytes.size() / 2);  // chop off the back half
	CHECK_THROWS_AS(deserialize(bytes), ReplayFormatError);
}

TEST_CASE("trust boundary: empty buffer is rejected")
{
	std::vector<uint8_t> empty;
	CHECK_THROWS_AS(deserialize(empty), ReplayFormatError);
}

TEST_CASE("trust boundary: an oversized count is rejected, not allocated")
{
	// Hand-build the smallest valid prefix (magic + version + header + solver + initialSystems count)
	// then inject a hostile count where the bones-count of the first snapshot would go.
	ByteWriter w;
	w.bytes(kReplayMagic, sizeof(kReplayMagic));
	w.u32(kReplayFormatVersion);
	w.fixedStr("", kGitShaLength);
	w.u32(0);  // configHash
	w.u32(1);  // threadCountHint
	w.u8(0);   // hasGolden
	// solver block (16 fields, values irrelevant)
	w.vec3({ 0, 0, 0 });
	w.f32(0); w.u8(0); w.f32(0); w.f32(0); w.f32(0); w.f32(0);
	w.i32(0); w.f32(0); w.f32(0); w.i32(0);
	w.u8(0); w.f32(0); w.f32(0); w.f32(0); w.u8(0);
	w.u32(1);            // initialSystems count = 1
	w.u32(0);            // snapshot.systemId
	w.u32(0);            // snapshot.skeletonId
	w.u32(0);            // snapshot.buildTimeMicros
	w.u32(0);            // shapes count = 0
	w.u32(0xFFFFFFFFu);  // bones count = ~4 billion -> must be rejected by count()
	std::vector<uint8_t> bytes = w.take();
	CHECK_THROWS_AS(deserialize(bytes), ReplayFormatError);
}

TEST_CASE("trust boundary: an oversized string length is rejected")
{
	ByteWriter w;
	w.bytes(kReplayMagic, sizeof(kReplayMagic));
	w.u32(kReplayFormatVersion);
	w.fixedStr("", kGitShaLength);
	w.u32(0);
	w.u32(1);
	w.u8(0);
	w.vec3({ 0, 0, 0 });
	w.f32(0); w.u8(0); w.f32(0); w.f32(0); w.f32(0); w.f32(0);
	w.i32(0); w.f32(0); w.f32(0); w.i32(0);
	w.u8(0); w.f32(0); w.f32(0); w.f32(0); w.u8(0);
	w.u32(1);            // initialSystems count
	w.u32(0);            // systemId
	w.u32(0);            // skeletonId
	w.u32(0);            // buildTimeMicros
	w.u32(0);            // shapes count
	w.u32(1);            // bones count = 1
	w.u32(0xFFFFFFFFu);  // first bone name length -> hostile, must be rejected
	std::vector<uint8_t> bytes = w.take();
	CHECK_THROWS_AS(deserialize(bytes), ReplayFormatError);
}
