// End-to-end replay-pipeline tests (the synthetic half of §15 - no in-game fixture needed). These
// build a tiny world in memory, round-trip it through the format, rebuild it via ReplaySystem, step
// it, and assert it produces finite output - exercising the whole T4 pipeline + scene-log churn.

#include "Replay/hdtReplayFormat.h"
#include "hdtReplaySystem.h"
#include "replayRunner.h"

#include <array>
#include <cmath>
#include <doctest/doctest.h>

// Only the replay namespace is opened: hdt::Bone (a core type) would otherwise shadow replay::Bone.
using namespace hdt::replay;

namespace
{
	// A 2-bone system: a kinematic driver and a dynamic sphere joined by a 6DOF constraint.
	Document makeSyntheticDoc(int nframes)
	{
		Document doc;

		Snapshot snap;
		snap.systemId = 1;

		CollisionShape sphere;
		sphere.type = ShapeType::Sphere;
		sphere.radius = 2.0f;
		sphere.margin = 0.1f;
		snap.shapes.push_back(sphere);

		Bone driver;
		driver.name = "driver";
		driver.mass = 0.0f;
		driver.invMass = 0.0f;
		driver.shapeIndex = -1;
		driver.kinematic = 1;
		snap.bones.push_back(driver);

		Bone dyn;
		dyn.name = "dyn";
		dyn.mass = 1.0f;
		dyn.invMass = 1.0f;
		dyn.invInertiaDiagLocal = { 0.5f, 0.5f, 0.5f };
		dyn.shapeIndex = 0;
		dyn.kinematic = 0;
		dyn.gravityFactor = 1.0f;
		dyn.initialWorldTransform.origin = { 0, 0, -5 };
		snap.bones.push_back(dyn);

		Constraint c;
		c.type = ConstraintType::Generic6Dof;
		c.boneA = 0;
		c.boneB = 1;
		c.linearLowerLimit = { 0, 0, 0 };
		c.linearUpperLimit = { 0, 0, 0 };
		c.angularLowerLimit = { -1, -1, -1 };
		c.angularUpperLimit = { 1, 1, 1 };
		c.linearStiffness = { 100, 100, 100 };
		c.linearDamping = { 1, 1, 1 };
		snap.constraints.push_back(c);

		// A mesh body skinned to the dynamic bone, so the step actually runs collision detection.
		MeshBody body;
		body.name = "cloth";
		body.shapeKind = BodyShapeKind::PerVertex;
		body.margin = 1.0f;
		BodySkinnedBone sb;
		sb.boneIndex = 1;
		sb.vertexToBone.scale = 1.0f;
		sb.localBoundingSphere = SphereVolume{ { 0, 0, 0 }, 5.0f };
		body.skinnedBones.push_back(sb);
		for (int i = 0; i < 4; ++i) {
			BodyVertex v;
			v.skinPos = { static_cast<float>(i), 0.0f, 0.0f, 0.0f };
			v.weight[0] = 1.0f;
			v.boneIdx[0] = 0;
			body.vertices.push_back(v);
		}
		snap.bodies.push_back(body);

		doc.initialSystems.push_back(snap);

		for (int i = 0; i < nframes; ++i) {
			Frame f;
			f.remainingTimeStep = 1.0f / 60.0f;
			f.tick = 1.0f / 60.0f;
			f.reset = (i == 0) ? 1 : 0;
			QsTransform t;
			t.origin = { std::sin(i * 0.1f), 0.0f, 0.0f };
			f.kinematicTargets.push_back(BoneTarget{ 1, 0, t });
			doc.frames.push_back(f);
		}
		return doc;
	}
}

TEST_CASE("replay pipeline: serialize -> load -> build -> step yields finite output")
{
	Document doc = makeSyntheticDoc(40);
	std::vector<uint8_t> bytes = serialize(doc);
	Document loaded = deserialize(bytes);
	REQUIRE(loaded == doc);

	setFlushToZero();
	LoadedWorld lw;
	initWorld(lw, loaded);
	REQUIRE(lw.systems.size() == 1);

	hdt::ReplaySystem* sys = lw.systems[1].get();
	REQUIRE(sys != nullptr);
	REQUIRE(sys->boneCount() == 2);
	CHECK(sys->bodyCount() == 1);

	PhaseTimings t = runReplay(lw, loaded, 0, 0);
	CHECK(t.timedFrames() == loaded.frames.size());
	CHECK(t.read.size() == loaded.frames.size());
	CHECK(t.write.size() == loaded.frames.size());

	hdt::ReplayBone* dyn = sys->bone(1);
	REQUIRE(dyn != nullptr);
	auto o = dyn->m_currentTransform.getOrigin();
	CHECK(std::isfinite(o.x()));
	CHECK(std::isfinite(o.y()));
	CHECK(std::isfinite(o.z()));
}

TEST_CASE("scene-log: add then remove builds and destroys a system at the right frames")
{
	Document doc = makeSyntheticDoc(10);

	Snapshot snap2 = doc.initialSystems[0];
	snap2.systemId = 2;

	SceneEvent add;
	add.kind = SceneEventKind::AddSystem;
	add.frame = 2;
	add.systemId = 2;
	add.snapshot = snap2;
	doc.sceneLog.push_back(add);

	SceneEvent rem;
	rem.kind = SceneEventKind::RemoveSystem;
	rem.frame = 5;
	rem.systemId = 2;
	doc.sceneLog.push_back(rem);

	setFlushToZero();
	LoadedWorld lw;
	initWorld(lw, doc);
	CHECK(lw.systems.count(2) == 0);

	applySceneEvents(lw, doc, 2);
	CHECK(lw.systems.count(2) == 1);

	applySceneEvents(lw, doc, 5);
	CHECK(lw.systems.count(2) == 0);
}

TEST_CASE("determinism: single-thread replay twice yields identical output")
{
	Document doc = makeSyntheticDoc(60);

	auto runOnce = [&]() {
		setFlushToZero();
		LoadedWorld lw;
		initWorld(lw, doc);
		runReplay(lw, doc, 0, 0);
		auto o = lw.systems[1]->bone(1)->m_currentTransform.getOrigin();
		return std::array<float, 3>{ o.x(), o.y(), o.z() };
	};

	auto a = runOnce();
	auto b = runOnce();
	CHECK(a[0] == doctest::Approx(b[0]));
	CHECK(a[1] == doctest::Approx(b[1]));
	CHECK(a[2] == doctest::Approx(b[2]));
}
