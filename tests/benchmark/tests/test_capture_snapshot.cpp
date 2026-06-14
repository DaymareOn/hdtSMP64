// Snapshot-builder round-trip (the capture-side of the seam). replay::buildSnapshot reads a live
// core system back into an on-disk Snapshot - the inverse of ReplaySystem::build. Building a system
// from a snapshot and reading it back must preserve the bone/shape/constraint structure. This runs
// against the shim (no Skyrim layer needed) because buildSnapshot operates purely on core types.

#include "Replay/hdtReplayCapture.h"
#include "hdtReplaySystem.h"

#include <doctest/doctest.h>

using namespace hdt::replay;

namespace
{
	Snapshot makeSnapshot()
	{
		Snapshot s;
		s.systemId = 42;

		CollisionShape sphere;
		sphere.type = ShapeType::Sphere;
		sphere.radius = 2.0f;
		sphere.margin = 0.1f;
		s.shapes.push_back(sphere);

		Bone driver;
		driver.name = "driver";
		driver.shapeIndex = -1;
		driver.kinematic = 1;
		s.bones.push_back(driver);

		Bone dyn;
		dyn.name = "dyn";
		dyn.mass = 1.0f;
		dyn.invMass = 1.0f;
		dyn.invInertiaDiagLocal = { 0.5f, 0.5f, 0.5f };
		dyn.shapeIndex = 0;
		dyn.initialWorldTransform.origin = { 0, 0, -5 };
		s.bones.push_back(dyn);

		Constraint c;
		c.type = ConstraintType::Generic6Dof;
		c.boneA = 0;
		c.boneB = 1;
		c.angularLowerLimit = { -1, -1, -1 };
		c.angularUpperLimit = { 1, 1, 1 };
		s.constraints.push_back(c);

		// A per-vertex mesh body skinned entirely to the dynamic bone, three weighted vertices.
		MeshBody body;
		body.name = "cloth";
		body.shapeKind = BodyShapeKind::PerVertex;
		body.margin = 1.0f;
		BodySkinnedBone sb;
		sb.boneIndex = 1;  // the dynamic bone
		sb.vertexToBone.scale = 1.0f;
		sb.localBoundingSphere = SphereVolume{ { 0, 0, 0 }, 5.0f };
		body.skinnedBones.push_back(sb);
		for (int i = 0; i < 3; ++i) {
			BodyVertex v;
			v.skinPos = { static_cast<float>(i), 0.0f, 0.0f, 0.0f };
			v.weight[0] = 1.0f;
			v.boneIdx[0] = 0;  // local skinnedBones index
			body.vertices.push_back(v);
		}
		s.bodies.push_back(body);

		return s;
	}
}

TEST_CASE("buildSnapshot recovers bones, shapes and constraints from a rebuilt system")
{
	Snapshot orig = makeSnapshot();
	auto sys = hdt::ReplaySystem::build(orig);
	REQUIRE(sys);

	Snapshot rebuilt = buildSnapshot(*sys, orig.systemId);

	CHECK(rebuilt.systemId == orig.systemId);
	REQUIRE(rebuilt.bones.size() == orig.bones.size());
	REQUIRE(rebuilt.constraints.size() == orig.constraints.size());
	REQUIRE(rebuilt.shapes.size() >= 1);

	for (std::size_t i = 0; i < orig.bones.size(); ++i) {
		CHECK(rebuilt.bones[i].name == orig.bones[i].name);
		CHECK(rebuilt.bones[i].kinematic == orig.bones[i].kinematic);
		CHECK(rebuilt.bones[i].mass == doctest::Approx(orig.bones[i].mass));
	}

	// The dynamic bone keeps its sphere; recover and check the radius.
	const int shapeIdx = rebuilt.bones[1].shapeIndex;
	REQUIRE(shapeIdx >= 0);
	const CollisionShape& sh = rebuilt.shapes[shapeIdx];
	CHECK(sh.type == ShapeType::Sphere);
	CHECK(sh.radius == doctest::Approx(2.0f));

	CHECK(rebuilt.constraints[0].type == ConstraintType::Generic6Dof);
	CHECK(rebuilt.constraints[0].boneA == 0);
	CHECK(rebuilt.constraints[0].boneB == 1);

	// Mesh body round-trip: the cloth body, its skinning, and its vertices survive build->capture.
	REQUIRE(rebuilt.bodies.size() == 1);
	const MeshBody& rb = rebuilt.bodies[0];
	CHECK(rb.name == "cloth");
	CHECK(rb.shapeKind == BodyShapeKind::PerVertex);
	REQUIRE(rb.skinnedBones.size() == 1);
	CHECK(rb.skinnedBones[0].boneIndex == 1);
	CHECK(rb.skinnedBones[0].vertexToBone.scale == doctest::Approx(1.0f));
	// All three vertices are weighted, so finishBuild keeps them.
	CHECK(rb.vertices.size() == 3);
	CHECK(rb.vertices[0].weight[0] == doctest::Approx(1.0f));
}
