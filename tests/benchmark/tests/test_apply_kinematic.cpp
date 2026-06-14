// Unit tests for SkinnedMeshBone::applyKinematicTarget (D6). These are the "shim build" tests: they
// compile the shared physics-core bone against the RE shim (ShimPCH.h) and Bullet, with no Skyrim
// layer, proving both that the extracted apply-math is correct and that the core links standalone.
// Coverage mirrors PLAN §15: reset -> snap + zero velocity; kinematic -> expected lin/ang velocity;
// scale delta -> inertia/mass rescale.

#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"

#include <doctest/doctest.h>

namespace
{
	// Concrete bone: the base readTransform/writeTransform are pure virtual. (ReplayBone is the real
	// production subclass; this keeps the unit test self-contained.)
	struct TestBone : hdt::SkinnedMeshBone
	{
		using SkinnedMeshBone::SkinnedMeshBone;
		void readTransform(float) override {}
		void writeTransform() override {}
	};

	btRigidBody::btRigidBodyConstructionInfo makeCI(float mass, btCollisionShape* shape)
	{
		btVector3 inertia(0, 0, 0);
		if (mass > 0.0f)
			shape->calculateLocalInertia(mass, inertia);
		return btRigidBody::btRigidBodyConstructionInfo(mass, nullptr, shape, inertia);
	}
}

TEST_CASE("applyKinematicTarget reset snaps onto the target and zeroes velocity")
{
	btSphereShape shape(1.0f);
	auto ci = makeCI(0.0f, &shape);  // kinematic/static driver bone
	TestBone bone(RE::BSFixedString("driver"), ci);

	hdt::btQsTransform target;
	target.setOrigin(1.0f, 2.0f, 3.0f);

	bone.applyKinematicTarget(target, 1.0f / 60.0f, /*reset*/ true);

	auto origin = bone.m_rig.getWorldTransform().getOrigin();
	CHECK(origin.x() == doctest::Approx(1.0f));
	CHECK(origin.y() == doctest::Approx(2.0f));
	CHECK(origin.z() == doctest::Approx(3.0f));
	CHECK(bone.m_rig.getLinearVelocity().length() == doctest::Approx(0.0f));
	CHECK(bone.m_rig.getAngularVelocity().length() == doctest::Approx(0.0f));
}

TEST_CASE("applyKinematicTarget derives kinematic velocity from the move")
{
	btSphereShape shape(1.0f);
	auto ci = makeCI(0.0f, &shape);  // kinematic driver: velocity branch runs
	TestBone bone(RE::BSFixedString("driver"), ci);

	// Body starts at identity (origin 0). Move it to x=1 over a 0.5s step -> linear velocity x=2.
	hdt::btQsTransform target;
	target.setOrigin(1.0f, 0.0f, 0.0f);
	bone.applyKinematicTarget(target, 0.5f, /*reset*/ false);

	auto v = bone.m_rig.getLinearVelocity();
	CHECK(v.x() == doctest::Approx(2.0f));
	CHECK(v.y() == doctest::Approx(0.0f));
	CHECK(v.z() == doctest::Approx(0.0f));
}

TEST_CASE("applyKinematicTarget rescales mass on a scale delta")
{
	btSphereShape shape(1.0f);
	auto ci = makeCI(1.0f, &shape);  // dynamic bone, invMass == 1
	TestBone bone(RE::BSFixedString("dynamic"), ci);
	REQUIRE(bone.m_rig.getInvMass() == doctest::Approx(1.0f));

	// Scale 1 -> 2: factor = old/new = 0.5, factor^3 = 0.125, so new invMass = invMass * factor^3.
	hdt::btQsTransform target;
	target.setScale(2.0f);
	bone.applyKinematicTarget(target, 1.0f / 60.0f, /*reset*/ false);

	CHECK(bone.m_rig.getInvMass() == doctest::Approx(0.125f));
}
