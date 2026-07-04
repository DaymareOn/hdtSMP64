// applyTranslationLock tests -- the pure Bullet core of the DynamicHDT.LockTranslation Papyrus function
// (#414). The helper only touches a btRigidBody's linear factor and linear velocity, with no game /
// CommonLibSSE state, so it runs headlessly against a real rigid body in this doctest harness (which
// already links Bullet). These cover the axis->factor mapping, the per-axis velocity zeroing, and the
// reversibility invariant that the runtime relies on: locking a bone must not lose its mass, so it can be
// unlocked back to normal translation.

#include "hdtBoneTranslationLock.h"

#include <doctest/doctest.h>

namespace
{
	// A minimal dynamic rigid body. The lock logic only reads/writes the linear factor and velocity, so a
	// null collision shape and motion state with explicit unit inertia are enough -- nothing under test
	// dereferences the shape. Returned by value; btRigidBody is copyable and the ci only needs to outlive
	// the constructor call.
	btRigidBody makeBody(btScalar mass)
	{
		btRigidBody::btRigidBodyConstructionInfo ci(mass, nullptr, nullptr, btVector3(1, 1, 1));
		return btRigidBody(ci);
	}
}

TEST_CASE("translationLockLinearFactor maps locked axes to 0 and free axes to 1")
{
	CHECK(hdt::translationLockLinearFactor(false, false, false) == btVector3(1, 1, 1));
	CHECK(hdt::translationLockLinearFactor(true, true, true) == btVector3(0, 0, 0));
	CHECK(hdt::translationLockLinearFactor(true, false, false) == btVector3(0, 1, 1));
	CHECK(hdt::translationLockLinearFactor(false, true, false) == btVector3(1, 0, 1));
	CHECK(hdt::translationLockLinearFactor(false, false, true) == btVector3(1, 1, 0));
}

TEST_CASE("applyTranslationLock sets the linear factor from the lock flags")
{
	btRigidBody rig = makeBody(1.0f);

	hdt::applyTranslationLock(rig, true, false, true);
	CHECK(rig.getLinearFactor() == btVector3(0, 1, 0));
}

TEST_CASE("applyTranslationLock zeroes velocity only on locked axes")
{
	btRigidBody rig = makeBody(1.0f);
	rig.setLinearVelocity(btVector3(5, 6, 7));

	hdt::applyTranslationLock(rig, false, true, false);  // lock Y only

	CHECK(rig.getLinearVelocity() == btVector3(5, 0, 7));
	// The interpolation velocity is kept in sync so the render step doesn't reintroduce the drift.
	CHECK(rig.getInterpolationLinearVelocity() == btVector3(5, 0, 7));
}

TEST_CASE("a fully locked bone unlocks back to normal translation without losing mass")
{
	btRigidBody rig = makeBody(2.0f);
	const btScalar invMass = rig.getInvMass();  // 1 / 2
	REQUIRE(invMass == doctest::Approx(0.5f));

	hdt::applyTranslationLock(rig, true, true, true);
	CHECK(rig.getLinearFactor() == btVector3(0, 0, 0));
	CHECK(rig.getInvMass() == doctest::Approx(invMass));  // scalar inverse mass survives the lock

	hdt::applyTranslationLock(rig, false, false, false);  // unlock every axis
	CHECK(rig.getLinearFactor() == btVector3(1, 1, 1));
	CHECK(rig.getInvMass() == doctest::Approx(invMass));  // and translation is fully restored
}

TEST_CASE("locking a zero-mass (kinematic-style) body is harmless")
{
	btRigidBody rig = makeBody(0.0f);
	REQUIRE(rig.getInvMass() == doctest::Approx(0.0f));

	hdt::applyTranslationLock(rig, true, true, true);

	CHECK(rig.getLinearFactor() == btVector3(0, 0, 0));
	CHECK(rig.getInvMass() == doctest::Approx(0.0f));  // still massless, nothing corrupted
}
