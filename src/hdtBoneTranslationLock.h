#pragma once

// Pure Bullet-only core of the DynamicHDT.LockTranslation Papyrus function. Deliberately free of any
// game / CommonLibSSE types so the logic can be unit-tested headlessly against a real btRigidBody
// (see tests/validator/test_translation_lock.cpp). dhdtPapyrusFunctions.cpp calls applyTranslationLock
// on each matching bone's m_rig once it has resolved the actor's skeleton.

#include <btBulletDynamicsCommon.h>

namespace hdt
{
	/// Translate a per-axis lock request into Bullet's linear factor. A locked axis maps to 0 and an
	/// unlocked axis to 1: setLinearFactor scales the body's inverse mass per-axis, so a 0 removes all
	/// linear response on that axis (forces and constraints can no longer accelerate it) while 1 is
	/// Bullet's normal default. The angular factor is never touched, so rotation / jiggle is unaffected —
	/// that is what makes a translation-lock different from making the bone kinematic.
	inline btVector3 translationLockLinearFactor(bool lockX, bool lockY, bool lockZ)
	{
		return btVector3(lockX ? btScalar(0) : btScalar(1),
			lockY ? btScalar(0) : btScalar(1),
			lockZ ? btScalar(0) : btScalar(1));
	}

	/// Apply a per-axis translation lock to one rigid body in two steps. (1) Set the linear factor from
	/// the lock flags so the body can no longer be accelerated along the locked axes. (2) Zero the body's
	/// *current* linear velocity on each locked axis — a linear factor of 0 only blocks *new* velocity,
	/// but Bullet still integrates whatever velocity the body already carried, so without this a
	/// freshly-locked bone would coast one more step before stopping. The scalar inverse mass is left
	/// untouched (only the per-axis vector inverse mass is scaled), so calling this again with all-false
	/// restores normal translation exactly: lock/unlock is fully reversible with no mass loss.
	inline void applyTranslationLock(btRigidBody& rig, bool lockX, bool lockY, bool lockZ)
	{
		rig.setLinearFactor(translationLockLinearFactor(lockX, lockY, lockZ));

		btVector3 linearVelocity = rig.getLinearVelocity();
		if (lockX)
			linearVelocity.setX(btScalar(0));
		if (lockY)
			linearVelocity.setY(btScalar(0));
		if (lockZ)
			linearVelocity.setZ(btScalar(0));
		rig.setLinearVelocity(linearVelocity);
		rig.setInterpolationLinearVelocity(linearVelocity);
	}
}
