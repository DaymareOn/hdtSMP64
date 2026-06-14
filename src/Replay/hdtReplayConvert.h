#pragma once

// hdtReplayConvert.h - conversions between Bullet math types and the on-disk replay PODs.
//
// Shared utility for both sides of the seam: the capture side (Bullet -> POD, in the game DLL) and
// the replay side (POD -> Bullet, in smp_replay). Keeping the converters here removes the duplication
// that previously lived in hdtReplayCapture.h and hdtReplaySystem.h, and guarantees both directions
// agree. Depends on Bullet (unlike hdtReplayFormat.h, which is deliberately Bullet-free), so it is
// included only by translation units that already pull in Bullet.

#include "hdtReplayFormat.h"
#include "hdtSkinnedMesh/hdtBulletHelper.h"

namespace hdt::replay
{
	// ---- Bullet -> on-disk POD (capture direction) ----

	/// Converts a Bullet 3-vector to the on-disk Vec3 (drops the SIMD w lane).
	inline Vec3 toVec3(const btVector3& v) { return Vec3{ v.x(), v.y(), v.z() }; }

	/// Converts a Bullet 4-vector to the on-disk Vec4.
	inline Vec4 toVec4(const btVector4& v) { return Vec4{ v.x(), v.y(), v.z(), v.w() }; }

	/// Converts a Bullet quaternion to the on-disk Quat.
	inline Quat toQuat(const btQuaternion& q) { return Quat{ q.x(), q.y(), q.z(), q.w() }; }

	/// Converts a Bullet btQsTransform (rotation + origin + uniform scale) to the on-disk QsTransform.
	inline QsTransform toQs(const btQsTransform& t)
	{
		QsTransform out;
		out.basis = toQuat(t.getBasis());
		out.origin = toVec3(t.getOrigin());
		out.scale = t.getScale();
		return out;
	}

	/// Converts a Bullet btTransform (rotation + origin) to the on-disk Transform.
	inline Transform toTransform(const btTransform& t)
	{
		Transform out;
		out.basis = toQuat(t.getRotation());
		out.origin = toVec3(t.getOrigin());
		return out;
	}

	/// Recovers the originating btQsTransform from a baked btMatrix4x3T (rotation * uniform scale, plus
	/// translation), so a vertexToBone matrix can be re-fed through SkinnedMeshBody::addBone unchanged.
	/// The linear part's columns are the rotation columns times the uniform scale, hence
	/// scale = |column0|, rotation = columns / scale, origin = column3.
	inline QsTransform qsFromMatrix4x3T(const btMatrix4x3T& m)
	{
		QsTransform out;
		const btVector3 c0 = m.m_col[0], c1 = m.m_col[1], c2 = m.m_col[2];
		const float s = c0.length();
		if (s > FLT_EPSILON) {
			btMatrix3x3 r(
				c0.x() / s, c1.x() / s, c2.x() / s,
				c0.y() / s, c1.y() / s, c2.y() / s,
				c0.z() / s, c1.z() / s, c2.z() / s);
			btQuaternion q;
			r.getRotation(q);
			out.basis = toQuat(q);
			out.scale = s;
		}
		out.origin = toVec3(m.m_col[3]);
		return out;
	}

	// ---- on-disk POD -> Bullet (replay direction) ----

	/// Converts an on-disk Vec3 to a Bullet 3-vector.
	inline btVector3 toBt(const Vec3& v) { return btVector3(v.x, v.y, v.z); }

	/// Converts an on-disk Quat to a Bullet quaternion.
	inline btQuaternion toBt(const Quat& q) { return btQuaternion(q.x, q.y, q.z, q.w); }

	/// Converts an on-disk Transform to a Bullet btTransform.
	inline btTransform toBtTransform(const Transform& t) { return btTransform(toBt(t.basis), toBt(t.origin)); }

	/// Converts an on-disk QsTransform to a Bullet btQsTransform. Scale is clamped to a positive value
	/// because btQsTransform::setScale asserts scale > 0.
	inline btQsTransform toBtQs(const QsTransform& t)
	{
		btQsTransform r;
		r.setBasis(toBt(t.basis));
		r.setOrigin(toBt(t.origin));
		r.setScale(t.scale > 0.0f ? t.scale : 1.0f);
		return r;
	}
}
