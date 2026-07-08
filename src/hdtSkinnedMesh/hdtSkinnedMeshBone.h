#pragma once

#include "hdtAABB.h"
#include "hdtBulletHelper.h"
#include <memory>

namespace hdt
{
	class SkinnedMeshBody;
	_CRT_ALIGN(16)
	struct SkinnedMeshBone :
		public RE::BSIntrusiveRefCounted
	{
		BT_DECLARE_ALIGNED_ALLOCATOR();

		SkinnedMeshBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci);
		virtual ~SkinnedMeshBone();

		RE::BSFixedString m_name;
		float m_marginMultipler;
		float m_boudingSphereMultipler = 1.0f;
		float m_gravityFactor = 1.0f;
		float m_windFactor = 1.0f;  // Mapped to <wind-factor> in the XML. Acts as a multiplier for the global wind force applied to this bone (0.0 = no wind, 2.0 = double wind)

		btRigidBody m_rig;
		btTransform m_localToRig;
		btTransform m_rigToLocal;
		btQsTransform m_currentTransform;

		// D1 temporal interpolation: rig world transform at the end of the previous physics step.
		// Render frames between steps blend this with the current m_rig transform (see writeTransform).
		btTransform m_prevRigTransform;

		std::vector<RE::BSFixedString> m_canCollideWithBone;
		std::vector<RE::BSFixedString> m_noCollideWithBone;

		virtual void readTransform(float timeStep) = 0;
		// alpha in [0,1] blends from m_prevRigTransform (0) to the current solved transform (1).
		virtual void writeTransform(float alpha) = 0;

		// Capture the current solved transform as the interpolation start point, before the next step.
		void snapshotInterpolation() { m_prevRigTransform = m_rig.getWorldTransform(); }

		// Blend between the previous and current solved rig transforms for smooth sub-step rendering.
		btTransform interpolatedWorldTransform(float alpha) const
		{
			const btTransform& cur = m_rig.getWorldTransform();
			if (alpha >= 1.0f)
				return cur;
			btTransform out;
			out.setOrigin(m_prevRigTransform.getOrigin().lerp(cur.getOrigin(), alpha));
			out.setRotation(m_prevRigTransform.getRotation().slerp(cur.getRotation(), alpha));
			return out;
		}

		void internalUpdate();

		bool canCollideWith(SkinnedMeshBone* rhs);
	};
}
