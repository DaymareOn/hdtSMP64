#include "hdtSkinnedMeshBone.h"

namespace hdt
{
	SkinnedMeshBone::SkinnedMeshBone(const RE::BSFixedString& name, btRigidBody::btRigidBodyConstructionInfo& ci) :
		m_name(name), m_rig(ci)
	{
		m_rigToLocal.setIdentity();
		m_localToRig.setIdentity();
		m_currentTransform.setScale(1);

		m_marginMultipler = 1.0f;

		m_rig.setUserPointer(this);
	}

	SkinnedMeshBone::~SkinnedMeshBone()
	{
	}

	void SkinnedMeshBone::applyKinematicTarget(const btQsTransform& target, float timeStep, bool reset)
	{
		auto oldScale = m_currentTransform.getScale();

		m_currentTransform = target;

		auto newScale = m_currentTransform.getScale();
		auto current = m_rig.getWorldTransform();
		auto isStaticOrKinematic = m_rig.isStaticOrKinematicObject();
		auto scaleChanged = !btFuzzyZero(newScale - oldScale);

		if (scaleChanged) {
			auto factor = oldScale / newScale;
			if (!isStaticOrKinematic) {
				auto factor2 = factor * factor;
				auto factor3 = factor2 * factor;
				auto factor5 = factor3 * factor2;
				auto inertia = m_rig.getInvInertiaDiagLocal();
				m_rig.setMassProps(1.0f / (m_rig.getInvMass() * factor3), btVector3(1, 1, 1));
				m_rig.setInvInertiaDiagLocal(inertia * factor5);
				m_rig.updateInertiaTensor();
			}
			auto invFactor = 1.0f / factor;
			m_localToRig.getOrigin() *= invFactor;
			m_rigToLocal.getOrigin() *= invFactor;
			m_rig.getCollisionShape()->setLocalScaling(setAll(newScale));
		}

		auto dest = m_currentTransform.asTransform() * m_localToRig;
		if (reset) {
			static const btVector3 zero(0, 0, 0);
			m_rig.setWorldTransform(dest);
			m_rig.setInterpolationWorldTransform(dest);
			m_rig.setLinearVelocity(zero);
			m_rig.setAngularVelocity(zero);
			m_rig.setInterpolationLinearVelocity(zero);
			m_rig.setInterpolationAngularVelocity(zero);
			m_rig.updateInertiaTensor();
		} else if (isStaticOrKinematic) {
			btVector3 linVel, angVel;
			btTransformUtil::calculateVelocity(current, dest, timeStep, linVel, angVel);
			m_rig.setLinearVelocity(linVel);
			m_rig.setAngularVelocity(angVel);
			m_rig.setInterpolationLinearVelocity(linVel);
			m_rig.setInterpolationAngularVelocity(angVel);
		}
	}

	void SkinnedMeshBone::internalUpdate()
	{
		auto t = m_rigToLocal * m_rig.getInterpolationWorldTransform();
		m_currentTransform.setBasis(t.getBasis());
		m_currentTransform.setOrigin(t.getOrigin());
	}

	bool SkinnedMeshBone::canCollideWith(SkinnedMeshBone* rhs)
	{
		if (m_canCollideWithBone.size()) {
			return std::find(m_canCollideWithBone.begin(), m_canCollideWithBone.end(), rhs->m_name) !=
			       m_canCollideWithBone.end();
		}
		return std::find(m_noCollideWithBone.begin(), m_noCollideWithBone.end(), rhs->m_name) == m_noCollideWithBone.end();
	}
}
