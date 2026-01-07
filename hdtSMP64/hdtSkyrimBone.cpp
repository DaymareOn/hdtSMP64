#include "hdtSkyrimBone.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtForceUpdateList.h"

namespace hdt
{
	SkyrimBone::SkyrimBone(IDStr name, NiNode* node, NiNode* skeleton, btRigidBody::btRigidBodyConstructionInfo& ci)
		: SkinnedMeshBone(name, ci), m_node(node), m_skeleton(skeleton)
	{
		if (ci.m_mass)
			m_rig.setCollisionFlags(0);
		else m_rig.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

		m_depth = 0;
		for (auto i = node; i; i = i->m_parent)
			++m_depth;

		this->m_forceUpdateType = hdt::ForceUpdateList::GetSingleton()->isAmong(this->m_name);
	}

	void SkyrimBone::resetTransformToOriginal()
	{
		m_node->m_localTransform = convertBt(m_origTransform);
		updateTransformUpDown(m_node, false);
	}

	void SkyrimBone::readTransform(float timeStep)
	{
		auto oldScale = m_currentTransform.getScale();
		m_currentTransform = convertNi(m_node->m_worldTransform);
		auto newScale = m_currentTransform.getScale();

		auto current = m_rig.getWorldTransform();

		auto factor = oldScale / newScale;
		if (!m_rig.isStaticOrKinematicObject() && !btFuzzyZero(factor - 1))
		{
			auto factor2 = factor * factor;
			auto factor3 = factor2 * factor;
			auto factor5 = factor3 * factor2;
			auto inertia = m_rig.getInvInertiaDiagLocal();
			m_rig.setMassProps(1.0f / (m_rig.getInvMass() * factor3), btVector3(1, 1, 1));
			m_rig.setInvInertiaDiagLocal(inertia * factor5);
			m_rig.updateInertiaTensor();
		}

		factor = newScale / oldScale;
		if (!btFuzzyZero(factor - 1))
		{
			m_localToRig.getOrigin() *= factor;
			m_rigToLocal.getOrigin() *= factor;
		}
		m_rig.getCollisionShape()->setLocalScaling(setAll(newScale));

		auto dest = m_currentTransform.asTransform() * m_localToRig;
		
		if (timeStep <= RESET_PHYSICS)
		{
			m_origToSkeletonTransform = convertNi(m_skeleton->m_worldTransform).inverse() * convertNi(m_node->m_worldTransform);
			m_origTransform = convertNi(m_node->m_localTransform);
			m_rig.setWorldTransform(dest);
			m_rig.setInterpolationWorldTransform(dest);
			m_rig.setLinearVelocity(btVector3(0, 0, 0));
			m_rig.setAngularVelocity(btVector3(0, 0, 0));
			m_rig.setInterpolationLinearVelocity(btVector3(0, 0, 0));
			m_rig.setInterpolationAngularVelocity(btVector3(0, 0, 0));
			m_rig.updateInertiaTensor();

			//auto det = dest.getBasis().determinant();
			//if (det < FLT_EPSILON || isnan(det) || isinf(det))
			//	_WARNING("Invalid rotation matrix!!");

			//det = m_rig.getInvInertiaTensorWorld().determinant();
			//if (isnan(det) || isinf(det))
			//	_WARNING("Invalid inertia tensor matrix!!");
		}
		else if (m_rig.isStaticOrKinematicObject())
		{
			btVector3 linVel, angVel;
			btTransformUtil::calculateVelocity(current, dest, timeStep, linVel, angVel);
			m_rig.setLinearVelocity(linVel);
			m_rig.setAngularVelocity(angVel);
			m_rig.setInterpolationLinearVelocity(linVel);
			m_rig.setInterpolationAngularVelocity(angVel);
		}
		//else
		//{
		//	auto det = m_rig.getWorldTransform().getBasis().determinant();
		//	if (isnan(det))
		//	{
		//		_WARNING("Invalid world transform");
		//		m_rig.setWorldTransform(dest);
		//		m_rig.setInterpolationWorldTransform(dest);
		//		m_rig.setLinearVelocity(btVector3(0, 0, 0));
		//		m_rig.setAngularVelocity(btVector3(0, 0, 0));
		//		m_rig.setInterpolationLinearVelocity(btVector3(0, 0, 0));
		//		m_rig.setInterpolationAngularVelocity(btVector3(0, 0, 0));
		//		m_rig.updateInertiaTensor();
		//	}
		//}
	}

	void SkyrimBone::writeTransform()
	{
		//if (m_rig.isStaticOrKinematicObject()) return;
		auto transform = m_rig.getWorldTransform() * m_rigToLocal;

		m_currentTransform.setBasis(transform.getBasis());
		m_currentTransform.setOrigin(transform.getOrigin());

		m_node->m_worldTransform.rot = convertBt(transform.getBasis());
		m_node->m_worldTransform.pos = convertBt(transform.getOrigin());
		m_node->m_worldTransform = m_node->m_worldTransform;

		if (m_forceUpdateType == 1) {
			updateTransformUpDown(m_node, false);
		}
		else if (m_forceUpdateType == 2) {
			for (int j = 0; j < m_node->m_children.m_size; ++j) {
				auto m_weapon_node = m_node->m_children.m_data[j];
				//Why when re-equipping things some nodes turn into nullptr?
				//Equipment skeleton renamed weapon bones which were romoved when the equipment was disattahced.
				if (!m_weapon_node)continue;
				m_weapon_node->m_worldTransform = m_node->m_worldTransform;
				updateTransformUpDown(m_weapon_node,false);
			}
		}

		//_MESSAGE("wrote transforms bone %s [%f, %f, %f]", m_node->m_name, m_node->m_worldTransform.pos.x, m_node->m_worldTransform.pos.y, m_node->m_worldTransform.pos.z);

		//auto parentTransform = m_node->m_parent ? m_node->m_parent->unkTransform : NiTransform();
		//NiTransform invParentTransform;
		//parentTransform.Invert(invParentTransform);
		//m_node->m_localTransform = invParentTransform * m_node->unkTransform;

		//updateTransformUpDown(m_node->GetAsNiNode());
	}


	//void SkyrimBone::debugPrint(std::string name) {
	//	if (this->m_name == name && SkyrimPhysicsWorld::get()->isSuspended() == false) {
	//		auto tf0 = m_rig.getWorldTransform().getOrigin();
	//		auto tf = (convertNi(m_skeleton->m_worldTransform).inverse() * convertNi(m_node->m_worldTransform)).getOrigin();
	//		auto tf1 = (convertNi(m_node->m_parent->m_parent->m_worldTransform).inverse() * convertNi(m_node->m_worldTransform)).getOrigin();

	//		Console_Print("wrote transforms bone %s [%.3f, %.3f, %.3f] | [%.3f, %.3f, %.3f] | [%.3f, %.3f, %.3f], %d, Kinematic: %s", m_node->m_name, tf0.x(), tf0.y(), tf0.z(), tf.x(), tf.y(), tf.z(), tf1.x(), tf1.y(), tf1.z(), clock(), m_rig.isStaticOrKinematicObject() ? "true" : "false");
	//	}
	//}
}
