#include "hdtSkyrimBone.h"
#include "hdtForceUpdateList.h"
#include "hdtSkyrimPhysicsWorld.h"

namespace hdt
{
	SkyrimBone::SkyrimBone(const RE::BSFixedString& name, RE::NiNode* node, RE::NiNode* skeleton, btRigidBody::btRigidBodyConstructionInfo& ci) :
		SkinnedMeshBone(name, ci), m_node(node), m_skeleton(skeleton)
	{
		if (ci.m_mass)
			m_rig.setCollisionFlags(0);

		else
			m_rig.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

		m_depth = 0;
		for (auto i = node; i; i = i->parent)
			++m_depth;

		this->m_forceUpdateType = GetForceUpdateTypeFromName(m_name);
	}

	void SkyrimBone::readTransform(float timeStep)
	{
		// The Skyrim layer only computes the driver target from the NiNode and the reset flag from
		// the timestep; the apply-math lives in the shared core (D6) so offline replay runs it too.
		applyKinematicTarget(convertNi(m_node->world), timeStep, timeStep <= RESET_PHYSICS);
	}

	void SkyrimBone::writeTransform()
	{
		//if (m_rig.isStaticOrKinematicObject()) return;
		auto transform = m_rig.getWorldTransform() * m_rigToLocal;

		m_currentTransform.setBasis(transform.getBasis());
		m_currentTransform.setOrigin(transform.getOrigin());

		m_node->world.rotate = convertBt(transform.getBasis());
		m_node->world.translate = convertBt(transform.getOrigin());
		// Todo: Look into why the hell we're doing this lol?
		m_node->world = m_node->world;

		if (m_forceUpdateType == 1) {
			updateTransformUpDown(m_node.get(), false);
		} else if (m_forceUpdateType == 2) {
			const auto& children = m_node->GetChildren();
			for (uint16_t j = 0; j < children.size(); ++j) {
				const auto& m_weapon_node = children[j];
				//Why when re-equipping things some nodes turn into nullptr?
				//Equipment skeleton renamed weapon bones which were removed when the equipment was disattached.
				if (!m_weapon_node) {
					continue;
				}

				m_weapon_node->world = m_node->world;
				updateTransformUpDown(m_weapon_node.get(), false);
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
