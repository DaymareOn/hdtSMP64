#include "DynamicHDT.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkyrimSystem.h"

uint32_t hdt::util::splitArmorAddonFormID(std::string nodeName)
{
	try {
		return std::stoul(nodeName.substr(1, 8), nullptr, 16);
	} catch (...) {
		return 0;
	}
}

std::string hdt::util::UInt32toString(uint32_t formID)
{
	char buffer[16];
	sprintf_s(buffer, "%08X", formID);
	return std::string(buffer);
}

std::string _deprefix(std::string_view str_with_prefix)
{
	std::string str_no_prefix{ str_with_prefix };
	std::string_view autoRenameSubstr = "hdtSSEPhysics_AutoRename_"sv;

	// follows case-insensitivity semantics of BSFixedString
	if (str_with_prefix.size() >= autoRenameSubstr.size() &&
		_memicmp(str_with_prefix.data(), autoRenameSubstr.data(), autoRenameSubstr.size()) == 0) {
		str_no_prefix = str_with_prefix.substr(str_with_prefix.find(' ') + 1);
	}

	return str_no_prefix;
}

bool _match_name(const RE::BSFixedString& a, const RE::BSFixedString& b)
{
	if (a.empty() || b.empty()) {
		return false;
	}

	// follows case-insensitivity semantics of BSFixedString
	const auto aDeprefixed = _deprefix(a);
	const auto bDeprefixed = _deprefix(b);
	return _stricmp(aDeprefixed.c_str(), bDeprefixed.c_str()) == 0;
}

void hdt::util::transferCurrentPosesBetweenSystems(hdt::SkyrimSystem* src, hdt::SkyrimSystem* dst)
{
	for (auto& b1 : src->getBones()) {
		if (!b1) {
			continue;
		}
		for (auto& b2 : dst->getBones()) {
			if (!b2) {
				continue;
			}

			if (_match_name(b1->m_name, b2->m_name)) {
				btTransform nodeWorldTrans = b1->m_rig.getWorldTransform() * b1->m_rigToLocal;
				btTransform newRigTrans = nodeWorldTrans * b2->m_localToRig;

				b2->m_rig.setWorldTransform(newRigTrans);
				b2->m_rig.setInterpolationWorldTransform(newRigTrans);

				b2->m_currentTransform = b1->m_currentTransform;
				b2->m_origToSkeletonTransform = b1->m_origToSkeletonTransform;
				b2->m_origTransform = b1->m_origTransform;
				btVector3 oldCoM = b1->m_rig.getWorldTransform().getOrigin();
				btVector3 newCoM = newRigTrans.getOrigin();
				btVector3 comShift = newCoM - oldCoM;

				btVector3 angVel = b1->m_rig.getAngularVelocity();

				btVector3 linVel = b1->m_rig.getLinearVelocity() + angVel.cross(comShift);

				constexpr float transitionDamping = 0.8f;
				b2->m_rig.setLinearVelocity(linVel * transitionDamping);
				b2->m_rig.setAngularVelocity(angVel * transitionDamping);
				b2->m_rig.setInterpolationLinearVelocity(linVel * transitionDamping);
				b2->m_rig.setInterpolationAngularVelocity(angVel * transitionDamping);

				break;
			}
		}
	}
}
