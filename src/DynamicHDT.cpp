#include "DynamicHDT.h"
#include "hdtSkyrimSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

uint32_t hdt::util::splitArmorAddonFormID(std::string nodeName)
{
	try 
	{
		return std::stoul(nodeName.substr(1, 8), nullptr, 16);
	}
	catch (...) 
	{
		return 0;
	}
}

std::string hdt::util::UInt32toString(uint32_t formID)
{
	char buffer[16];
	sprintf_s(buffer, "%08X", formID);
	return std::string(buffer);
}

std::string _deprefix(std::string str_with_prefix) 
{
	std::string str_no_prefix = str_with_prefix;
	if (str_with_prefix.find("hdtSSEPhysics_AutoRename_") == 0) 
	{
		str_no_prefix = str_with_prefix.substr(str_with_prefix.find(' ') + 1);
	}

	return str_no_prefix;
}

bool _match_name(hdt::IDStr& a, hdt::IDStr& b) 
{
	if (!a || !b)
	{
		return false;
	}
	
	return _deprefix(a->cstr()) == _deprefix(b->cstr());
}

void hdt::util::transferCurrentPosesBetweenSystems(hdt::SkyrimSystem* src, hdt::SkyrimSystem* dst)
{
	for (auto& b1 : src->getBones()) 
	{
		if (!b1)
		{
			continue;
		}
		for (auto& b2 : dst->getBones()) 
		{
			if (!b2)
			{
				continue;
			}

			if (_match_name(b1->m_name, b2->m_name)) 
			{
				b2->m_rig.setWorldTransform(b1->m_rig.getWorldTransform());
				b2->m_rig.setAngularVelocity(b1->m_rig.getAngularVelocity());
				b2->m_rig.setLinearVelocity(b1->m_rig.getLinearVelocity());
				break;
			}
		}
	}
}
