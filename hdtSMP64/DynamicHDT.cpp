#include "DynamicHDT.h"
#include "hdtSkyrimSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

UInt32 hdt::util::splitArmorAddonFormID(std::string nodeName)
{
	try {
		return std::stoul(nodeName.substr(1, 8), nullptr, 16);
	}
	catch (...) {
		return 0;
	}
}

std::string hdt::util::UInt32toString(UInt32 formID)
{
	char buffer[16];
	sprintf_s(buffer, "%08X", formID);
	return std::string(buffer);
}
