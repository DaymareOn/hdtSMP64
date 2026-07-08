// Definitions of the exported FSMPWig_* C-ABI. Defining FSMPWIG_EXPORTS before the header
// makes the declarations dllexport; the bodies just forward to the StrandManager singleton,
// whose accessors take the publish lock so these are safe to call from the render thread.
#define FSMPWIG_EXPORTS
#include "FSMP_WigAPI.h"

#include "StrandSystem.h"

extern "C"
{
	std::uint32_t FSMPWig_GetVersion(void)
	{
		return 1;
	}

	void FSMPWig_SetEnabled(std::uint32_t enabled)
	{
		hdt::StrandManager::instance().setEnabled(enabled != 0);
	}

	std::uint32_t FSMPWig_GetInstanceCount(void)
	{
		return hdt::StrandManager::instance().instanceCount();
	}

	std::uint32_t FSMPWig_GetInstance(std::uint32_t index, FSMPWigInstanceDesc* outDesc)
	{
		return outDesc ? hdt::StrandManager::instance().getDesc(index, outDesc) : 0;
	}

	std::uint32_t FSMPWig_CopyPositions(std::uint32_t index, float* dst, std::uint32_t dstCapacityFloats)
	{
		return (dst && dstCapacityFloats) ? hdt::StrandManager::instance().copyPositions(index, dst, dstCapacityFloats) : 0;
	}
}
