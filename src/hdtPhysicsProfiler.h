#pragma once

#include <cstdint>

namespace hdt::physicsprofiler
{
	void setCapture(bool a_enabled, std::uint64_t a_sampleFrames, std::uint64_t a_printFrames);
	void advanceFrame();
}
