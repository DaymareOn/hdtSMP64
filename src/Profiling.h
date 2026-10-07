#pragma once

#ifdef HDT_TRACY
#	include <tracy/Tracy.hpp>

#	include <cstddef>

namespace hdt::profiling
{
	template <std::size_t N>
	inline void appInfo(const char (&text)[N])
	{
		::tracy::Profiler::MessageAppInfo(text, N - 1);
	}
}

#	define HDT_ZONE(name) ZoneScopedN(name)
#	define HDT_FRAME(name) FrameMarkNamed(name)
#	define HDT_PLOT(name, value) TracyPlot(name, value)
#	define HDT_APP_INFO(text) ::hdt::profiling::appInfo(text)
#else
#	define HDT_ZONE(name) ((void)0)
#	define HDT_FRAME(name) ((void)0)
#	define HDT_PLOT(name, value) ((void)0)
#	define HDT_APP_INFO(text) ((void)0)
#endif
