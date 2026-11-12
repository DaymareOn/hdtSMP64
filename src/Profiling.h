#pragma once

// Optional Tracy instrumentation, enabled by the ENABLE_TRACY CMake option (which links Tracy::TracyClient and defines
// HDT_TRACY on this target --- see the vs2022-windows-avx2-tracy configure preset). Every macro below compiles away to
// nothing without it, so this header is safe to include unconditionally.
#ifdef HDT_TRACY
#	include <tracy/Tracy.hpp>  // TracyClient's interface include dir is <prefix>/include/tracy.

#	define HDT_ZONE(name) ZoneScopedN(name)
#	define HDT_FRAME(name) FrameMarkNamed(name)
#	define HDT_PLOT(name, value) TracyPlot(name, value)
#	define HDT_THREAD_NAME(name) ::tracy::SetThreadName(name)  // declared in TracySystem.hpp, which Tracy.hpp pulls in
// `text` must be a string literal; the length handed to Tracy excludes its terminator.
#	define HDT_APP_INFO(text) ::tracy::Profiler::MessageAppInfo(text, sizeof(text) - 1)
#else
#	define HDT_ZONE(name) ((void)0)
#	define HDT_FRAME(name) ((void)0)
#	define HDT_PLOT(name, value) ((void)0)
#	define HDT_THREAD_NAME(name) ((void)0)
#	define HDT_APP_INFO(text) ((void)0)
#endif
