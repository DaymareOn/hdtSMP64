#pragma once

// The main plugin gets these headers transitively through PCH.h. The test target has
// no PCH of its own, so we supply the minimum subset the shared source files
// (XmlReader.h → hdtBulletHelper.h, hdtTemplateDefaults.cpp) implicitly depend on.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <algorithm>  // std::clamp      — hdtBulletHelper.h
#include <atomic>     // std::atomic     — hdt::SpinLock (hdtBulletHelper.h)
#include <bit>        // std::bit_floor  — hdtBulletHelper.h
#include <cstdint>    // std::uint*_t    — hdtBulletHelper.h
#include <mutex>      // std::lock_guard — hdtBulletHelper.h
#include <vector>     // std::vector     — hdt::vectorA16 alias (hdtBulletHelper.h)
#include <windows.h>  // BYTE            — XMLReader's buffer constructor

// hdtNifSchema.cpp emits verbose spdlog trace lines (`logger::info("...{}", ...)`, guarded by a
// debug flag but compiled unconditionally). In the plugin, `logger` comes from CommonLibSSE via
// PCH.h; the headless test target has none, so stand in a no-op variadic `logger` that swallows any
// arguments. Test-target only — the plugin build still uses the real logger through PCH.h.
namespace logger
{
	template <class... Args>
	inline void trace(Args&&...)
	{}
	template <class... Args>
	inline void debug(Args&&...)
	{}
	template <class... Args>
	inline void info(Args&&...)
	{}
	template <class... Args>
	inline void warn(Args&&...)
	{}
	template <class... Args>
	inline void error(Args&&...)
	{}
	template <class... Args>
	inline void critical(Args&&...)
	{}
}
