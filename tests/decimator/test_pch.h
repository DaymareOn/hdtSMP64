#pragma once

// The main plugin gets these headers transitively through PCH.h → <RE/Skyrim.h>.
// The test target has no PCH of its own, so we supply the minimum subset that
// the shared source files (hdtBulletHelper.h, hdtVertex.h) implicitly depend on.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>  // ZeroMemory — used by hdt::Vertex default constructor
#include <atomic>     // std::atomic        — hdt::RefObject, hdt::SpinLock
#include <cstdint>    // std::uint32_t      — hdt::RefObject
#include <mutex>      // std::lock_guard    — HDT_LOCK_GUARD macro
#include <vector>     // std::vector        — hdt::vectorA16 alias
