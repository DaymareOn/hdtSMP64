#pragma once

#pragma warning(push)
#include <RE/Skyrim.h>
#include <REL/Relocation.h>
#include <SKSE/SKSE.h>

#ifdef NDEBUG
#	include <spdlog/sinks/basic_file_sink.h>
#else
#	include <spdlog/sinks/msvc_sink.h>
#endif
#pragma warning(pop)

#include "IString.h"
#include "FrameworkUtils.h"

//
#include <vector>
#include <atomic>
#include <mutex>
#include <functional>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <random>
#include <fstream>
#include <d3d11.h>
#include <clocale>
#include <ppl.h>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <sstream>
#include <iostream>
#include <optional>
#include <cinttypes>
#include <amp.h>
#include <amp_graphics.h>
#include <amp_math.h>
#include <amp_short_vectors.h>

using namespace std::literals;

namespace logger = SKSE::log;

namespace util
{
	using SKSE::stl::report_and_fail;
}

namespace RE
{
	template <class T1, class T2>
	[[nodiscard]] constexpr bool operator==(const BSTSmartPointer<T1>& a_lhs, T2* a_rhs)
	{
		return a_lhs.get() == a_rhs;
	}

	template <class T1, class T2>
	[[nodiscard]] constexpr bool operator!=(const BSTSmartPointer<T1>& a_lhs, T2* a_rhs)
	{
		return !(a_lhs.get() == a_rhs);
	}

	template <class T1, class T2>
	[[nodiscard]] constexpr bool operator==(const NiPointer<T1>& a_lhs, T2* a_rhs)
	{
		return a_lhs.get() == a_rhs;
	}

	template <class T1, class T2>
	[[nodiscard]] constexpr bool operator!=(const NiPointer<T1>& a_lhs, T2* a_rhs)
	{
		return !(a_lhs.get() == a_rhs);
	}
}

// WRAPPER FUNCTIONS(makes the overall code cleaner as it's a function call instead of defineing RE::NiPointer<T>{...} / RE::BSTSmartPointer<T>{...} every time...
namespace hdt
{
	template <class T>
	[[nodiscard]] RE::NiPointer<T> make_nismart(T* a_ptr)
	{
		RE::NiPointer<T> result;
		result.reset(a_ptr);
		return result;
	}

	template <class T>
	[[nodiscard]] RE::BSTSmartPointer<T> make_smart(T* a_ptr)
	{
		RE::BSTSmartPointer<T> result;
		result.reset(a_ptr);
		return result;
	}
}

namespace std
{
	template <>
	struct hash<hdt::IDStr>
	{
		size_t operator()(const hdt::IDStr& id) const noexcept 
		{
            return std::hash<std::string>()(id->cstr());
        }
	};

	template <>
	struct hash<hdt::IString>
	{
		size_t operator()(const hdt::IString& id) const noexcept
		{
			return std::hash<std::string>()(id.cstr());
		}
	};
}

#define DLLEXPORT __declspec(dllexport)

#include "Plugin.h"
