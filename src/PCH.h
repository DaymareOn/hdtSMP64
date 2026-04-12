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

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <clocale>
#include <d3d11.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <ppl.h>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
	// TODO: should this be contributed to CommonLibSSE-NG?
	template <class CharT>
	struct hash<RE::detail::BSFixedString<CharT>>
	{
	public:
		[[nodiscard]] inline std::size_t operator()(const RE::detail::BSFixedString<CharT>& a_key) const noexcept
		{
			return std::hash<const CharT*>{}(a_key.data());
		}
	};
}

#define DLLEXPORT __declspec(dllexport)

#include "Plugin.h"
