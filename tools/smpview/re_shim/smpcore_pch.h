// smpcore_pch.h — standalone replacement for hdtSMP64 src/PCH.h
//
// Lets the hdtSkinnedMesh/* physics core compile with NO CommonLibSSE / SKSE.
// The mod's real PCH force-includes <RE/Skyrim.h>; here we provide only the
// handful of RE:: utility types the core actually uses: BSIntrusiveRefCounted,
// BSTSmartPointer<T>, make_smart, BSFixedString (+std::hash), the `logger`
// namespace, plus <windows.h>/<tbb/tbb.h>/std (all things the mod PCH supplied
// transitively — none of them CommonLibSSE).
//
// Modeled on the existing standalone refcount base hdt::RefObject in
// hdtBulletHelper.h, which already proves intrusive refcounting works here.
#pragma once

// Win32 types the core uses (ZeroMemory, UINT, ...) — the mod PCH gets these
// transitively via <RE/Skyrim.h>; standalone we include <windows.h> directly.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// std umbrella the core relied on the mod PCH to provide.
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cinttypes>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// TBB umbrella — core uses tbb::parallel_for_each / parallel_for (mod PCH:30).
#include <tbb/tbb.h>

using namespace std::literals;

// --- minimal RE:: shim ------------------------------------------------------
namespace RE
{
	// Intrusive refcount base. CommonLibSSE semantics: IncRef/DecRef return the
	// new count; the smart pointer deletes the object when DecRef() hits 0.
	class BSIntrusiveRefCounted
	{
	public:
		std::uint32_t IncRef() const noexcept { return ++_refCount; }
		std::uint32_t DecRef() const noexcept { return --_refCount; }
		std::uint32_t GetRefCount() const noexcept { return _refCount.load(); }

	private:
		mutable std::atomic<std::uint32_t> _refCount{ 0 };
	};

	// Intrusive smart pointer. Works with any T exposing IncRef()/DecRef()
	// (both BSIntrusiveRefCounted-derived and hdt::RefObject-derived qualify).
	template <class T>
	class BSTSmartPointer
	{
	public:
		BSTSmartPointer() noexcept = default;
		BSTSmartPointer(std::nullptr_t) noexcept {}

		explicit BSTSmartPointer(T* a_ptr) :
			_ptr(a_ptr) { if (_ptr) _ptr->IncRef(); }

		BSTSmartPointer(const BSTSmartPointer& a_rhs) :
			_ptr(a_rhs._ptr) { if (_ptr) _ptr->IncRef(); }

		BSTSmartPointer(BSTSmartPointer&& a_rhs) noexcept :
			_ptr(a_rhs._ptr) { a_rhs._ptr = nullptr; }

		// Covariant construction (store derived in a base-typed pointer).
		template <class U>
		BSTSmartPointer(const BSTSmartPointer<U>& a_rhs) :
			_ptr(a_rhs.get()) { if (_ptr) _ptr->IncRef(); }

		~BSTSmartPointer() { release_(); }

		BSTSmartPointer& operator=(const BSTSmartPointer& a_rhs)
		{
			reset(a_rhs._ptr);
			return *this;
		}

		BSTSmartPointer& operator=(BSTSmartPointer&& a_rhs) noexcept
		{
			if (this != &a_rhs) {
				release_();
				_ptr = a_rhs._ptr;
				a_rhs._ptr = nullptr;
			}
			return *this;
		}

		template <class U>
		BSTSmartPointer& operator=(const BSTSmartPointer<U>& a_rhs)
		{
			reset(a_rhs.get());
			return *this;
		}

		void reset() noexcept { release_(); }

		void reset(T* a_ptr)
		{
			if (a_ptr)
				a_ptr->IncRef();  // bump new first (handles self-reset)
			release_();
			_ptr = a_ptr;
		}

		[[nodiscard]] T* get() const noexcept { return _ptr; }
		T* operator->() const noexcept { return _ptr; }
		T& operator*() const noexcept { return *_ptr; }
		explicit operator bool() const noexcept { return _ptr != nullptr; }

	private:
		void release_() noexcept
		{
			if (_ptr && _ptr->DecRef() == 0)
				delete _ptr;
			_ptr = nullptr;
		}

		T* _ptr = nullptr;
	};

	template <class T1, class T2>
	[[nodiscard]] constexpr bool operator==(const BSTSmartPointer<T1>& a_lhs, T2* a_rhs)
	{
		return a_lhs.get() == a_rhs;
	}
	template <class T1, class T2>
	[[nodiscard]] constexpr bool operator!=(const BSTSmartPointer<T1>& a_lhs, T2* a_rhs)
	{
		return a_lhs.get() != a_rhs;
	}
	template <class T1>
	[[nodiscard]] constexpr bool operator==(const BSTSmartPointer<T1>& a_lhs, std::nullptr_t)
	{
		return a_lhs.get() == nullptr;
	}
	template <class T1>
	[[nodiscard]] constexpr bool operator!=(const BSTSmartPointer<T1>& a_lhs, std::nullptr_t)
	{
		return a_lhs.get() != nullptr;
	}

	template <class T, class... Args>
	[[nodiscard]] BSTSmartPointer<T> make_smart(Args&&... a_args)
	{
		BSTSmartPointer<T> r;
		r.reset(new T(std::forward<Args>(a_args)...));
		return r;
	}

	// Interned-string stand-in. The mod's BSFixedString interns + compares by
	// pooled pointer; for a single-threaded standalone viewer plain std::string
	// value semantics are correct (just O(len) compare instead of O(1)).
	class BSFixedString
	{
	public:
		BSFixedString() = default;
		BSFixedString(const char* a_s) :
			_s(a_s ? a_s : "") {}
		BSFixedString(const std::string& a_s) :
			_s(a_s) {}

		[[nodiscard]] const char* data() const noexcept { return _s.c_str(); }
		[[nodiscard]] const char* c_str() const noexcept { return _s.c_str(); }
		[[nodiscard]] bool empty() const noexcept { return _s.empty(); }
		[[nodiscard]] const std::string& str() const noexcept { return _s; }

		bool operator==(const BSFixedString& a_rhs) const { return _s == a_rhs._s; }
		bool operator!=(const BSFixedString& a_rhs) const { return _s != a_rhs._s; }
		bool operator==(const char* a_rhs) const { return _s == (a_rhs ? a_rhs : ""); }
		bool operator!=(const char* a_rhs) const { return !(*this == a_rhs); }

	private:
		std::string _s;
	};
}

namespace hdt
{
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
	struct hash<RE::BSFixedString>
	{
		[[nodiscard]] std::size_t operator()(const RE::BSFixedString& a_key) const noexcept
		{
			return std::hash<std::string>{}(a_key.str());
		}
	};
}

// --- logging shim: core only uses logger::info; make all levels no-ops ------
namespace logger
{
	template <class... Args> void trace(Args&&...) {}
	template <class... Args> void debug(Args&&...) {}
	template <class... Args> void info(Args&&...) {}
	template <class... Args> void warn(Args&&...) {}
	template <class... Args> void error(Args&&...) {}
	template <class... Args> void critical(Args&&...) {}
}
