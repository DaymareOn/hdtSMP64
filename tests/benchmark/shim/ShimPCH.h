#pragma once

// ShimPCH.h - the standalone "Reverse Engineered types" shim for the smp_replay harness (D2).
//
// The in-game DLL force-includes src/PCH.h, which pulls in CommonLibSSE (RE::*, SKSE::*, logger).
// The harness force-includes THIS header instead. It provides just the handful of RE symbols the
// shared physics core (src/hdtSkinnedMesh/*) actually touches - BSFixedString, BSIntrusiveRefCounted,
// BSTSmartPointer, make_smart, and a no-op logger - implemented against the C++ standard library and
// Bullet only. The core then compiles unchanged; it never links the real CommonLibSSE, so there is
// no ODR clash (§10).
//
// This is deliberately minimal. It is NOT a general RE reimplementation - only what the core needs.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <tbb/tbb.h>

namespace RE
{
	// Interned string. The game pools these; the shim just owns a std::string. Hashing/equality are
	// by value (the game can compare by pointer because of pooling - the shim must not).
	class BSFixedString
	{
	public:
		BSFixedString() = default;
		BSFixedString(const char* s) :
			m_str(s ? s : "") {}
		BSFixedString(const std::string& s) :
			m_str(s) {}

		const char* c_str() const { return m_str.c_str(); }
		const char* data() const { return m_str.c_str(); }
		bool empty() const { return m_str.empty(); }
		std::size_t size() const { return m_str.size(); }

		bool operator==(const BSFixedString& rhs) const { return m_str == rhs.m_str; }
		bool operator!=(const BSFixedString& rhs) const { return m_str != rhs.m_str; }

		const std::string& str() const { return m_str; }

	private:
		std::string m_str;
	};

	// Intrusive refcount base. BSTSmartPointer drives IncRef/DecRef; the smart pointer (which knows
	// the full type) performs the delete, so multiply-inheriting types (e.g. SkinnedMeshBody, which is
	// also a btCollisionObject) are destroyed through the correct type.
	class BSIntrusiveRefCounted
	{
	public:
		std::uint32_t IncRef() const { return ++m_refCount; }
		std::uint32_t DecRef() const { return --m_refCount; }

	private:
		mutable std::atomic<std::uint32_t> m_refCount{ 0 };
	};

	template <class T>
	class BSTSmartPointer
	{
	public:
		BSTSmartPointer() = default;
		BSTSmartPointer(std::nullptr_t) {}

		explicit BSTSmartPointer(T* ptr) :
			m_ptr(ptr)
		{
			if (m_ptr)
				m_ptr->IncRef();
		}

		BSTSmartPointer(const BSTSmartPointer& o) :
			m_ptr(o.m_ptr)
		{
			if (m_ptr)
				m_ptr->IncRef();
		}

		template <class U>
		BSTSmartPointer(const BSTSmartPointer<U>& o) :
			m_ptr(o.get())
		{
			if (m_ptr)
				m_ptr->IncRef();
		}

		BSTSmartPointer(BSTSmartPointer&& o) noexcept :
			m_ptr(o.m_ptr)
		{
			o.m_ptr = nullptr;
		}

		~BSTSmartPointer() { release(); }

		BSTSmartPointer& operator=(const BSTSmartPointer& o)
		{
			reset(o.m_ptr);
			return *this;
		}

		BSTSmartPointer& operator=(BSTSmartPointer&& o) noexcept
		{
			if (this != &o) {
				release();
				m_ptr = o.m_ptr;
				o.m_ptr = nullptr;
			}
			return *this;
		}

		void reset() { release(); }

		void reset(T* ptr)
		{
			if (ptr == m_ptr)
				return;
			release();
			m_ptr = ptr;
			if (m_ptr)
				m_ptr->IncRef();
		}

		T* get() const { return m_ptr; }
		T* operator->() const { return m_ptr; }
		T& operator*() const { return *m_ptr; }
		explicit operator bool() const { return m_ptr != nullptr; }

	private:
		void release()
		{
			if (m_ptr) {
				if (m_ptr->DecRef() == 0)
					delete m_ptr;
				m_ptr = nullptr;
			}
		}

		T* m_ptr = nullptr;
	};

	template <class T1, class T2>
	bool operator==(const BSTSmartPointer<T1>& a, const BSTSmartPointer<T2>& b) { return a.get() == b.get(); }
	template <class T1, class T2>
	bool operator!=(const BSTSmartPointer<T1>& a, const BSTSmartPointer<T2>& b) { return a.get() != b.get(); }
	template <class T1, class T2>
	bool operator==(const BSTSmartPointer<T1>& a, T2* b) { return a.get() == b; }
	template <class T1, class T2>
	bool operator!=(const BSTSmartPointer<T1>& a, T2* b) { return a.get() != b; }

	// CommonLibSSE's constructing factory (used by hdtSkinnedMeshShape.cpp).
	template <class T, class... Args>
	BSTSmartPointer<T> make_smart(Args&&... args)
	{
		return BSTSmartPointer<T>(new T(std::forward<Args>(args)...));
	}
}

namespace std
{
	template <>
	struct hash<RE::BSFixedString>
	{
		std::size_t operator()(const RE::BSFixedString& k) const noexcept
		{
			return std::hash<std::string>{}(k.str());
		}
	};
}

// hdt::make_smart - the wrapper from src/PCH.h, replicated for the shim build.
namespace hdt
{
	template <class T>
	RE::BSTSmartPointer<T> make_smart(T* ptr)
	{
		RE::BSTSmartPointer<T> result;
		result.reset(ptr);
		return result;
	}
}

// Minimal stand-in for `namespace logger = SKSE::log;`. The core makes exactly one logger call
// (thread-count report); swallow it. Format args are accepted and ignored.
namespace logger
{
	template <class... Args>
	inline void info(Args&&...) {}
	template <class... Args>
	inline void warn(Args&&...) {}
	template <class... Args>
	inline void error(Args&&...) {}
	template <class... Args>
	inline void debug(Args&&...) {}
	template <class... Args>
	inline void critical(Args&&...) {}
}
