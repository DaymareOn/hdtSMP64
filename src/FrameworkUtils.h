#pragma once

#include "StringImpl.h"

namespace hdt
{
	class IDStr
	{
	public:
		// 1
		IDStr() = default;

		// 2 (copy)
		IDStr(const IDStr& a_rhs) :
			_ptr(a_rhs._ptr)
		{}

		// 3 (move)
		IDStr(IDStr&& a_rhs) noexcept :
			_ptr(std::exchange(a_rhs._ptr, nullptr))
		{}

		// 4 (init)
		IDStr(const char* str) :
			_ptr(!str ? nullptr : StringManager::instance()->get(str, str + strlen(str)))
		{
		}

		// 5 (init)
		IDStr(const std::string& str) :
			_ptr(!str.c_str() ? nullptr : StringManager::instance()->get(str.c_str(), str.c_str() + str.length()))
		{}

		// 0 (copy)
		inline IDStr& operator=(const IDStr& a_rhs)
		{
			if (this != std::addressof(a_rhs)) {
				_ptr = a_rhs._ptr;
			}

			return *this;
		}

		// (move)
		inline IDStr& operator=(IDStr&& a_rhs) noexcept
		{
			if (this != std::addressof(a_rhs)) {
				_ptr = a_rhs._ptr;
				a_rhs._ptr = nullptr;
			}

			return *this;
		}

		//
		explicit constexpr operator bool() const noexcept
		{
			return static_cast<bool>(_ptr.get());
		}

		bool operator==(const IDStr& lhs) const noexcept
		{
			return _ptr == lhs._ptr;
		}

		constexpr IString& operator*() const noexcept
		{
			return *_ptr;
		}

		constexpr IString* operator->() const noexcept
		{
			return _ptr.get();
		}

		constexpr operator IString*() const noexcept
		{
			return _ptr.get();
		}

		operator RE::BSTSmartPointer<IString>() const noexcept
		{
			return _ptr;
		}

		constexpr IString* get() const noexcept
		{
			return _ptr.get();
		}

		inline bool operator==(const IDStr& a_rhs)
		{
			return _ptr == a_rhs._ptr;
		}

		inline bool operator!=(const IDStr& a_rhs)
		{
			return !(*this == a_rhs);
		}
	private:
		RE::BSTSmartPointer<IString> _ptr{ nullptr };
	};
}
