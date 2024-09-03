#pragma once

namespace hdt
{
	// Unified Const String Class
	class IString : 
		public RE::BSIntrusiveRefCounted
	{
	public:
		virtual ~IString() = default;
		
		// add
		virtual const char* cstr() const = 0;
		virtual size_t size() const = 0;
	};
}
