#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace RE
{
	template <class T>
	using BSTSmartPointer = std::shared_ptr<T>;

	struct BSFixedString
	{
		BSFixedString(const char* a_text) :
			text(a_text)
		{}
		const char* c_str() const { return text.c_str(); }
		std::string text;
	};
}

namespace logger
{
	inline std::string lastMessage;

	template <class... Args>
	void info(std::format_string<Args...> a_format, Args&&... a_args)
	{
		lastMessage = std::format(a_format, std::forward<Args>(a_args)...);
	}

	template <class... Args>
	void warn(std::format_string<Args...> a_format, Args&&... a_args)
	{
		lastMessage = std::format(a_format, std::forward<Args>(a_args)...);
	}
}
