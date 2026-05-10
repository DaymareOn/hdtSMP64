#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace hdt
{
	// Build a local timestamp string in format YYYYMMDD_HHMMSS.
	inline std::string BuildTimestampStringForFilenames()
	{
		auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::tm tmBuf{};
		localtime_s(&tmBuf, &t);
		std::ostringstream ss;
		ss << std::put_time(&tmBuf, "%Y%m%d_%H%M%S");
		return ss.str();
	}

}  // namespace hdt
