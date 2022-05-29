#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
using namespace std;
using namespace chrono;

namespace hdt
{
	class Log
	{
		Log::Log() {}

		Log::~Log() {}

	public:
		static Log* Log::instance()
		{
			static Log s;
			return &s;
		}

		int level = 0;
	};



	inline std::string now()
	{
		time_t now_time = system_clock::to_time_t(system_clock::now());
		std::stringstream ss;
		ss << std::put_time(gmtime(&now_time), "%Y-%m-%d %H:%M:%S");
		return ss.str() + " UTC";
	}

	inline void _FATALERROR(const char* fmt, ...)
	{
		va_list args;
		va_start(args, fmt);
		std::string newfmt = now() + " [FatalError] " + fmt;
		gLog.Log(IDebugLog::kLevel_FatalError, newfmt.c_str(), args);
		va_end(args);
	};

	inline void _ERROR(const char* fmt, ...)
	{
		if (Log::instance()->level < IDebugLog::kLevel_Error) return;

		va_list args;
		va_start(args, fmt);
		std::string newfmt = now() + " [Error] " + fmt;
		gLog.Log(IDebugLog::kLevel_Error, newfmt.c_str(), args);
		va_end(args);
	}

	inline void _WARNING(const char* fmt, ...)
	{
		if (Log::instance()->level < IDebugLog::kLevel_Warning) return;

		va_list args;
		va_start(args, fmt);
		std::string newfmt = now() + " [Warning] " + fmt;
		gLog.Log(IDebugLog::kLevel_Warning, newfmt.c_str(), args);
		va_end(args);
	}

	inline void _MESSAGE(const char* fmt, ...)
	{
		if (Log::instance()->level < IDebugLog::kLevel_Message) return;

		va_list args;
		va_start(args, fmt);
		std::string newfmt = now() + " [Message] " + fmt;
		gLog.Log(IDebugLog::kLevel_Message, newfmt.c_str(), args);
		va_end(args);
	}

	inline void _VMESSAGE(const char* fmt, ...)
	{
		if (Log::instance()->level < IDebugLog::kLevel_VerboseMessage) return;

		va_list args;
		va_start(args, fmt);
		std::string newfmt = now() + " [Verbose] " + fmt;
		gLog.Log(IDebugLog::kLevel_VerboseMessage, newfmt.c_str(), args);
		va_end(args);
	}

	inline void _DMESSAGE(const char* fmt, ...)
	{
		if (Log::instance()->level < IDebugLog::kLevel_DebugMessage) return;

		va_list args;
		va_start(args, fmt);
		std::string newfmt = now() + " [Debug] " + fmt;
		gLog.Log(IDebugLog::kLevel_DebugMessage, newfmt.c_str(), args);
		va_end(args);
	}
}