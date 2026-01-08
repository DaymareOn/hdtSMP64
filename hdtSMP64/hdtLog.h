#pragma once

// Unified timestamped logging for hdtSMP64
// Uses SKSE's gLog (writes to hdtSMP64.log) with timestamps

#include <ctime>
#include <cstdio>
#include <cstdarg>
#include <mutex>

// Forward declare gLog from SKSE
extern IDebugLog gLog;

namespace hdt
{
	enum class LogLevel
	{
		Debug = 0,
		Info = 1,
		Warning = 2,
		Error = 3
	};

	class Logger
	{
	public:
		static Logger& getInstance()
		{
			static Logger instance;
			return instance;
		}

		// init() no longer needed - we use gLog which is already initialized
		void init(const char* /*filename*/)
		{
			m_initialized = true;
		}

		void setLevel(LogLevel level) { m_level = level; }
		void setEnabled(bool enabled) { m_enabled = enabled; }

		void log(LogLevel level, const char* format, ...)
		{
			if (!m_enabled || level < m_level)
				return;

			std::lock_guard<std::mutex> lock(m_mutex);

			try
			{
				// Format timestamp
				char timeBuf[24];
				auto now = std::time(nullptr);
				auto tm = *std::localtime(&now);
				std::strftime(timeBuf, sizeof(timeBuf), "[%H:%M:%S]", &tm);

				// Level prefix
				const char* levelStr = "";
				switch (level)
				{
				case LogLevel::Debug:   levelStr = "[DEBUG]"; break;
				case LogLevel::Info:    levelStr = "[INFO] "; break;
				case LogLevel::Warning: levelStr = "[WARN] "; break;
				case LogLevel::Error:   levelStr = "[ERROR]"; break;
				}

				// Format message
				char msgBuf[2048];
				va_list args;
				va_start(args, format);
				vsnprintf(msgBuf, sizeof(msgBuf), format, args);
				va_end(args);

				// Combine and send to gLog
				char finalBuf[2200];
				snprintf(finalBuf, sizeof(finalBuf), "%s %s %s", timeBuf, levelStr, msgBuf);
				gLog.Message(finalBuf);
			}
			catch (...)
			{
				// Silently fail - don't crash the game over logging
			}
		}

		void close() { /* gLog handles its own cleanup */ }

	private:
		Logger() : m_enabled(true), m_level(LogLevel::Info), m_initialized(false) {}
		~Logger() = default;

		std::mutex m_mutex;
		bool m_enabled;
		bool m_initialized;
		LogLevel m_level;
	};

	// Convenience macros - these now write to the main hdtSMP64.log with timestamps
	#define HDT_LOG_DEBUG(fmt, ...) hdt::Logger::getInstance().log(hdt::LogLevel::Debug, fmt, ##__VA_ARGS__)
	#define HDT_LOG_INFO(fmt, ...)  hdt::Logger::getInstance().log(hdt::LogLevel::Info, fmt, ##__VA_ARGS__)
	#define HDT_LOG_WARN(fmt, ...)  hdt::Logger::getInstance().log(hdt::LogLevel::Warning, fmt, ##__VA_ARGS__)
	#define HDT_LOG_ERROR(fmt, ...) hdt::Logger::getInstance().log(hdt::LogLevel::Error, fmt, ##__VA_ARGS__)

	// Timestamped replacements for SKSE logging macros
	// Use these instead of _MESSAGE, _WARNING, etc. for timestamped output
	namespace logging
	{
		inline void TimestampedMessage(const char* fmt, ...)
		{
			char timeBuf[16];
			auto now = std::time(nullptr);
			auto tm = *std::localtime(&now);
			std::strftime(timeBuf, sizeof(timeBuf), "[%H:%M:%S] ", &tm);

			char msgBuf[4096];
			va_list args;
			va_start(args, fmt);
			vsnprintf(msgBuf, sizeof(msgBuf), fmt, args);
			va_end(args);

			char finalBuf[4200];
			snprintf(finalBuf, sizeof(finalBuf), "%s%s", timeBuf, msgBuf);
			gLog.Message(finalBuf);
		}
	}

	// Drop-in timestamped replacement for _MESSAGE
	#define _TMESSAGE(fmt, ...) hdt::logging::TimestampedMessage(fmt, ##__VA_ARGS__)
}
