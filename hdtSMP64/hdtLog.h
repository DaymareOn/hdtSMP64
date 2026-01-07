#pragma once

#include <fstream>
#include <mutex>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdarg>

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

		void init(const char* filename)
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			try
			{
				if (m_file.is_open())
					m_file.close();
				m_file.open(filename, std::ios::out | std::ios::trunc);
				if (m_file.is_open())
				{
					// Write directly without calling log() to avoid lock recursion
					auto now = std::time(nullptr);
					auto tm = *std::localtime(&now);
					m_file << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S] ");
					m_file << "[INFO]  hdtSMP64 Logger initialized" << std::endl;
					m_file.flush();
				}
			}
			catch (...)
			{
				// Silently fail - don't crash the game over logging
			}
		}

		void setLevel(LogLevel level) { m_level = level; }
		void setEnabled(bool enabled) { m_enabled = enabled; }

		void log(LogLevel level, const char* format, ...)
		{
			if (!m_enabled || level < m_level)
				return;

			std::lock_guard<std::mutex> lock(m_mutex);

			if (!m_file.is_open())
				return;

			try
			{
				// Timestamp
				auto now = std::time(nullptr);
				auto tm = *std::localtime(&now);
				m_file << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S] ");

				// Level
				switch (level)
				{
				case LogLevel::Debug:   m_file << "[DEBUG] "; break;
				case LogLevel::Info:    m_file << "[INFO]  "; break;
				case LogLevel::Warning: m_file << "[WARN]  "; break;
				case LogLevel::Error:   m_file << "[ERROR] "; break;
				}

				// Message
				char buffer[2048];
				va_list args;
				va_start(args, format);
				vsnprintf(buffer, sizeof(buffer), format, args);
				va_end(args);

				m_file << buffer << std::endl;
				m_file.flush();
			}
			catch (...)
			{
				// Silently fail - don't crash the game over logging
			}
		}

		void close()
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			try
			{
				if (m_file.is_open())
				{
					auto now = std::time(nullptr);
					auto tm = *std::localtime(&now);
					m_file << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S] ");
					m_file << "[INFO]  Logger shutting down" << std::endl;
					m_file.close();
				}
			}
			catch (...)
			{
				// Silently fail
			}
		}

	private:
		Logger() : m_enabled(true), m_level(LogLevel::Info) {}
		~Logger() { close(); }

		std::ofstream m_file;
		std::mutex m_mutex;
		bool m_enabled;
		LogLevel m_level;
	};

	// Convenience macros
	#define HDT_LOG_DEBUG(fmt, ...) hdt::Logger::getInstance().log(hdt::LogLevel::Debug, fmt, ##__VA_ARGS__)
	#define HDT_LOG_INFO(fmt, ...)  hdt::Logger::getInstance().log(hdt::LogLevel::Info, fmt, ##__VA_ARGS__)
	#define HDT_LOG_WARN(fmt, ...)  hdt::Logger::getInstance().log(hdt::LogLevel::Warning, fmt, ##__VA_ARGS__)
	#define HDT_LOG_ERROR(fmt, ...) hdt::Logger::getInstance().log(hdt::LogLevel::Error, fmt, ##__VA_ARGS__)
}
