#pragma once

namespace hdt::physicsprofiler
{
	struct DumpOptions
	{
		double m_minScopeMs = 0.001;
		std::uint32_t m_maxDepth = 64;
		std::uint32_t m_flatLimit = 32;
		bool m_logTree = true;
		bool m_logFlat = true;
		bool m_logThreads = true;
	};

	void install();
	void uninstall();

	void enter(const char* a_name) noexcept;
	void leave() noexcept;

	void endFrame();
	std::uint64_t frameCountSinceReset();
	std::uint64_t totalFrameCount();

	void setProfileHistory(std::uint64_t a_frames);
	std::uint64_t getProfileHistory();

	bool reset();
	bool dumpAndReset(const DumpOptions& a_options = {});
	bool dumpEvery(std::uint64_t a_frames, const DumpOptions& a_options = {});
}
