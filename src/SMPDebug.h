#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace RE
{
	class TESObjectREFR;
}

namespace hdt
{
	// printf a line to the game console AND into an in-memory ring buffer the menu's Output panels show, so
	// smp command output is visible right in the menu without opening the console. Thread-safe: `smp report`
	// finishes on a worker thread and echoes its result from there.
	void smpEcho(const char* fmt, ...);

	// Append a (possibly multi-line) block of generated text to the menu ring buffer ONLY --- no game-console
	// print, no log write. Used for the big generated outputs (validation report content, node-tree dumps,
	// profiler tables) that must be readable in the menu but would flood the game console. Thread-safe.
	void menuConsoleAppendLines(std::string_view text);

	// A copy of the captured console lines, oldest first (the menu renders these). Returned by value so the
	// caller never holds the internal lock while drawing.
	std::vector<std::string> menuConsoleSnapshot();

	// Drop all captured lines (the Commands page's Clear button).
	void clearMenuConsole();

	// Run an "smp" subcommand by name, exactly as the console would. Split out of the console entry point
	// (SMPDebug_Execute in main.cpp) so the in-game menu's Commands page can trigger the same actions
	// (reset / list / detail / report / ...) without synthesizing a console command string. buffer is the
	// subcommand (e.g. "list"); buffer2/buffer3 are its optional args (e.g. "gear"). a_thisObj is the
	// console's targeted reference, used only by "dumptree"; pass the player or nullptr from the menu.
	// Returns true when the command was handled.
	bool RunSMPDebugCommand(const char* buffer, const char* buffer2, const char* buffer3,
		RE::TESObjectREFR* a_thisObj);
}
