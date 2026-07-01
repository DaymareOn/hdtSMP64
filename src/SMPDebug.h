#pragma once

namespace RE
{
	class TESObjectREFR;
}

namespace hdt
{
	// Run an "smp" subcommand by name, exactly as the console would. Split out of the console entry point
	// (SMPDebug_Execute in main.cpp) so the in-game menu's Commands page can trigger the same actions
	// (reset / list / detail / report / ...) without synthesizing a console command string. buffer is the
	// subcommand (e.g. "list"); buffer2/buffer3 are its optional args (e.g. "gear"). a_thisObj is the
	// console's targeted reference, used only by "dumptree"; pass the player or nullptr from the menu.
	// Returns true when the command was handled.
	bool RunSMPDebugCommand(const char* buffer, const char* buffer2, const char* buffer3,
		RE::TESObjectREFR* a_thisObj);
}
