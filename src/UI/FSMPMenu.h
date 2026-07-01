#pragma once

namespace hdt::FSMPMenu
{
	// Register FSMP's pages with the SKSE Menu Framework's Mod Control Panel: one "FSMP" section with a
	// page per config area (Simplification / Performance / Wind / Validation / Logs / Commands / Presets).
	// Call once, after the framework is up (kPostPostLoad). Safe to call when the framework is not
	// installed --- it simply no-ops via the SDK.
	void Register();
}
