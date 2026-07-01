#include "UI/FSMPMenu.h"

#include "ActorManager.h"
#include "GlobalConfig.h"
#include "SMPDebug.h"
#include "UI/Localization.h"
#include "Validator/hdtAssetValidator.h"
#include "config.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <shellapi.h>  // ShellExecuteA --- open the About-page links in the user's browser
#pragma comment(lib, "Shell32.lib")

#include "SKSEMenuFramework.h"

// In-DLL configuration UI. The SKSE Menu Framework owns the D3D11/ImGui hook and calls our Render* functions
// while its panel is open; we emit ImGui widgets (under the ImGuiMCP namespace, forwarded into
// SKSEMenuFramework.dll) bound to the live physics singletons. Edits persist to userConfigs.json and re-apply
// through the same path as `smp reset`.
//
// Every user-visible string is wrapped in tr() (see UI/Localization): it returns the translation for the
// current language, or the English source string when there is none. tip() and the persistent help line
// translate their argument too. The Language dropdown (header / About) switches language live.
//
// Layout: each page is a status header (logo, version, master ON/OFF pill, language picker, search box), then
// settings laid out as aligned three-column tables (label | control | reset-to-default), then a help line
// that echoes the hovered setting's description. Apply policy is unchanged from a plain `smp reset`:
//   - toggles/buttons commit immediately on click;
//   - sliders preview live (they write the singleton every drag frame) but only persist+reset on release
//     (IsItemDeactivatedAfterEdit), so we don't reset the physics world on every pixel of a drag;
//   - log level and mods-dir persist without a physics reset (they don't affect the simulation state).

namespace
{
	using hdt::ActorManager;
	using hdt::GlobalConfig;
	using hdt::SkyrimPhysicsWorld;
	using hdt::loc::tr;

	// Font Awesome 6 (Free, Solid) glyphs as raw UTF-8 bytes. Rendered only between FontAwesome::PushSolid()
	// and FontAwesome::Pop() (the framework supplies the icon font); purely decorative, so a wrong glyph shows
	// a placeholder box but never breaks a control.
	namespace fa
	{
		constexpr const char* Scissors  = "\xEF\x83\x84";  // U+F0C4 scissors
		constexpr const char* Bolt      = "\xEF\x83\xA7";  // U+F0E7 bolt
		constexpr const char* Wind      = "\xEF\x9C\xAE";  // U+F72E wind
		constexpr const char* Clipboard = "\xEF\x91\xAC";  // U+F46C clipboard-check
		constexpr const char* FileLines = "\xEF\x85\x9C";  // U+F15C file-lines
		constexpr const char* Terminal  = "\xEF\x84\xA0";  // U+F120 terminal
		constexpr const char* Sliders   = "\xEF\x87\x9E";  // U+F1DE sliders
		constexpr const char* Info      = "\xEF\x81\x9A";  // U+F05A circle-info
		constexpr const char* Undo      = "\xEF\x83\xA2";  // U+F0E2 arrow-rotate-left
		constexpr const char* Language  = "\xEF\x86\xAB";  // U+F1AB language
		constexpr const char* Link      = "\xEF\x83\x81";  // U+F0C1 link
		constexpr const char* Warning   = "\xEF\x81\xB1";  // U+F071 triangle-exclamation
		constexpr const char* Search    = "\xEF\x80\x82";  // U+F002 magnifying-glass
	}

	// Palette (ImVec4 RGBA, 0..1).
	constexpr ImGuiMCP::ImVec4 kAccent{ 0.40f, 0.72f, 1.00f, 1.00f };  // section icons / titles / links
	constexpr ImGuiMCP::ImVec4 kDim{ 0.62f, 0.62f, 0.66f, 1.00f };     // help line / secondary text
	constexpr ImGuiMCP::ImVec4 kWarn{ 0.98f, 0.78f, 0.20f, 1.00f };    // inline warnings
	constexpr ImGuiMCP::ImVec4 kGreen{ 0.16f, 0.55f, 0.22f, 1.00f };   // SMP ON
	constexpr ImGuiMCP::ImVec4 kGreenH{ 0.20f, 0.66f, 0.28f, 1.00f };
	constexpr ImGuiMCP::ImVec4 kRed{ 0.60f, 0.18f, 0.18f, 1.00f };     // SMP OFF
	constexpr ImGuiMCP::ImVec4 kRedH{ 0.72f, 0.22f, 0.22f, 1.00f };

	// Per-frame UI state. header() resets both at the top of every page render.
	char g_filter[96] = "";     // the search box contents
	bool g_filtering = false;   // true only while a filter page shows a non-empty search box
	const char* g_help = nullptr;  // English help text of the row under the mouse this frame (for the help line)

	// Persist current settings and rerun the full reload+reset sequence (the menu's equivalent of smp reset).
	void commitReset()
	{
		hdt::saveUserSettings();
		hdt::applyConfigReset();
	}

	// Localized tooltip on the previous widget; AllowWhenDisabled so greyed-out controls still explain
	// themselves. "%s" guards against a translation that contains a stray '%' being read as a format string.
	void tip(const char* english)
	{
		if (ImGuiMCP::IsItemHovered(ImGuiMCP::ImGuiHoveredFlags_AllowWhenDisabled))
			ImGuiMCP::SetTooltip("%s", tr(english));
	}

	// Case-insensitive ASCII substring test, used by the search box. Both sides are lower-cased first.
	bool icontains(const char* hay, const char* needle)
	{
		if (!needle || !*needle)
			return true;
		std::string h, n;
		for (const char* p = hay; p && *p; ++p)
			h += static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
		for (const char* p = needle; p && *p; ++p)
			n += static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
		return h.find(n) != std::string::npos;
	}

	// A settings row is shown when there is no active filter, or when the filter matches its label in either
	// the current language (what the user sees) or the English source (so a mod author's known name works too).
	bool visible(const char* label)
	{
		if (!g_filtering)
			return true;
		return icontains(tr(label), g_filter) || icontains(label, g_filter);
	}

	// A hidden ImGui id derived from the setting label, so identical widget types on different rows do not
	// collide. The returned string lives to the end of the full call expression, which is all ImGui needs.
	std::string hid(const char* label)
	{
		return std::string("##") + label;
	}

	std::string readFileBytes(const std::filesystem::path& path)
	{
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return {};
		return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
	}

	// ---- Header / chrome ---------------------------------------------------------------------------------

	// The master SMP on/off state as a green/red pill button; clicking it toggles SMP exactly like the console
	// `smp on` / `smp off` (so behaviour stays identical to the command).
	void masterPill()
	{
		const bool on = !SkyrimPhysicsWorld::get()->disabled;
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Button, on ? kGreen : kRed);
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ButtonHovered, on ? kGreenH : kRedH);
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ButtonActive, on ? kGreenH : kRedH);
		if (ImGuiMCP::Button(on ? tr("SMP is ON") : tr("SMP is OFF")))
			hdt::RunSMPDebugCommand(on ? "off" : "on", "", "", nullptr);
		ImGuiMCP::PopStyleColor(3);
		tip("Turn the whole SMP simulation on or off (same as 'smp on' / 'smp off').");
	}

	// A locale's native display name for the dropdown ("fr_fr" -> "Français (fr_fr)"). Regional codes fall
	// back to their base language's name; an unknown code shows raw. The code is always appended so the entry
	// stays identifiable even if the menu font lacks that script (e.g. CJK). This map is display-only --- the
	// set of available locales still comes from the folder scan in Localization.cpp.
	std::string localeName(const std::string& code)
	{
		static const std::unordered_map<std::string, std::string> names = {
			{ "en", "English" }, { "fr", "Français" }, { "de", "Deutsch" }, { "it", "Italiano" },
			{ "es", "Español" }, { "pl", "Polski" }, { "ru", "Русский" }, { "ja", "日本語" },
			{ "zh", "中文" }, { "cs", "Čeština" }, { "pt", "Português" }
		};
		const std::string base = code.substr(0, code.find('_'));
		const auto it = names.find(base);
		if (it == names.end())
			return code;
		return it->second + " (" + code + ")";
	}

	// Apply a chosen locale live: record the override, persist it to userConfigs.json, and reload the string
	// table. Page content re-renders translated on the next frame. (The page tab names, registered once at
	// startup, only re-translate on the next game launch.)
	void setLocale(const std::string& code)
	{
		hdt::g_locale = code;
		hdt::saveUserSettings();
		hdt::loc::load();
	}

	// The Language picker: a globe icon plus a combo listing "Auto" (follow Skyrim's language) and every
	// installed locale by native name. Selecting an entry switches the menu language immediately.
	void languageCombo()
	{
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kAccent, "%s", fa::Language);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();

		std::vector<std::string> locales = hdt::loc::availableLocales();
		std::sort(locales.begin(), locales.end());

		const std::string& cur = hdt::g_locale;  // "" = auto
		const std::string preview = cur.empty() ? std::string(tr("Auto (game language)")) : localeName(cur);

		ImGuiMCP::SetNextItemWidth(220.0f);
		if (ImGuiMCP::BeginCombo("##lang", preview.c_str())) {
			if (ImGuiMCP::Selectable(tr("Auto (game language)"), cur.empty()))
				setLocale("");
			for (const auto& l : locales)
				if (ImGuiMCP::Selectable(localeName(l).c_str(), cur == l))
					setLocale(l);
			ImGuiMCP::EndCombo();
		}
		tip("The menu language. 'Auto' follows Skyrim's own language.");
	}

	// The status header drawn atop every page: FSMP logo, name + version, the master pill, the language
	// picker, and (on settings pages) a search box. Also resets the per-frame help text and filter scope.
	void header(bool withFilter = true)
	{
		g_help = nullptr;
		g_filtering = withFilter && g_filter[0] != '\0';

		// Loaded once, lazily; a missing file yields a null id and we simply skip the image.
		static ImGuiMCP::ImTextureID logo =
			SKSEMenuFramework::LoadTexture("Data/SKSE/Plugins/hdtSkinnedMeshConfigs/FSMP.dds");
		if (logo) {
			ImGuiMCP::Image(logo, ImGuiMCP::ImVec2{ 34.0f * 768.0f / 444.0f, 34.0f });
			ImGuiMCP::SameLine();
		}
		ImGuiMCP::Text("Faster HDT-SMP");
		ImGuiMCP::SameLine();
		ImGuiMCP::TextDisabled("v%s", Plugin::VERSION.string().c_str());
		ImGuiMCP::SameLine();
		masterPill();
		ImGuiMCP::SameLine();
		languageCombo();

		if (withFilter) {
			ImGuiMCP::Spacing();
			FontAwesome::PushSolid();
			ImGuiMCP::TextColored(kDim, "%s", fa::Search);
			FontAwesome::Pop();
			ImGuiMCP::SameLine();
			ImGuiMCP::SetNextItemWidth(-FLT_MIN);
			ImGuiMCP::InputTextWithHint("##filter", tr("Search settings"), g_filter, sizeof(g_filter));
		}
		ImGuiMCP::Separator();
	}

	// An accent-coloured icon + title + separator introducing a group of settings. Hidden while a filter is
	// active, so the matching rows from any group show as one flat list.
	void section(const char* glyph, const char* title)
	{
		if (g_filtering)
			return;
		ImGuiMCP::Spacing();
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kAccent, "%s", glyph);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();
		ImGuiMCP::TextColored(kAccent, "%s", tr(title));
		ImGuiMCP::Separator();
	}

	// A yellow, wrapped warning line drawn full width (outside the settings table) for an inconsistent config.
	void warn(const char* englishMsg)
	{
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kWarn, "%s", fa::Warning);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Text, kWarn);
		ImGuiMCP::TextWrapped("%s", tr(englishMsg));
		ImGuiMCP::PopStyleColor();
	}

	// The persistent help line at the bottom of a page: the hovered setting's description, or a prompt when
	// nothing is hovered. Complements (does not replace) the per-control tooltips.
	void helpLine()
	{
		ImGuiMCP::Spacing();
		ImGuiMCP::Separator();
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Text, kDim);
		ImGuiMCP::TextWrapped("%s", g_help ? tr(g_help) : tr("Hover a setting to see help here."));
		ImGuiMCP::PopStyleColor();
	}

	// ---- Settings-row framework -------------------------------------------------------------------------

	// Open a three-column settings table: [label | control | reset]. Fixed label and reset widths so every
	// group on every page lines up. Returns false if the table couldn't open (caller then skips its rows).
	bool beginRows(const char* id)
	{
		if (!ImGuiMCP::BeginTable(id, 3, ImGuiMCP::ImGuiTableFlags_PadOuterX))
			return false;
		ImGuiMCP::TableSetupColumn("label", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, 300.0f);
		ImGuiMCP::TableSetupColumn("control", ImGuiMCP::ImGuiTableColumnFlags_WidthStretch);
		ImGuiMCP::TableSetupColumn("reset", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, 24.0f);
		return true;
	}
	void endRows()
	{
		ImGuiMCP::EndTable();
	}

	// Start a row: draw the label (col 0), remember its help on hover, and leave the cursor in the control
	// cell (col 1) with the item width stretched to fill it. Shared by every typed row and the custom rows.
	void rowLabel(const char* label, const char* help)
	{
		ImGuiMCP::TableNextRow();
		ImGuiMCP::TableNextColumn();
		ImGuiMCP::Text("%s", tr(label));
		if (ImGuiMCP::IsItemHovered(0))
			g_help = help;
		ImGuiMCP::TableNextColumn();
		ImGuiMCP::SetNextItemWidth(-FLT_MIN);
	}

	// Finish a row: draw the reset cell (col 2), a small undo button enabled only when the value differs from
	// its shipped default. Returns true when clicked. PushID(label) keeps identical glyph buttons distinct.
	bool rowReset(const char* label, bool differs)
	{
		ImGuiMCP::TableNextColumn();
		ImGuiMCP::PushID(label);
		ImGuiMCP::BeginDisabled(!differs);
		FontAwesome::PushSolid();
		const bool clicked = ImGuiMCP::SmallButton(fa::Undo);
		FontAwesome::Pop();
		ImGuiMCP::EndDisabled();
		if (differs)
			tip("Reset this setting to its default");
		ImGuiMCP::PopID();
		return clicked;
	}

	// Checkbox row. Updates *v in place; returns true (commit) when toggled or reset to default.
	bool rowCheck(const char* label, const char* help, bool* v, bool def)
	{
		if (!visible(label))
			return false;
		rowLabel(label, help);
		const bool changed = ImGuiMCP::Checkbox(hid(label).c_str(), v);
		if (ImGuiMCP::IsItemHovered(0))
			g_help = help;
		tip(help);
		const bool reset = rowReset(label, *v != def);
		if (reset)
			*v = def;
		return changed || reset;
	}

	// Float-slider row. Previews live (writes *v every drag frame); returns true (commit) on release or reset.
	bool rowFloat(const char* label, const char* help, float* v, float def, float lo, float hi, const char* fmt)
	{
		if (!visible(label))
			return false;
		rowLabel(label, help);
		ImGuiMCP::SliderFloat(hid(label).c_str(), v, lo, hi, fmt);
		if (ImGuiMCP::IsItemHovered(0))
			g_help = help;
		tip(help);
		const bool commit = ImGuiMCP::IsItemDeactivatedAfterEdit();
		const bool reset = rowReset(label, *v != def);
		if (reset)
			*v = def;
		return commit || reset;
	}

	// Int-slider row. Same commit policy as rowFloat.
	bool rowInt(const char* label, const char* help, int* v, int def, int lo, int hi)
	{
		if (!visible(label))
			return false;
		rowLabel(label, help);
		ImGuiMCP::SliderInt(hid(label).c_str(), v, lo, hi);
		if (ImGuiMCP::IsItemHovered(0))
			g_help = help;
		tip(help);
		const bool commit = ImGuiMCP::IsItemDeactivatedAfterEdit();
		const bool reset = rowReset(label, *v != def);
		if (reset)
			*v = def;
		return commit || reset;
	}

	// ---- Pages ------------------------------------------------------------------------------------------

	void __stdcall RenderSimplification()
	{
		header();
		auto* a = ActorManager::instance();
		auto* w = SkyrimPhysicsWorld::get();
		const GlobalConfig& d = hdt::shippedDefaults();

		section(fa::Scissors, "Disabling some SMP");
		if (beginRows("simpl.disable")) {
			if (rowCheck("Disable SMP hair when there's a wig",
					"Skip SMP hair when an armor occupies the hair/longhair slot (a wig on top of the hair).",
					&a->m_disableSMPHairWhenWigEquipped, d.disableSMPHairWhenWigEquipped))
				commitReset();
			if (rowCheck("No SMP for your character in 1st person view",
					"Skip the player's physics while in first-person view to save performance.",
					&a->m_disable1stPersonViewPhysics, d.disable1stPersonViewPhysics))
				commitReset();
			if (rowCheck("Enable NPC face parts physics",
					"Calculate physics for NPC face parts.",
					&a->m_skinNPCFaceParts, d.enableNPCFaceParts))
				commitReset();
			endRows();
		}

		section(fa::Sliders, "Distance / screen-size culling");
		if (beginRows("simpl.cull")) {
			if (rowFloat("Always-on distance",
					"SMP is always calculated for skeletons closer than this (units), even off-screen.",
					&a->m_minCullingDistance, d.minCullingDistance, 0.0f, 10000.0f, "%.0f"))
				commitReset();
			if (rowFloat("Min screen size %",
					"Skip non-player skeletons smaller than this % of screen height. 0 disables the check.",
					&a->m_minScreenSizePercent, d.minScreenSizePercent, 0.0f, 100.0f, "%.1f"))
				commitReset();
			if (rowCheck("Skip dead actors",
					"Skip physics for dead non-player actors (corpses). The player is never affected.",
					&a->m_skipDeadActors, d.skipDeadActors))
				commitReset();
			endRows();
		}

		section(fa::Bolt, "Limiting active SMP NPCs");
		if (beginRows("simpl.limit")) {
			if (rowInt("Maximum SMP NPCs",
					"Upper bound on simultaneously simulated skeletons (including the player).",
					&a->m_maxActiveSkeletons, d.maximumActiveSkeletons, 0, 200))
				commitReset();
			if (rowCheck("Auto-adjust the max number of SMP NPCs",
					"Dynamically reduce active skeletons to stay within the frame-time budget below.",
					&a->m_autoAdjustMaxSkeletons, d.autoAdjustMaxSkeletons))
				commitReset();
			// The budget and sample size only matter when auto-adjust is on.
			ImGuiMCP::BeginDisabled(!a->m_autoAdjustMaxSkeletons);
			if (rowFloat("Frame-time budget (ms)",
					"How many ms/frame SMP may spend before it starts dropping active skeletons.",
					&w->m_budgetMs, d.budgetMs, 0.1f, 20.0f, "%.1f"))
				commitReset();
			if (rowInt("Adjustment speed (sample size)",
					"How many samples to average for the auto-adjuster. Higher = smoother but slower to react.",
					&w->m_sampleSize, d.sampleSize, 1, 50))
				commitReset();
			ImGuiMCP::EndDisabled();
			endRows();
		}

		helpLine();
	}

	void __stdcall RenderPerformance()
	{
		header();
		auto* w = SkyrimPhysicsWorld::get();
		auto& si = w->getSolverInfo();
		const GlobalConfig& d = hdt::shippedDefaults();

		section(fa::Sliders, "Simulation quality");
		if (beginRows("perf.quality")) {
			if (rowInt("Solver iterations",
					"The physics engine's solver iterations. Higher = more accurate, more CPU.",
					&si.m_numIterations, d.numIterations, 4, 128))
				commitReset();
			if (rowFloat("ERP",
					"Error-reduction force pulling constraints back into place each step.",
					&si.m_erp, d.erp, 0.01f, 1.0f, "%.2f"))
				commitReset();
			endRows();
		}

		section(fa::Bolt, "Simulation frequency");
		if (beginRows("perf.freq")) {
			if (rowCheck("Use real time",
					"Drive physics from the real-world clock instead of the in-game clock (better under slow-time).",
					&w->m_useRealTime, d.useRealTime))
				commitReset();
			if (rowInt("Simulation frequency (min-fps)",
					"Physics steps per second. Never set below 60 or the physics engine misbehaves. Higher = smoother, more CPU.",
					&w->min_fps, d.minFps, 60, 300))
				commitReset();
			if (rowInt("Max sub-steps",
					"Max physics steps per frame. Slowdowns occur below (min-fps / maxSubSteps) fps.",
					&w->m_maxSubSteps, d.maxSubSteps, 1, 60))
				commitReset();
			endRows();
		}

		section(fa::Undo, "Rotation limits");
		if (beginRows("perf.rot")) {
			if (rowCheck("Limit rotation speed",
					"Rotate the player slowly through large turns instead of instantly (prevents physics explosions).",
					&w->m_clampRotations, d.clampRotations))
				commitReset();
			ImGuiMCP::BeginDisabled(!w->m_clampRotations);
			if (rowFloat("Rotation speed limit (rad/s)",
					"Maximum rotation speed in radians per second when limiting is on.",
					&w->m_rotationSpeedLimit, d.rotationSpeedLimit, 0.0f, 100.0f, "%.1f"))
				commitReset();
			ImGuiMCP::EndDisabled();
			ImGuiMCP::BeginDisabled(w->m_clampRotations);
			if (rowCheck("Reset SMP on big unlimited turns",
					"When rotation isn't limited, reset physics on a large turn instead of simulating the whole sweep.",
					&w->m_unclampedResets, d.unclampedResets))
				commitReset();
			ImGuiMCP::BeginDisabled(!w->m_unclampedResets);
			if (rowFloat("Reset angle (degrees)",
					"Turn angle (degrees) above which the reset is triggered.",
					&w->m_unclampedResetAngle, d.unclampedResetAngle, 0.0f, 360.0f, "%.0f"))
				commitReset();
			ImGuiMCP::EndDisabled();
			ImGuiMCP::EndDisabled();
			endRows();
		}

		helpLine();
	}

	void __stdcall RenderWind()
	{
		header();
		auto* w = SkyrimPhysicsWorld::get();
		const GlobalConfig& d = hdt::shippedDefaults();

		section(fa::Wind, "Wind");
		if (beginRows("wind")) {
			if (rowCheck("Enable FSMP-native wind",
					"Apply FSMP's own wind force to physics objects.",
					&w->m_enableWind, d.windEnabled))
				commitReset();
			ImGuiMCP::BeginDisabled(!w->m_enableWind);
			if (rowFloat("Wind strength",
					"Base wind strength. For reference, gravity is 9.8.",
					&w->m_windStrength, d.windStrength, 0.0f, 100.0f, "%.1f"))
				commitReset();
			if (rowFloat("Distance for no wind",
					"How close to an obstruction for wind to be fully blocked.",
					&w->m_distanceForNoWind, d.distanceForNoWind, 0.0f, 10000.0f, "%.0f"))
				commitReset();
			if (rowFloat("Distance for max wind",
					"How far from an obstruction for wind to be unblocked. Scales linearly with the above.",
					&w->m_distanceForMaxWind, d.distanceForMaxWind, 0.0f, 10000.0f, "%.0f"))
				commitReset();
			ImGuiMCP::EndDisabled();
			endRows();
		}

		if (!g_filtering && w->m_enableWind && w->m_distanceForNoWind >= w->m_distanceForMaxWind)
			warn("Distance for no wind should be below distance for max wind.");

		helpLine();
	}

	void __stdcall RenderValidation()
	{
		header();
		const GlobalConfig& d = hdt::shippedDefaults();  // d.modsDir is empty ("scan the VFS")

		// The exact same English string is the label, the tooltip, and the help-line text, and it is the
		// localization key --- keep it in one place so all three (and the translation) stay in sync.
		static constexpr const char* kModsHelp =
			"Your mod manager's mods folder (MO2 mods/ or Vortex staging). When set, 'smp report' scans it\n"
			"natively instead of through the virtual file system, which is much faster on big load orders.\n"
			"Leave empty to scan data/ through the VFS.";

		section(fa::Clipboard, "Mods folder");
		if (beginRows("valid")) {
			if (visible("Mods folder")) {
				// Keep the edit buffer synced with the live value except while the user is actively typing.
				static char modsBuf[1024] = "";
				static bool editing = false;
				if (!editing) {
					const auto& s = hdt::g_validationConfig.modsDir;
					std::strncpy(modsBuf, s.c_str(), sizeof(modsBuf) - 1);
					modsBuf[sizeof(modsBuf) - 1] = '\0';
				}
				rowLabel("Mods folder", kModsHelp);
				ImGuiMCP::InputText(hid("Mods folder").c_str(), modsBuf, sizeof(modsBuf));
				if (ImGuiMCP::IsItemHovered(0))
					g_help = kModsHelp;
				tip(kModsHelp);
				editing = ImGuiMCP::IsItemActive();
				if (ImGuiMCP::IsItemDeactivatedAfterEdit()) {
					hdt::g_validationConfig.modsDir = modsBuf;
					hdt::saveUserSettings();  // no physics reset; mods-dir only affects validation
				}
				if (rowReset("Mods folder", hdt::g_validationConfig.modsDir != d.modsDir)) {
					hdt::g_validationConfig.modsDir = d.modsDir;
					modsBuf[0] = '\0';
					editing = false;
					hdt::saveUserSettings();
				}
			}
			endRows();
		}

		helpLine();
	}

	void __stdcall RenderLogs()
	{
		header();
		const GlobalConfig& d = hdt::shippedDefaults();
		static constexpr const char* kLogHelp = "0 = Fatal, 1 = Error, 2 = Warning, 3 = Message, 4 = Verbose, 5 = Debug.";

		section(fa::FileLines, "Logs");
		if (beginRows("logs")) {
			if (visible("Log level")) {
				// g_logLevel stores the inverted spdlog scale, so bind a local 0..5 value and apply it live
				// on any change (so a drag accumulates); persist on release only (logging needs no reset).
				rowLabel("Log level", kLogHelp);
				int level = std::clamp(5 - hdt::g_logLevel, 0, 5);
				const auto applyLevel = [](int lvl) {
					hdt::g_logLevel = 5 - std::clamp(lvl, 0, 5);
					spdlog::set_level(static_cast<spdlog::level::level_enum>(hdt::g_logLevel));
					spdlog::flush_on(static_cast<spdlog::level::level_enum>(hdt::g_logLevel));
				};
				if (ImGuiMCP::SliderInt(hid("Log level").c_str(), &level, 0, 5))
					applyLevel(level);
				if (ImGuiMCP::IsItemHovered(0))
					g_help = kLogHelp;
				tip(kLogHelp);
				if (ImGuiMCP::IsItemDeactivatedAfterEdit())
					hdt::saveUserSettings();
				if (rowReset("Log level", level != d.logLevel)) {
					applyLevel(d.logLevel);
					hdt::saveUserSettings();
				}
			}
			endRows();
		}

		helpLine();
	}

	void __stdcall RenderCommands()
	{
		header(false);  // the master switch now lives in the header pill; commands need no search box
		auto* player = static_cast<RE::TESObjectREFR*>(RE::PlayerCharacter::GetSingleton());

		section(fa::Terminal, "Console commands (output goes to the console / log)");
		if (ImGuiMCP::Button(tr("smp (basic info)")))
			hdt::RunSMPDebugCommand("", "", "", nullptr);
		ImGuiMCP::SameLine();
		if (ImGuiMCP::Button(tr("smp reset")))
			hdt::applyConfigReset();
		tip("Reload config and reset all physics systems.");
		ImGuiMCP::SameLine();
		if (ImGuiMCP::Button(tr("smp list")))
			hdt::RunSMPDebugCommand("list", "", "", nullptr);
		ImGuiMCP::SameLine();
		if (ImGuiMCP::Button(tr("smp detail")))
			hdt::RunSMPDebugCommand("detail", "", "", nullptr);
		if (ImGuiMCP::Button(tr("smp dumptree (player)")))
			hdt::RunSMPDebugCommand("dumptree", "", "", player);
		tip("Dump the player's 3D node tree to the log (needs log level 3+).");
		ImGuiMCP::SameLine();
		if (ImGuiMCP::Button(tr("smp QueryOverride")))
			hdt::RunSMPDebugCommand("QueryOverride", "", "", nullptr);

		section(fa::Clipboard, "Validator");
		if (ImGuiMCP::Button(tr("smp report")))
			hdt::RunSMPDebugCommand("report", "", "", nullptr);
		tip("Run the physics-asset validator in the background; writes a report file.");
		ImGuiMCP::SameLine();
		if (ImGuiMCP::Button(tr("smp report gear")))
			hdt::RunSMPDebugCommand("report", "gear", "", nullptr);
		tip("Validate currently equipped gear only.");
	}

	// ---- Presets ----------------------------------------------------------------------------------------

	struct PresetEntry
	{
		std::string name;
		std::filesystem::path path;
		GlobalConfig cfg;
	};

	std::vector<PresetEntry> g_presets;
	bool g_presetsScanned = false;

	// Read every *.json under configsPresets/ and parse it once into a cached struct, so the per-frame
	// "is this the active preset?" comparison is a cheap struct compare rather than re-reading files.
	void scanPresets()
	{
		g_presets.clear();
		namespace fs = std::filesystem;
		std::error_code ec;
		const fs::path dir = "data/skse/plugins/hdtSkinnedMeshConfigs/configsPresets";
		if (fs::exists(dir, ec)) {
			for (const auto& entry : fs::directory_iterator(dir, ec)) {
				if (entry.path().extension() == ".json") {
					std::string bytes = readFileBytes(entry.path());
					g_presets.push_back({ entry.path().stem().string(), entry.path(),
						hdt::parseConfigJson(bytes) });
				}
			}
		}
		g_presetsScanned = true;
	}

	// Compare only the physics fields: presets do not own mods-dir, the node-backup list, or the menu locale,
	// so blank those on both sides before comparing (otherwise a user's mods-dir or language would stop any
	// preset from ever highlighting).
	GlobalConfig physicsOnly(GlobalConfig c)
	{
		c.modsDir.clear();
		c.backupNodeByName.clear();
		c.locale.clear();
		return c;
	}

	void __stdcall RenderPresets()
	{
		header(false);
		if (!g_presetsScanned)
			scanPresets();

		if (ImGuiMCP::Button(tr("Refresh")))
			scanPresets();
		ImGuiMCP::SameLine();
		ImGuiMCP::TextDisabled(tr("(%d found)"), static_cast<int>(g_presets.size()));
		ImGuiMCP::Separator();

		const GlobalConfig live = physicsOnly(hdt::readConfig());

		if (g_presets.empty()) {
			ImGuiMCP::TextWrapped("%s", tr("No presets found in configsPresets/. Drop *.json preset files there."));
			return;
		}

		for (const auto& preset : g_presets) {
			const bool active = physicsOnly(preset.cfg) == live;
			if (active) {
				ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Button, kGreen);
				ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ButtonHovered, kGreen);
				ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ButtonActive, kGreen);
			}
			// The preset name is a file name (not translated); only the "(loaded)" marker is localized.
			std::string label = active ? (preset.name + "  " + tr("(loaded)")) : preset.name;
			if (ImGuiMCP::Button(label.c_str()) && !active) {
				// Apply the preset's physics fields, but keep the user's mods-dir, node backups, and language.
				GlobalConfig next = preset.cfg;
				const GlobalConfig cur = hdt::readConfig();
				next.modsDir = cur.modsDir;
				next.backupNodeByName = cur.backupNodeByName;
				next.locale = cur.locale;
				hdt::applyConfig(next);
				commitReset();
			}
			if (active)
				ImGuiMCP::PopStyleColor(3);
		}
	}

	// ---- About ------------------------------------------------------------------------------------------

	// One "icon + small button" link row; returns true when clicked so the caller can open the URL.
	bool linkButton(const char* label)
	{
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kAccent, "%s", fa::Link);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();
		const bool clicked = ImGuiMCP::SmallButton(tr(label));
		tip("Open in your browser");
		return clicked;
	}

	void __stdcall RenderAbout()
	{
		header(false);

		static ImGuiMCP::ImTextureID logo =
			SKSEMenuFramework::LoadTexture("Data/SKSE/Plugins/hdtSkinnedMeshConfigs/FSMP.dds");
		ImGuiMCP::Spacing();
		if (logo)
			ImGuiMCP::Image(logo, ImGuiMCP::ImVec2{ 96.0f * 768.0f / 444.0f, 96.0f });

		ImGuiMCP::Text("Faster HDT-SMP");
		ImGuiMCP::SameLine();
		ImGuiMCP::TextDisabled("v%s (%s)", Plugin::VERSION.string().c_str(), Plugin::AVX_VARIANT.data());
		ImGuiMCP::TextColored(kDim, "%s", tr("In-game configuration menu for Faster HDT-SMP."));

		ImGuiMCP::Spacing();
		ImGuiMCP::SeparatorText(tr("Language"));
		languageCombo();

		ImGuiMCP::Spacing();
		ImGuiMCP::Separator();
		if (linkButton("Documentation"))
			ShellExecuteA(nullptr, "open", "https://github.com/DaymareOn/hdtSMP64/wiki", nullptr, nullptr, SW_SHOWNORMAL);
		if (linkButton("Nexus page"))
			ShellExecuteA(nullptr, "open", "https://www.nexusmods.com/skyrimspecialedition/mods/57339", nullptr, nullptr, SW_SHOWNORMAL);
		if (linkButton("Report a bug"))
			ShellExecuteA(nullptr, "open", "https://github.com/DaymareOn/hdtSMP64/issues", nullptr, nullptr, SW_SHOWNORMAL);
	}
}

namespace hdt::FSMPMenu
{
	void Register()
	{
		hdt::loc::load();  // load the localized strings before we register any translated page name

		SKSEMenuFramework::SetSection("FSMP");
		SKSEMenuFramework::AddSectionItem(tr("Simplification"), RenderSimplification);
		SKSEMenuFramework::AddSectionItem(tr("Performance"), RenderPerformance);
		SKSEMenuFramework::AddSectionItem(tr("Wind"), RenderWind);
		SKSEMenuFramework::AddSectionItem(tr("Validation"), RenderValidation);
		SKSEMenuFramework::AddSectionItem(tr("Logs"), RenderLogs);
		SKSEMenuFramework::AddSectionItem(tr("Commands"), RenderCommands);
		SKSEMenuFramework::AddSectionItem(tr("Presets"), RenderPresets);
		SKSEMenuFramework::AddSectionItem(tr("About"), RenderAbout);
	}
}
