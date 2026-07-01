#include "UI/FSMPMenu.h"

#include "ActorManager.h"
#include "GlobalConfig.h"
#include "SMPDebug.h"
#include "UI/Localization.h"
#include "Validator/hdtAssetValidator.h"
#include "config.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cfloat>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <shlobj.h>    // IFileOpenDialog / SHCreateItemFromParsingName --- native folder picker
#include <shellapi.h>  // ShellExecuteA --- open the About-page links in the browser
#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "Ole32.lib")

#include "SKSEMenuFramework.h"

// In-DLL configuration UI. The SKSE Menu Framework owns the D3D11/ImGui hook and calls our render functions
// while its panel is open; we emit ImGui widgets (under the ImGuiMCP namespace, forwarded into
// SKSEMenuFramework.dll) bound to the live physics singletons. Edits persist to userConfigs.json and re-apply
// through the same path as `smp reset`.
//
// Navigation is a single section item ("FSMP") whose page renders an ImGui tab bar. The framework bakes a
// section item's label into its path at registration and offers no rename, so the OLD design (one section
// item per page) could not retranslate the left-panel labels when the language changed live. Driving the
// pages from our own tab bar lets every tab label go through tr() each frame, so switching language updates
// the whole menu immediately.
//
// Every user-visible string is wrapped in tr() (see UI/Localization): the translation for the current
// language, or the English source string when there is none. Apply policy is unchanged from `smp reset`:
// toggles/buttons commit on click; sliders preview live but only persist+reset on release; log level and
// mods-dir persist without a physics reset.

namespace
{
	using hdt::ActorManager;
	using hdt::GlobalConfig;
	using hdt::SkyrimPhysicsWorld;
	using hdt::loc::tr;

	// Font Awesome 6 (Free, Solid) glyphs as raw UTF-8 bytes, rendered only between FontAwesome::PushSolid()
	// and FontAwesome::Pop(). Purely decorative: a wrong glyph shows a placeholder box but never breaks a
	// control.
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
		constexpr const char* FolderOpen = "\xEF\x81\xBC";  // U+F07C folder-open
		constexpr const char* GaugeHigh = "\xEF\x98\xA5";  // U+F625 gauge-high
		constexpr const char* Rotate    = "\xEF\x80\xA1";  // U+F021 arrows-rotate (refresh)
		constexpr const char* TrashCan  = "\xEF\x8B\xAD";  // U+F2ED trash-can
	}

	// Palette (ImVec4 RGBA, 0..1).
	constexpr ImGuiMCP::ImVec4 kAccent{ 0.40f, 0.72f, 1.00f, 1.00f };  // section icons / titles / links
	constexpr ImGuiMCP::ImVec4 kDim{ 0.62f, 0.62f, 0.66f, 1.00f };     // help line / secondary text
	constexpr ImGuiMCP::ImVec4 kWarn{ 0.98f, 0.78f, 0.20f, 1.00f };    // inline warnings / "high" perf
	constexpr ImGuiMCP::ImVec4 kGreen{ 0.16f, 0.55f, 0.22f, 1.00f };   // physics ON pill bg
	constexpr ImGuiMCP::ImVec4 kGreenH{ 0.20f, 0.66f, 0.28f, 1.00f };
	constexpr ImGuiMCP::ImVec4 kRed{ 0.60f, 0.18f, 0.18f, 1.00f };     // physics OFF pill bg
	constexpr ImGuiMCP::ImVec4 kRedH{ 0.72f, 0.22f, 0.22f, 1.00f };
	constexpr ImGuiMCP::ImVec4 kOkTxt{ 0.40f, 0.85f, 0.45f, 1.00f };   // "light" perf value
	constexpr ImGuiMCP::ImVec4 kBadTxt{ 0.95f, 0.45f, 0.45f, 1.00f };  // "heavy" perf value

	// Per-frame UI state. chrome() resets these at the top of every menu render.
	char g_filter[96] = "";     // the search box contents
	bool g_filtering = false;   // true only while a settings tab shows a non-empty search box
	const char* g_help = nullptr;  // English help text of the row under the mouse this frame (for the help line)

	// The framework-managed overlay window shown during gameplay (see RenderPerfOverlay / the Metrics tab).
	SKSEMenuFramework::Model::WindowInterface* g_overlay = nullptr;

	// ---- Folder picker (Validation tab) -----------------------------------------------------------------
	// A native folder dialog must not run on the render thread (it is modal and would block drawing), so it
	// runs on a detached worker thread and hands its result back through these guarded fields, which the
	// render loop drains on a later frame.
	std::mutex g_folderMx;
	std::string g_pickedFolder;
	std::atomic<bool> g_folderReady{ false };
	std::atomic<bool> g_dialogBusy{ false };

	std::string narrow(const wchar_t* w)
	{
		if (!w)
			return {};
		const int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
		if (len <= 1)
			return {};
		std::string s(static_cast<size_t>(len - 1), '\0');
		WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), len, nullptr, nullptr);
		return s;
	}

	std::wstring widen(const std::string& s)
	{
		if (s.empty())
			return {};
		const int len = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
		if (len <= 0)
			return {};
		std::wstring w(static_cast<size_t>(len), L'\0');
		MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), len);
		return w;
	}

	// Best-effort guess of the mod manager's mods folder: our own DLL is loaded (outside the VFS, under MO2)
	// from <mods>\<FSMP mod>\SKSE\Plugins\hdtsmp64.dll, so the folder that contains the FSMP mod folder is four
	// levels up from the DLL file. Returns "" if that can't be resolved; the picker then opens wherever the OS
	// defaults. GetModuleHandleEx(FROM_ADDRESS) locates our module from the address of a function inside it.
	std::wstring deducedModsDir()
	{
		HMODULE h = nullptr;
		if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
				reinterpret_cast<LPCWSTR>(&deducedModsDir), &h) ||
			!h)
			return {};
		wchar_t path[MAX_PATH]{};
		if (!GetModuleFileNameW(h, path, MAX_PATH))
			return {};
		std::error_code ec;
		const std::filesystem::path dll(path);  // <mods>\<FSMP>\SKSE\Plugins\hdtsmp64.dll
		const std::filesystem::path mods = dll.parent_path().parent_path().parent_path().parent_path();
		if (!mods.empty() && std::filesystem::exists(mods, ec))
			return mods.wstring();
		return {};
	}

	// Open a native "pick a folder" dialog on a worker thread, seeded at initialDir. On OK the chosen path is
	// stashed in g_pickedFolder / g_folderReady for the render loop to apply. One dialog at a time.
	void openFolderDialogAsync(std::wstring initialDir)
	{
		if (g_dialogBusy.exchange(true))
			return;
		std::thread([initialDir = std::move(initialDir)]() {
			std::string result;
			if (SUCCEEDED(CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE))) {
				IFileOpenDialog* dlg = nullptr;
				if (SUCCEEDED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER,
						IID_PPV_ARGS(&dlg)))) {
					DWORD opts = 0;
					dlg->GetOptions(&opts);
					dlg->SetOptions(opts | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM | FOS_PATHMUSTEXIST);
					if (!initialDir.empty()) {
						IShellItem* si = nullptr;
						if (SUCCEEDED(SHCreateItemFromParsingName(initialDir.c_str(), nullptr, IID_PPV_ARGS(&si)))) {
							dlg->SetFolder(si);
							si->Release();
						}
					}
					if (SUCCEEDED(dlg->Show(nullptr))) {
						IShellItem* item = nullptr;
						if (SUCCEEDED(dlg->GetResult(&item))) {
							PWSTR pathw = nullptr;
							if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &pathw))) {
								result = narrow(pathw);
								CoTaskMemFree(pathw);
							}
							item->Release();
						}
					}
					dlg->Release();
				}
				CoUninitialize();
			}
			{
				std::lock_guard<std::mutex> lk(g_folderMx);
				g_pickedFolder = std::move(result);
				g_folderReady = true;
			}
			g_dialogBusy = false;
		}).detach();
	}

	// ---- Small helpers ----------------------------------------------------------------------------------

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

	// Case-insensitive ASCII substring test for the search box. Both sides are lower-cased first.
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

	// A settings row shows when there is no active filter, or the filter matches its label in either the
	// current language (what the user sees) or the English source (so a known name still works).
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

	// ---- Header / chrome --------------------------------------------------------------------------------

	// The master physics on/off state as a green/red pill button; clicking it toggles physics exactly like the
	// console `smp on` / `smp off`, so behaviour stays identical to the command.
	void masterPill()
	{
		const bool on = !SkyrimPhysicsWorld::get()->disabled;
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Button, on ? kGreen : kRed);
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ButtonHovered, on ? kGreenH : kRedH);
		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ButtonActive, on ? kGreenH : kRedH);
		if (ImGuiMCP::Button(on ? tr("Physics is ON") : tr("Physics is OFF")))
			hdt::RunSMPDebugCommand(on ? "off" : "on", "", "", nullptr);
		ImGuiMCP::PopStyleColor(3);
		tip("Turn the whole physics simulation on or off (same as 'smp on' / 'smp off').");
	}

	// A locale's native display name for the dropdown ("fr_fr" -> "Français (fr_fr)"). Regional codes fall
	// back to their base language's name, and the code is always appended so an entry stays identifiable even
	// if the menu font lacks that script (e.g. CJK). Display-only --- the available set comes from the folder.
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
	// table. Every tr() (including the tab labels) picks up the new language on the next frame.
	void setLocale(const std::string& code)
	{
		hdt::g_locale = code;
		hdt::saveUserSettings();
		hdt::loc::load();
	}

	// The Language picker: a globe icon plus a combo listing "Auto" and every installed locale by native name.
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

	// The status header drawn above the tab bar: FSMP logo, name + version, the master pill, and the language
	// picker. Also resets the per-frame help text and filter scope (a tab that has no search box stays
	// unfiltered).
	void chrome()
	{
		g_help = nullptr;
		g_filtering = false;

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
		ImGuiMCP::Separator();
	}

	// The search box for a settings tab. Sets g_filtering for the rest of this frame.
	void filterBox()
	{
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kDim, "%s", fa::Search);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();
		ImGuiMCP::SetNextItemWidth(-FLT_MIN);
		ImGuiMCP::InputTextWithHint("##filter", tr("Search settings"), g_filter, sizeof(g_filter));
		g_filtering = g_filter[0] != '\0';
	}

	// An accent-coloured icon + title + separator introducing a group of settings. Hidden while a filter is
	// active, so matching rows from any group show as one flat list.
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

	// The persistent help line at the bottom of a settings tab: the hovered setting's description, or a prompt
	// when nothing is hovered. Complements (does not replace) the per-control tooltips.
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
	// group on every tab lines up. Returns false if the table couldn't open (caller then skips its rows).
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

	// ---- Tab bodies -------------------------------------------------------------------------------------

	void SimplificationBody()
	{
		filterBox();
		auto* a = ActorManager::instance();
		auto* w = SkyrimPhysicsWorld::get();
		const GlobalConfig& d = hdt::shippedDefaults();

		section(fa::Scissors, "Disabling some physics");
		if (beginRows("simpl.disable")) {
			if (rowCheck("Disable hair physics when there's a wig",
					"Skip hair physics when an armor occupies the hair/longhair slot (a wig on top of the hair).",
					&a->m_disableSMPHairWhenWigEquipped, d.disableSMPHairWhenWigEquipped))
				commitReset();
			if (rowCheck("No physics for your character in 1st person view",
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
					"Physics is always calculated for NPCs closer than this (units), even off-screen.",
					&a->m_minCullingDistance, d.minCullingDistance, 0.0f, 10000.0f, "%.0f"))
				commitReset();
			if (rowFloat("Min screen size %",
					"Skip non-player NPCs smaller than this % of screen height. 0 disables the check.",
					&a->m_minScreenSizePercent, d.minScreenSizePercent, 0.0f, 100.0f, "%.1f"))
				commitReset();
			if (rowCheck("Skip dead actors",
					"Skip physics for dead non-player actors (corpses). The player is never affected.",
					&a->m_skipDeadActors, d.skipDeadActors))
				commitReset();
			endRows();
		}

		section(fa::Bolt, "Limiting active physics NPCs");
		if (beginRows("simpl.limit")) {
			if (rowInt("Maximum physics NPCs",
					"Upper bound on simultaneously simulated NPCs (including the player).",
					&a->m_maxActiveSkeletons, d.maximumActiveSkeletons, 0, 200))
				commitReset();
			if (rowCheck("Auto-adjust the max number of physics NPCs",
					"Dynamically reduce active NPCs to stay within the frame-time budget below.",
					&a->m_autoAdjustMaxSkeletons, d.autoAdjustMaxSkeletons))
				commitReset();
			ImGuiMCP::BeginDisabled(!a->m_autoAdjustMaxSkeletons);
			if (rowFloat("Frame-time budget (ms)",
					"How many ms/frame physics may spend before it starts dropping active NPCs.",
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

	void PerformanceBody()
	{
		filterBox();
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
			if (rowCheck("Reset physics on big unlimited turns",
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

	void WindBody()
	{
		filterBox();
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

	void ValidationBody()
	{
		const GlobalConfig& d = hdt::shippedDefaults();  // d.modsDir is empty ("scan the VFS")

		// The exact same English string is the label, the tooltip, and the help-line text, and it is the
		// localization key --- keep it in one place so all three (and the translation) stay in sync.
		static constexpr const char* kModsHelp =
			"Your mod manager's mods folder (MO2 mods/ or Vortex staging). When set, 'smp report' scans it\n"
			"natively instead of through the virtual file system, which is much faster on big load orders.\n"
			"Leave empty to scan data/ through the VFS.";

		static char modsBuf[1024] = "";
		static bool editing = false;

		// Drain a folder the picker thread may have produced since last frame.
		if (g_folderReady.exchange(false)) {
			std::string picked;
			{
				std::lock_guard<std::mutex> lk(g_folderMx);
				picked = g_pickedFolder;
			}
			if (!picked.empty()) {
				hdt::g_validationConfig.modsDir = picked;
				std::strncpy(modsBuf, picked.c_str(), sizeof(modsBuf) - 1);
				modsBuf[sizeof(modsBuf) - 1] = '\0';
				editing = false;
				hdt::saveUserSettings();
			}
		}

		section(fa::Clipboard, "Mods folder");
		if (!editing) {
			const auto& s = hdt::g_validationConfig.modsDir;
			std::strncpy(modsBuf, s.c_str(), sizeof(modsBuf) - 1);
			modsBuf[sizeof(modsBuf) - 1] = '\0';
		}

		ImGuiMCP::Text("%s", tr("Mods folder"));
		if (ImGuiMCP::IsItemHovered(0))
			g_help = kModsHelp;

		ImGuiMCP::SetNextItemWidth(-90.0f);
		ImGuiMCP::InputText("##modsdir", modsBuf, sizeof(modsBuf));
		if (ImGuiMCP::IsItemHovered(0))
			g_help = kModsHelp;
		tip(kModsHelp);
		editing = ImGuiMCP::IsItemActive();
		if (ImGuiMCP::IsItemDeactivatedAfterEdit()) {
			hdt::g_validationConfig.modsDir = modsBuf;
			hdt::saveUserSettings();  // no physics reset; mods-dir only affects validation
		}

		ImGuiMCP::SameLine();
		FontAwesome::PushSolid();
		const bool browse = ImGuiMCP::Button(fa::FolderOpen);
		FontAwesome::Pop();
		tip("Browse for your mods folder");
		if (browse) {
			// Seed the dialog at the current value if it exists, else at the deduced mods folder.
			std::error_code ec;
			std::wstring init;
			if (!hdt::g_validationConfig.modsDir.empty() &&
				std::filesystem::exists(hdt::g_validationConfig.modsDir, ec))
				init = widen(hdt::g_validationConfig.modsDir);
			else
				init = deducedModsDir();
			openFolderDialogAsync(std::move(init));
		}
		ImGuiMCP::SameLine();
		ImGuiMCP::BeginDisabled(hdt::g_validationConfig.modsDir == d.modsDir);
		FontAwesome::PushSolid();
		const bool clr = ImGuiMCP::Button(fa::Undo);
		FontAwesome::Pop();
		ImGuiMCP::EndDisabled();
		tip("Reset this setting to its default");
		if (clr) {
			hdt::g_validationConfig.modsDir = d.modsDir;
			modsBuf[0] = '\0';
			editing = false;
			hdt::saveUserSettings();
		}

		helpLine();
	}

	void LogsBody()
	{
		const GlobalConfig& d = hdt::shippedDefaults();
		static constexpr const char* kLogHelp = "0 = Fatal, 1 = Error, 2 = Warning, 3 = Message, 4 = Verbose, 5 = Debug.";

		section(fa::FileLines, "Logs");
		if (beginRows("logs")) {
			rowLabel("Log level", kLogHelp);
			// g_logLevel stores the inverted spdlog scale, so bind a local 0..5 value and apply it live on any
			// change (so a drag accumulates); persist on release only (logging needs no physics reset).
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
			endRows();
		}

		helpLine();
	}

	void CommandsBody()
	{
		auto* player = static_cast<RE::TESObjectREFR*>(RE::PlayerCharacter::GetSingleton());

		section(fa::Terminal, "Console commands (shown below, and in the console / log)");
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

		// The validator ("smp report") takes optional [gear] and [error] flags, in any combination. Two
		// checkboxes plus one Run button cover every variant (report / report gear / report error /
		// report gear error) instead of one button per combination.
		section(fa::Clipboard, "Validator");
		static bool gearOnly = false;
		static bool errorsOnly = false;
		ImGuiMCP::Checkbox(tr("Gear only"), &gearOnly);
		tip("Validate currently equipped gear only.");
		ImGuiMCP::SameLine();
		ImGuiMCP::Checkbox(tr("Errors only"), &errorsOnly);
		tip("Write an errors-only report (no warnings/info).");
		if (ImGuiMCP::Button(tr("Run report"))) {
			// RunSMPDebugCommand parses buffer2 then buffer3, each "gear" or "error"; pass the selected flags.
			const char* a2 = gearOnly ? "gear" : (errorsOnly ? "error" : "");
			const char* a3 = (gearOnly && errorsOnly) ? "error" : "";
			hdt::RunSMPDebugCommand("report", a2, a3, nullptr);
		}
		tip("Run the physics-asset validator in the background; writes a report file.");

		// The profiler ("smp profile") toggles capture and takes two frame counts.
		section(fa::GaugeHigh, "Profiler");
		static int sampleFrames = 240;
		static int printFrames = 240;
		ImGuiMCP::SetNextItemWidth(140.0f);
		ImGuiMCP::InputInt(tr("Sample frames"), &sampleFrames);
		ImGuiMCP::SetNextItemWidth(140.0f);
		ImGuiMCP::InputInt(tr("Print every N frames"), &printFrames);
		if (ImGuiMCP::Button(tr("Toggle physics profiler"))) {
			if (sampleFrames < 1)
				sampleFrames = 1;
			if (printFrames < 1)
				printFrames = 1;
			hdt::RunSMPDebugCommand("profile", std::to_string(sampleFrames).c_str(),
				std::to_string(printFrames).c_str(), nullptr);
		}
		tip("Toggle physics profiler capture on/off; results are written to hdtSMP64.log.");

		// The captured output, shown right here so it is visible without opening the console.
		ImGuiMCP::Spacing();
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kAccent, "%s", fa::Terminal);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();
		ImGuiMCP::TextColored(kAccent, "%s", tr("Output"));
		ImGuiMCP::SameLine();
		FontAwesome::PushSolid();
		const bool clear = ImGuiMCP::SmallButton(fa::TrashCan);
		FontAwesome::Pop();
		tip("Clear");
		if (clear)
			hdt::clearMenuConsole();
		ImGuiMCP::Separator();

		const std::vector<std::string> lines = hdt::menuConsoleSnapshot();
		if (ImGuiMCP::BeginChild("##smpout", ImGuiMCP::ImVec2{ 0.0f, -FLT_MIN },
				ImGuiMCP::ImGuiChildFlags_Border, ImGuiMCP::ImGuiWindowFlags_HorizontalScrollbar)) {
			for (const auto& l : lines)
				ImGuiMCP::TextUnformatted(l.c_str());
			// Keep the newest line in view when output arrives.
			if (ImGuiMCP::GetScrollY() >= ImGuiMCP::GetScrollMaxY() - 1.0f)
				ImGuiMCP::SetScrollHereY(1.0f);
		}
		ImGuiMCP::EndChild();
	}

	// ---- Metrics (live perf) + overlay ------------------------------------------------------------------

	// Colour a millisecond figure by how heavy it is (wiki thresholds: under ~3ms light, ~3-8ms high, more
	// than that you should trim your physics load).
	ImGuiMCP::ImVec4 msColor(float ms)
	{
		if (ms < 3.0f)
			return kOkTxt;
		if (ms < 8.0f)
			return kWarn;
		return kBadTxt;
	}

	// One "label : value ms" row in the Metrics table, the value tinted by msColor.
	void metricRow(const char* label, float ms, bool color = true)
	{
		ImGuiMCP::TableNextRow();
		ImGuiMCP::TableNextColumn();
		ImGuiMCP::Text("%s", tr(label));
		ImGuiMCP::TableNextColumn();
		if (color)
			ImGuiMCP::TextColored(msColor(ms), "%.2f ms", ms);
		else
			ImGuiMCP::Text("%.2f ms", ms);
	}

	void MetricsBody()
	{
		auto* w = SkyrimPhysicsWorld::get();
		auto* a = ActorManager::instance();

		const float impact = w->m_averageSMPProcessingTimeInMainLoop;
		const float bg = w->m_2ndStepAverageProcessingTime;
		const float hidden = std::max(0.0f, bg - w->m_avgWaitMs);
		const float total = w->m_avgSetupMs + bg + w->m_avgWriteMs;

		ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Text, kDim);
		ImGuiMCP::TextWrapped("%s", tr("Live physics performance data --- the same numbers written to hdtSMP64.log."));
		ImGuiMCP::PopStyleColor();
		ImGuiMCP::Spacing();

		if (w->disabled) {
			ImGuiMCP::TextColored(kWarn, "%s", tr("Physics simulation is off."));
			ImGuiMCP::Spacing();
		}

		section(fa::GaugeHigh, "Frame-time cost");
		if (ImGuiMCP::BeginTable("metrics.time", 2, ImGuiMCP::ImGuiTableFlags_PadOuterX)) {
			ImGuiMCP::TableSetupColumn("l", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, 260.0f);
			ImGuiMCP::TableSetupColumn("v", ImGuiMCP::ImGuiTableColumnFlags_WidthStretch);
			metricRow("Frame-time impact", impact);
			metricRow("Setup", w->m_avgSetupMs, false);
			metricRow("Wait", w->m_avgWaitMs, false);
			metricRow("Apply", w->m_avgWriteMs, false);
			metricRow("Background calc", bg, false);
			metricRow("Hidden (overlapped)", hidden, false);
			metricRow("Total CPU work", total, false);
			ImGuiMCP::EndTable();
		}

		section(fa::Bolt, "Load");
		if (ImGuiMCP::BeginTable("metrics.load", 2, ImGuiMCP::ImGuiTableFlags_PadOuterX)) {
			ImGuiMCP::TableSetupColumn("l", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, 260.0f);
			ImGuiMCP::TableSetupColumn("v", ImGuiMCP::ImGuiTableColumnFlags_WidthStretch);
			ImGuiMCP::TableNextRow();
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%s", tr("Active physics NPCs"));
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%d / %d", a->activeSkeletons, a->m_maxActiveSkeletons);
			ImGuiMCP::TableNextRow();
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%s", tr("Frame-time budget (ms)"));
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%.1f ms", w->m_budgetMs);
			ImGuiMCP::EndTable();
		}

		ImGuiMCP::Spacing();
		ImGuiMCP::Separator();
		if (g_overlay) {
			bool overlayOn = g_overlay->IsOpen.load();
			if (ImGuiMCP::Checkbox(tr("Show overlay while playing"), &overlayOn))
				g_overlay->IsOpen = overlayOn;
			tip("Show a small always-on-top readout of the frame-time cost while you play.");
		}
	}

	// The compact gameplay overlay (a framework-managed, non-blocking window). Draws the two figures that
	// matter most at a glance; the framework provides the window frame around this content.
	void __stdcall RenderPerfOverlay()
	{
		auto* w = SkyrimPhysicsWorld::get();
		auto* a = ActorManager::instance();
		ImGuiMCP::Text("FSMP");
		ImGuiMCP::SameLine();
		ImGuiMCP::TextColored(msColor(w->m_averageSMPProcessingTimeInMainLoop), "%.2f ms",
			w->m_averageSMPProcessingTimeInMainLoop);
		ImGuiMCP::Text("%s: %d / %d", tr("Active physics NPCs"), a->activeSkeletons, a->m_maxActiveSkeletons);
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

	void PresetsBody()
	{
		if (!g_presetsScanned)
			scanPresets();

		FontAwesome::PushSolid();
		const bool refresh = ImGuiMCP::Button(fa::Rotate);
		FontAwesome::Pop();
		tip("Refresh");
		if (refresh)
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

	void AboutBody()
	{
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

	// ---- Section-item render functions ------------------------------------------------------------------
	// One left-pane item per page. Each draws the shared status header (chrome) then its body. The page
	// content retranslates live when the language changes; the left-pane item labels are fixed at
	// registration (the framework has no rename), so they re-translate on the next game launch.
	void __stdcall RenderSimplification() { chrome(); SimplificationBody(); }
	void __stdcall RenderPerformance()    { chrome(); PerformanceBody(); }
	void __stdcall RenderWind()           { chrome(); WindBody(); }
	void __stdcall RenderValidation()     { chrome(); ValidationBody(); }
	void __stdcall RenderLogs()           { chrome(); LogsBody(); }
	void __stdcall RenderCommands()       { chrome(); CommandsBody(); }
	void __stdcall RenderPresets()        { chrome(); PresetsBody(); }
	void __stdcall RenderMetrics()        { chrome(); MetricsBody(); }
	void __stdcall RenderAbout()          { chrome(); AboutBody(); }
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
		SKSEMenuFramework::AddSectionItem(tr("Metrics"), RenderMetrics);
		SKSEMenuFramework::AddSectionItem(tr("About"), RenderAbout);

		// A non-blocking overlay window for the Metrics tab's "show while playing" toggle; starts hidden.
		g_overlay = SKSEMenuFramework::AddWindow(RenderPerfOverlay, false);
		if (g_overlay)
			g_overlay->IsOpen = false;
	}
}
