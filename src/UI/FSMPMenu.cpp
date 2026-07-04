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
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <shellapi.h>  // ShellExecuteA --- open web links / Explorer
#include <shlobj.h>    // IFileOpenDialog / SHCreateItemFromParsingName --- native folder picker
#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "Ole32.lib")

#include <miniz/miniz.h>  // zip the bug-report bundle (log + report + crashlog)

#include "SKSEMenuFramework.h"

// In-DLL configuration UI. The SKSE Menu Framework owns the D3D11/ImGui hook and calls our render functions
// while its panel is open; we emit ImGui widgets (under the ImGuiMCP namespace, forwarded into
// SKSEMenuFramework.dll) bound to the live physics singletons. Edits persist to userConfigs.json and re-apply
// through the same path as `smp reset`.
//
// Navigation is one left-pane section item per page under the "FSMP" section; Home is the first item (the
// landing page): big logo, version, description, web links. The framework bakes an item's label into
// its path at registration and offers no rename, so the left-pane labels re-translate on the next game launch;
// everything inside a page goes through tr() each frame and switches language live.
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
		constexpr const char* Scissors = "\xEF\x83\x84";    // U+F0C4 scissors
		constexpr const char* Bolt = "\xEF\x83\xA7";        // U+F0E7 bolt
		constexpr const char* Wind = "\xEF\x9C\xAE";        // U+F72E wind
		constexpr const char* Clipboard = "\xEF\x91\xAC";   // U+F46C clipboard-check
		constexpr const char* FileLines = "\xEF\x85\x9C";   // U+F15C file-lines
		constexpr const char* Terminal = "\xEF\x84\xA0";    // U+F120 terminal
		constexpr const char* Sliders = "\xEF\x87\x9E";     // U+F1DE sliders
		constexpr const char* Info = "\xEF\x81\x9A";        // U+F05A circle-info
		constexpr const char* Undo = "\xEF\x83\xA2";        // U+F0E2 arrow-rotate-left
		constexpr const char* Language = "\xEF\x86\xAB";    // U+F1AB language
		constexpr const char* Link = "\xEF\x83\x81";        // U+F0C1 link
		constexpr const char* Warning = "\xEF\x81\xB1";     // U+F071 triangle-exclamation
		constexpr const char* Search = "\xEF\x80\x82";      // U+F002 magnifying-glass
		constexpr const char* FolderOpen = "\xEF\x81\xBC";  // U+F07C folder-open
		constexpr const char* GaugeHigh = "\xEF\x98\xA5";   // U+F625 gauge-high
		constexpr const char* Rotate = "\xEF\x80\xA1";      // U+F021 arrows-rotate (refresh)
		constexpr const char* TrashCan = "\xEF\x8B\xAD";    // U+F2ED trash-can
		constexpr const char* FileZip = "\xEF\x87\x86";     // U+F1C6 file-zipper
	}

	// Palette (ImVec4 RGBA, 0..1).
	constexpr ImGuiMCP::ImVec4 kAccent{ 0.40f, 0.72f, 1.00f, 1.00f };  // section icons / titles / links
	constexpr ImGuiMCP::ImVec4 kDim{ 0.62f, 0.62f, 0.66f, 1.00f };     // help line / secondary text
	constexpr ImGuiMCP::ImVec4 kWarn{ 0.98f, 0.78f, 0.20f, 1.00f };    // inline warnings / "high" perf
	constexpr ImGuiMCP::ImVec4 kGreen{ 0.16f, 0.55f, 0.22f, 1.00f };   // physics ON pill bg
	constexpr ImGuiMCP::ImVec4 kGreenH{ 0.20f, 0.66f, 0.28f, 1.00f };
	constexpr ImGuiMCP::ImVec4 kRed{ 0.60f, 0.18f, 0.18f, 1.00f };  // physics OFF pill bg
	constexpr ImGuiMCP::ImVec4 kRedH{ 0.72f, 0.22f, 0.22f, 1.00f };
	constexpr ImGuiMCP::ImVec4 kOkTxt{ 0.40f, 0.85f, 0.45f, 1.00f };   // "light" perf value
	constexpr ImGuiMCP::ImVec4 kBadTxt{ 0.95f, 0.45f, 0.45f, 1.00f };  // "heavy" perf value

	// Per-frame UI state. chrome() resets these at the top of every menu render.
	char g_filter[96] = "";    // the search box contents
	bool g_filtering = false;  // true only while a settings tab shows a non-empty search box

	// Label-column sizing: every settings/measures row measures its translated label during the frame, and
	// chrome() applies the widest one on the next frame. Only one page renders at a time, so one accumulator
	// keeps every table on the current screen aligned --- whatever the language --- and no label is ever
	// covered by its control.
	float g_labelWidthAccum = 0.0f;
	float g_labelWidthApplied = 220.0f;

	// Set when the user switches language in-game: the framework cannot rename the registered left-panel
	// items, so those stay in the old language until the game restarts; chrome() shows a notice explaining it.
	bool g_localeChangedLive = false;

	// The framework-managed overlay window shown during gameplay (see RenderPerfOverlay / the Measures page).
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

	// Two-tone slider track: overlay a translucent accent fill from the track's left edge up to the grab, so
	// the "how far along am I" part reads at a glance (ImGui's stock slider colours the whole track the same
	// on both sides of the grab). Drawn over the last submitted item; frac is (value-min)/(max-min).
	void sliderFill(float frac)
	{
		frac = std::clamp(frac, 0.0f, 1.0f);
		ImGuiMCP::ImVec2 mn{}, mx{};
		ImGuiMCP::GetItemRectMin(&mn);
		ImGuiMCP::GetItemRectMax(&mx);
		ImGuiMCP::ImVec4 c = kAccent;
		c.w = 0.25f;  // translucent so the grab and the value text stay readable underneath
		ImGuiMCP::ImDrawListManager::AddRectFilled(ImGuiMCP::GetWindowDrawList(), mn,
			ImGuiMCP::ImVec2{ mn.x + (mx.x - mn.x) * frac, mx.y }, ImGuiMCP::GetColorU32(c), 3.0f, 0);
	}

	std::string readFileBytes(const std::filesystem::path& path)
	{
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return {};
		return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
	}

	// Read at most the last maxBytes of the FSMP log file (SKSE's log dir / hdtSMP64.log). Reading only the
	// tail keeps this bounded even when a validator report has bloated the log to tens of MB --- we never load
	// the whole file. Returns "" if the file is absent (e.g. a debug build that logs to the debugger). When we
	// start mid-file the partial first line is dropped so the view begins on a clean line.
	std::string readLogTail(std::uintmax_t maxBytes)
	{
		std::error_code ec;
		const auto dir = logger::log_directory();
		if (!dir)
			return {};
		const std::filesystem::path path = *dir / (std::string(Plugin::NAME) + ".log");
		const std::uintmax_t size = std::filesystem::file_size(path, ec);
		if (ec)
			return {};
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return {};
		const std::uintmax_t start = size > maxBytes ? size - maxBytes : 0;
		in.seekg(static_cast<std::streamoff>(start), std::ios::beg);
		std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		if (start > 0) {
			const auto nl = bytes.find('\n');
			if (nl != std::string::npos)
				bytes.erase(0, nl + 1);
		}
		return bytes;
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
	// table. Every tr() picks up the new language on the next frame; only the registered left-panel item
	// names cannot follow until a restart (see g_localeChangedLive).
	void setLocale(const std::string& code)
	{
		hdt::g_locale = code;
		hdt::saveUserSettings();
		hdt::loc::load();
		g_localeChangedLive = true;
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

	// The status header drawn atop every page: the master pill and the language picker (the brand --- logo,
	// name, version --- lives on the Home page only). Applies last frame's widest measured label to the label
	// columns and starts a fresh measurement, and resets the filter scope (a page that has no search box
	// stays unfiltered).
	void fontSizeButtons(const char* id, float* scale);  // defined with the Output panel below; used by chrome

	void chrome()
	{
		g_filtering = false;
		g_labelWidthApplied = std::clamp(g_labelWidthAccum, 150.0f, 480.0f);
		g_labelWidthAccum = 0.0f;

		masterPill();
		ImGuiMCP::SameLine();
		languageCombo();
		if (g_overlay) {
			// The overlay's controls live in the header (like the language picker) so the gameplay overlay can
			// be toggled and sized from any page, not only from Measures.
			ImGuiMCP::SameLine();
			bool overlayOn = g_overlay->IsOpen.load();
			if (ImGuiMCP::Checkbox(tr("Show overlay while playing"), &overlayOn))
				g_overlay->IsOpen = overlayOn;
			tip("Show a small always-on-top readout of the frame-time cost while you play.");
			ImGuiMCP::SameLine();
			fontSizeButtons("ovlfont", &hdt::g_overlayFontScale);  // sizes the overlay's text
		}
		if (g_localeChangedLive) {
			ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Text, kDim);
			ImGuiMCP::TextWrapped("%s", tr("The page names on the left switch language after a game restart."));
			ImGuiMCP::PopStyleColor();
		}
		ImGuiMCP::Separator();
	}

	// The search box for a settings tab. Sets g_filtering for the rest of this frame. The leading empty line
	// gives every page one blank row after the header (see the page bodies), for readability.
	void filterBox()
	{
		ImGuiMCP::NewLine();
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
		ImGuiMCP::NewLine();  // a full empty line between sections, for readability
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

	// ---- Output panel (shared) ----------------------------------------------------------------------------

	// A-/A+ small buttons stepping a font scale (hdt::g_outputFontScale or hdt::g_overlayFontScale), persisted
	// to userConfigs.json on click like any other setting. id keeps two pairs in the same window distinct.
	void fontSizeButtons(const char* id, float* scale)
	{
		ImGuiMCP::PushID(id);
		bool changed = false;
		if (ImGuiMCP::SmallButton("A-")) {
			*scale = std::max(GlobalConfig::minFontScale, *scale - 0.1f);
			changed = true;
		}
		tip("Decrease font size");
		ImGuiMCP::SameLine();
		if (ImGuiMCP::SmallButton("A+")) {
			*scale = std::min(GlobalConfig::maxFontScale, *scale + 0.1f);
			changed = true;
		}
		tip("Increase font size");
		ImGuiMCP::PopID();
		if (changed)
			hdt::saveUserSettings();  // no physics reset; a font scale only affects the menu
	}

	// The captured smp-command output, shown on every tab that can run a command (Commands, Validation,
	// Measures) so the result appears right where the command was launched. Header row: title, font-size
	// buttons, clear. The view fills the tab's remaining height and follows the newest line.
	void outputPanel()
	{
		ImGuiMCP::NewLine();  // an empty line before the output, like between sections
		FontAwesome::PushSolid();
		ImGuiMCP::TextColored(kAccent, "%s", fa::Terminal);
		FontAwesome::Pop();
		ImGuiMCP::SameLine();
		ImGuiMCP::TextColored(kAccent, "%s", tr("Output"));
		ImGuiMCP::SameLine();
		fontSizeButtons("outfont", &hdt::g_outputFontScale);
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
			ImGuiMCP::SetWindowFontScale(hdt::g_outputFontScale);
			// A list clipper submits only the visible rows, so a whole report tail (thousands of lines in the
			// ring buffer) costs the same to draw as a dozen.
			ImGuiMCP::ImGuiListClipper* clip = ImGuiMCP::ImGuiListClipperManager::Create();
			ImGuiMCP::ImGuiListClipperManager::Begin(clip, static_cast<int>(lines.size()), -1.0f);
			while (ImGuiMCP::ImGuiListClipperManager::Step(clip))
				for (int i = clip->DisplayStart; i < clip->DisplayEnd; ++i)
					ImGuiMCP::TextUnformatted(lines[static_cast<size_t>(i)].c_str());
			ImGuiMCP::ImGuiListClipperManager::Destroy(clip);
			// Keep following the newest line while the view is pinned to the bottom.
			if (ImGuiMCP::GetScrollY() >= ImGuiMCP::GetScrollMaxY() - 1.0f)
				ImGuiMCP::SetScrollY(ImGuiMCP::GetScrollMaxY());
		}
		ImGuiMCP::EndChild();
	}

	// ---- Settings-row framework -------------------------------------------------------------------------

	// Open a three-column settings table: [label | control | reset]. Fixed label and reset widths so every
	// group on every tab lines up. Returns false if the table couldn't open (caller then skips its rows).
	bool beginRows(const char* id)
	{
		if (!ImGuiMCP::BeginTable(id, 3, ImGuiMCP::ImGuiTableFlags_PadOuterX))
			return false;
		// The label column follows the widest translated label on the current page (measured last frame), so
		// no language ever gets its labels covered by the controls, and every table on the page lines up.
		ImGuiMCP::TableSetupColumn("label", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, g_labelWidthApplied + 12.0f);
		ImGuiMCP::TableSetupColumn("control", ImGuiMCP::ImGuiTableColumnFlags_WidthStretch);
		// The reset column is the last one, flush against the table's right edge; make it wide enough for the
		// icon button plus the cell padding, otherwise the button's right half is clipped at the edge.
		ImGuiMCP::TableSetupColumn("reset", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, 48.0f);
		return true;
	}
	void endRows()
	{
		ImGuiMCP::EndTable();
	}

	// Record a translated label's width so chrome() can size the label columns to the widest one next frame.
	void measureLabel(const char* translated)
	{
		ImGuiMCP::ImVec2 sz{};
		ImGuiMCP::CalcTextSize(&sz, translated, nullptr, false, -1.0f);
		g_labelWidthAccum = std::max(g_labelWidthAccum, sz.x);
	}

	// Start a row: draw the label (col 0) --- with the same tooltip as its control, so hovering either
	// explains the setting --- and leave the cursor in the control cell (col 1) with the item width stretched
	// to fill it. Shared by every typed row and the custom rows.
	void rowLabel(const char* label, const char* help)
	{
		ImGuiMCP::TableNextRow();
		ImGuiMCP::TableNextColumn();
		ImGuiMCP::Text("%s", tr(label));
		tip(help);
		measureLabel(tr(label));
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
		sliderFill((*v - lo) / (hi - lo));
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
		sliderFill(static_cast<float>(*v - lo) / static_cast<float>(hi - lo));
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
			if (rowCheck("Hide SMP hair when invisible",
					"Hide physics hair while an invisibility effect is active, so it doesn't stay visible on an invisible actor.",
					&a->m_hideSMPHairWhenInvisible, d.hideSMPHairWhenInvisible))
				commitReset();
			if (rowCheck("No physics for your character in 1st person view",
					"Skip the player's physics while in first-person view to save performance.",
					&a->m_disable1stPersonViewPhysics, d.disable1stPersonViewPhysics))
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
		tip(kModsHelp);
		measureLabel(tr("Mods folder"));

		// Reserve room on the right for the two icon buttons (browse + reset) so neither is clipped.
		ImGuiMCP::SetNextItemWidth(-130.0f);
		ImGuiMCP::InputText("##modsdir", modsBuf, sizeof(modsBuf));
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

		// The validator ("smp report") takes optional [gear] and [warnings] flags, in any combination. The
		// report is errors-only by default; "warnings" opts into the full report. Two checkboxes plus one Run
		// button cover every variant (report / report gear / report warnings / report gear warnings).
		section(fa::Clipboard, "Validator");
		static bool gearOnly = false;
		static bool includeWarnings = false;
		ImGuiMCP::Checkbox(tr("Gear only"), &gearOnly);
		tip("Validate currently equipped gear only.");
		ImGuiMCP::SameLine();
		ImGuiMCP::Checkbox(tr("Include warnings"), &includeWarnings);
		tip("Include warnings and info in the report (otherwise errors only).");
		if (ImGuiMCP::Button(tr("Run report"))) {
			// RunSMPDebugCommand parses buffer2 then buffer3, each "gear" or "warnings"; pass the selected flags.
			const char* a2 = gearOnly ? "gear" : (includeWarnings ? "warnings" : "");
			const char* a3 = (gearOnly && includeWarnings) ? "warnings" : "";
			hdt::RunSMPDebugCommand("report", a2, a3, nullptr);
		}
		tip("Run the physics-asset validator in the background; writes a report file.");

		outputPanel();
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
			sliderFill(static_cast<float>(level) / 5.0f);
			tip(kLogHelp);
			if (ImGuiMCP::IsItemDeactivatedAfterEdit())
				hdt::saveUserSettings();
			if (rowReset("Log level", level != d.logLevel)) {
				applyLevel(d.logLevel);
				hdt::saveUserSettings();
			}
			endRows();
		}

		// A live view of the FSMP log's tail, so the log (including [SMP Metrics] and profiler results) can be
		// read without leaving the game. Only the last chunk is loaded, so a report-bloated log stays cheap.
		section(fa::FileLines, "Log file");
		static std::string logText;
		static bool autoRefresh = true;
		static int frame = 0;
		bool doRead = ImGuiMCP::Button(tr("Refresh"));
		ImGuiMCP::SameLine();
		ImGuiMCP::Checkbox(tr("Auto-refresh"), &autoRefresh);
		ImGuiMCP::SameLine();
		ImGuiMCP::TextDisabled(tr("(last %d KB)"), 256);
		ImGuiMCP::SameLine();
		fontSizeButtons("logfont", &hdt::g_outputFontScale);
		if (autoRefresh && (frame++ % 30 == 0))  // ~twice a second at 60 fps, and on the first frame
			doRead = true;
		if (doRead)
			logText = readLogTail(256u * 1024u);
		if (ImGuiMCP::BeginChild("##logview", ImGuiMCP::ImVec2{ 0.0f, -FLT_MIN },
				ImGuiMCP::ImGuiChildFlags_Border, ImGuiMCP::ImGuiWindowFlags_HorizontalScrollbar)) {
			ImGuiMCP::SetWindowFontScale(hdt::g_outputFontScale);
			ImGuiMCP::TextUnformatted(logText.c_str());
			// Follow the tail while auto-refreshing, but only if the user is already near the bottom.
			if (autoRefresh && ImGuiMCP::GetScrollY() >= ImGuiMCP::GetScrollMaxY() - 4.0f)
				ImGuiMCP::SetScrollHereY(1.0f);
		}
		ImGuiMCP::EndChild();
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

		outputPanel();
	}

	// ---- Measures (live perf) + overlay ------------------------------------------------------------------

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

	// One "label : value ms" row in the Measures table, the value tinted by msColor.
	void metricRow(const char* label, float ms, bool color = true)
	{
		ImGuiMCP::TableNextRow();
		ImGuiMCP::TableNextColumn();
		ImGuiMCP::Text("%s", tr(label));
		measureLabel(tr(label));
		ImGuiMCP::TableNextColumn();
		if (color)
			ImGuiMCP::TextColored(msColor(ms), "%.2f ms", ms);
		else
			ImGuiMCP::Text("%.2f ms", ms);
	}

	void MeasuresBody()
	{
		auto* w = SkyrimPhysicsWorld::get();
		auto* a = ActorManager::instance();

		const float impact = w->m_averageSMPProcessingTimeInMainLoop;
		const float bg = w->m_2ndStepAverageProcessingTime;
		const float hidden = std::max(0.0f, bg - w->m_avgWaitMs);
		const float total = w->m_avgSetupMs + bg + w->m_avgWriteMs;

		ImGuiMCP::NewLine();  // one blank row after the header, like every page
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
			ImGuiMCP::TableSetupColumn("l", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, g_labelWidthApplied + 12.0f);
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
			ImGuiMCP::TableSetupColumn("l", ImGuiMCP::ImGuiTableColumnFlags_WidthFixed, g_labelWidthApplied + 12.0f);
			ImGuiMCP::TableSetupColumn("v", ImGuiMCP::ImGuiTableColumnFlags_WidthStretch);
			ImGuiMCP::TableNextRow();
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%s", tr("Active physics NPCs"));
			measureLabel(tr("Active physics NPCs"));
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%d / %d", a->activeSkeletons, a->m_maxActiveSkeletons);
			ImGuiMCP::TableNextRow();
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%s", tr("Frame-time budget (ms)"));
			measureLabel(tr("Frame-time budget (ms)"));
			ImGuiMCP::TableNextColumn();
			ImGuiMCP::Text("%.1f ms", w->m_budgetMs);
			ImGuiMCP::EndTable();
		}

		// The profiler ("smp profile") toggles capture and takes two frame counts. It lives here with the other
		// measurements; its output shows below and its periodic results go to the log (see the Logs tab).
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

		outputPanel();
	}

	// The compact gameplay overlay. The framework invokes this callback without a surrounding Begin() ---
	// content would land in ImGui's fallback "Debug" window --- so open our own window: titled "FSMP",
	// auto-resizing to hug its content exactly (which also follows the font scale), never stealing focus from
	// gameplay. The title-bar close button syncs back to the Measures-page toggle.
	void __stdcall RenderPerfOverlay()
	{
		auto* w = SkyrimPhysicsWorld::get();
		auto* a = ActorManager::instance();
		bool open = true;
		if (ImGuiMCP::Begin("FSMP", &open,
				ImGuiMCP::ImGuiWindowFlags_AlwaysAutoResize | ImGuiMCP::ImGuiWindowFlags_NoCollapse |
					ImGuiMCP::ImGuiWindowFlags_NoFocusOnAppearing | ImGuiMCP::ImGuiWindowFlags_NoNav)) {
			ImGuiMCP::SetWindowFontScale(hdt::g_overlayFontScale);  // sized by the A-/A+ next to the overlay toggle
			ImGuiMCP::TextColored(msColor(w->m_averageSMPProcessingTimeInMainLoop), "%.2f ms",
				w->m_averageSMPProcessingTimeInMainLoop);
			ImGuiMCP::Text("%s: %d / %d", tr("Active physics NPCs"), a->activeSkeletons, a->m_maxActiveSkeletons);
		}
		ImGuiMCP::End();
		if (!open && g_overlay)
			g_overlay->IsOpen = false;
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

	// Compare only the physics fields: presets do not own mods-dir, the node-backup list, or the UI settings
	// (locale, font scales), so neutralise those on both sides before comparing (otherwise a user's mods-dir
	// or language would stop any preset from ever highlighting).
	GlobalConfig physicsOnly(GlobalConfig c)
	{
		const GlobalConfig def;
		c.modsDir.clear();
		c.backupNodeByName.clear();
		c.locale.clear();
		c.outputFontScale = def.outputFontScale;
		c.overlayFontScale = def.overlayFontScale;
		return c;
	}

	// One "icon + button" web link; opens in the user's browser when clicked.
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

	// Emit text horizontally centred in the current content region.
	void centeredText(const ImGuiMCP::ImVec4& color, const char* text)
	{
		ImGuiMCP::ImVec2 avail{}, sz{};
		ImGuiMCP::GetContentRegionAvail(&avail);
		ImGuiMCP::CalcTextSize(&sz, text, nullptr, false, -1.0f);
		ImGuiMCP::SetCursorPosX(ImGuiMCP::GetCursorPosX() + std::max(0.0f, (avail.x - sz.x) * 0.5f));
		ImGuiMCP::TextColored(color, "%s", text);
	}

	// The Home landing page (the one shown when clicking "FSMP" in the framework's menu): the brand front and
	// centre --- a big logo, the name + version, the one-line description --- then the web links.
	// ---- Bug-report bundle (Home page) ------------------------------------------------------------------
	// A one-click "zip up what a bug report needs" helper: always the current hdtSMP64.log, optionally a fresh
	// gear/errors-only validation report and the most recent crash log. The zip is written into the SKSE log
	// folder and can be revealed (preselected) in Explorer. Building runs on a worker thread --- the report
	// step calls the validator synchronously and can take seconds --- so the menu never blocks.

	std::atomic<int> g_bundleState{ 0 };  // 0 idle, 1 building, 2 done, 3 failed
	std::mutex g_bundleFileMx;
	std::string g_bundleFile;  // filename of the last successfully built zip (guarded)

	// The SKSE log directory (hdtSMP64.log and the bundles live here), or empty if it can't be resolved.
	std::filesystem::path logDir()
	{
		const auto d = logger::log_directory();
		return d ? *d : std::filesystem::path{};
	}

	// True only for a CrashLoggerSSE crash file: crash-YYYY-MM-DD-HH-MM-SS.log --- a "crash-" prefix, then only
	// digits and dashes (the date and time), then ".log". Deliberately strict, so files that merely contain
	// "crash" (or other loggers' formats, e.g. .NET Script Framework's crash_*.txt) are never picked up.
	bool isCrashLogName(std::string name)
	{
		std::transform(name.begin(), name.end(), name.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		const std::string pre = "crash-";
		const std::string suf = ".log";
		if (name.size() <= pre.size() + suf.size() || name.rfind(pre, 0) != 0 ||
			name.compare(name.size() - suf.size(), suf.size(), suf) != 0)
			return false;
		for (size_t i = pre.size(); i < name.size() - suf.size(); ++i)
			if (!(name[i] >= '0' && name[i] <= '9') && name[i] != '-')
				return false;
		return true;
	}

	// Newest crash-YYYY-...-SS.log file directly under `dir`, or empty.
	std::filesystem::path newestCrashIn(const std::filesystem::path& dir)
	{
		std::error_code ec;
		if (!std::filesystem::exists(dir, ec))
			return {};
		std::filesystem::path best;
		std::filesystem::file_time_type bestT{};
		for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
			if (!e.is_regular_file(ec) || !isCrashLogName(e.path().filename().string()))
				continue;
			const auto t = e.last_write_time(ec);
			if (best.empty() || t > bestT) {
				best = e.path();
				bestT = t;
			}
		}
		return best;
	}

	// The most recent CrashLoggerSSE crash file (crash-<date>.log), searched where CrashLoggerSSE writes: the
	// SKSE folder and its Crashlogs subfolder. Empty if none.
	std::filesystem::path newestCrashLog()
	{
		const auto ld = logDir();
		if (ld.empty())
			return {};
		const std::filesystem::path candidates[] = {
			ld,
			ld / "Crashlogs",
		};
		std::filesystem::path best;
		std::filesystem::file_time_type bestT{};
		for (const auto& dir : candidates) {
			const auto c = newestCrashIn(dir);
			if (c.empty())
				continue;
			std::error_code ec;
			const auto t = std::filesystem::last_write_time(c, ec);
			if (best.empty() || t > bestT) {
				best = c;
				bestT = t;
			}
		}
		return best;
	}

	// Newest FSMP-bugreport-*.zip in the log folder (across sessions), or empty.
	std::filesystem::path newestBundleZip()
	{
		const auto ld = logDir();
		if (ld.empty())
			return {};
		std::error_code ec;
		std::filesystem::path best;
		std::filesystem::file_time_type bestT{};
		for (const auto& e : std::filesystem::directory_iterator(ld, ec)) {
			if (e.path().extension() != ".zip")
				continue;
			if (e.path().filename().string().rfind("FSMP-bugreport-", 0) != 0)
				continue;
			const auto t = e.last_write_time(ec);
			if (best.empty() || t > bestT) {
				best = e.path();
				bestT = t;
			}
		}
		return best;
	}

	// A filename-safe local timestamp, e.g. "20260702-143501".
	std::string fileTimestamp()
	{
		const std::time_t t = std::time(nullptr);
		std::tm tm{};
		localtime_s(&tm, &t);
		char buf[32] = "";
		std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm);
		return buf;
	}

	// Deflate the given (archive-name, source-path) pairs into a new zip; missing sources are skipped. Returns
	// false (and removes the partial file) if the archive couldn't be created or ended up empty.
	//
	// We read each source with std::ifstream (permissive shared read/write) and add it from memory, instead of
	// miniz's own mz_zip_writer_add_file: miniz opens with fopen_s, whose deny-write sharing FAILS to open the
	// live hdtSMP64.log while spdlog holds it open for writing --- so add_file would silently skip the very log
	// a bug report needs. ifstream opens it fine (the log-tail viewer relies on the same).
	bool writeZip(const std::filesystem::path& zipPath,
		const std::vector<std::pair<std::string, std::filesystem::path>>& entries)
	{
		mz_zip_archive zip{};
		if (!mz_zip_writer_init_file(&zip, zipPath.string().c_str(), 0))
			return false;
		bool added = false;
		for (const auto& [name, src] : entries) {
			std::error_code ec;
			if (src.empty() || !std::filesystem::exists(src, ec))
				continue;
			const std::string bytes = readFileBytes(src);
			if (mz_zip_writer_add_mem(&zip, name.c_str(), bytes.data(), bytes.size(), MZ_BEST_COMPRESSION))
				added = true;
		}
		const bool ok = added && mz_zip_writer_finalize_archive(&zip);
		mz_zip_writer_end(&zip);
		if (!ok) {
			std::error_code ec;
			std::filesystem::remove(zipPath, ec);
		}
		return ok;
	}

	// Build the bundle on a detached worker thread (one at a time). Collects the log, optionally a fresh
	// gear/errors-only report (the same work as `smp report gear`, run synchronously here) and the
	// newest crash log, then zips them into the log folder.
	void buildBugReportAsync(bool includeReport, bool includeCrash)
	{
		int expected = g_bundleState.exchange(1);
		if (expected == 1)  // already building
			return;
		std::thread([includeReport, includeCrash]() {
			const auto ld = logDir();
			std::vector<std::pair<std::string, std::filesystem::path>> entries;

			if (auto log = spdlog::default_logger())
				log->flush();  // make sure the on-disk log is current before we read it
			entries.emplace_back(std::string(Plugin::NAME) + ".log", ld / (std::string(Plugin::NAME) + ".log"));

			if (includeReport) {
				std::string reportPath;
				hdt::ValidatePhysicsAssets(reportPath, /*equippedOnly=*/true, hdt::ValidationReportMode::ErrorsOnly);
				if (!reportPath.empty())
					entries.emplace_back(std::filesystem::path(reportPath).filename().string(), reportPath);
			}
			if (includeCrash) {
				const auto crash = newestCrashLog();
				if (!crash.empty())
					entries.emplace_back(crash.filename().string(), crash);
			}

			const std::filesystem::path zipPath = ld / ("FSMP-bugreport-" + fileTimestamp() + ".zip");
			const bool ok = !ld.empty() && writeZip(zipPath, entries);
			if (ok) {
				std::lock_guard<std::mutex> lk(g_bundleFileMx);
				g_bundleFile = zipPath.filename().string();
			}
			g_bundleState = ok ? 2 : 3;
		}).detach();
	}

	// Open the SKSE log folder in Explorer, preselecting the newest bundle zip if one exists.
	void openLogsFolder()
	{
		const auto ld = logDir();
		if (ld.empty())
			return;
		const auto zip = newestBundleZip();
		if (!zip.empty()) {
			const std::wstring args = L"/select,\"" + zip.wstring() + L"\"";
			ShellExecuteW(nullptr, L"open", L"explorer.exe", args.c_str(), nullptr, SW_SHOWNORMAL);
		} else {
			ShellExecuteW(nullptr, L"open", ld.wstring().c_str(), nullptr, nullptr, SW_SHOWNORMAL);
		}
	}

	// The bug-report bundle UI, shown on the Home page under the web links.
	void bugReportSection()
	{
		section(fa::FileZip, "Bundle a bug report");

		ImGuiMCP::BeginDisabled(true);  // the log is always included
		bool always = true;
		ImGuiMCP::Checkbox(tr("Current hdtSMP64.log"), &always);
		ImGuiMCP::EndDisabled();

		static bool inclReport = false;
		static bool inclCrash = false;
		ImGuiMCP::Checkbox(tr("Last validation report"), &inclReport);
		tip("Adds a fresh 'smp report gear' report to the zip. This takes a few seconds.");
		ImGuiMCP::Checkbox(tr("Last crash log"), &inclCrash);
		tip("Adds the most recent CrashLogger crash log, if any.");

		const bool building = g_bundleState.load() == 1;
		ImGuiMCP::BeginDisabled(building);
		if (ImGuiMCP::Button(tr("Build report zip")))
			buildBugReportAsync(inclReport, inclCrash);
		ImGuiMCP::EndDisabled();
		ImGuiMCP::SameLine();
		FontAwesome::PushSolid();
		const bool openf = ImGuiMCP::Button(fa::FolderOpen);
		FontAwesome::Pop();
		tip("Open logs folder");
		if (openf)
			openLogsFolder();

		switch (g_bundleState.load()) {
		case 1:
			ImGuiMCP::TextColored(kDim, "%s", tr("Building the report zip..."));
			break;
		case 2:
			{
				std::string f;
				{
					std::lock_guard<std::mutex> lk(g_bundleFileMx);
					f = g_bundleFile;
				}
				ImGuiMCP::TextColored(kOkTxt, "%s %s", tr("Report zip saved:"), f.c_str());
				break;
			}
		case 3:
			ImGuiMCP::TextColored(kBadTxt, "%s", tr("Could not build the report zip."));
			break;
		default:
			break;
		}
	}

	void HomeBody()
	{
		static ImGuiMCP::ImTextureID logo =
			SKSEMenuFramework::LoadTexture("Data/SKSE/Plugins/hdtSkinnedMeshConfigs/FSMP.dds");

		ImGuiMCP::NewLine();
		if (logo) {
			// The logo spans half the page width, keeping the source image's 768x444 aspect ratio, centred.
			ImGuiMCP::ImVec2 avail{};
			ImGuiMCP::GetContentRegionAvail(&avail);
			const float logoW = avail.x * 0.5f;
			const float logoH = logoW * 444.0f / 768.0f;
			ImGuiMCP::SetCursorPosX(ImGuiMCP::GetCursorPosX() + std::max(0.0f, (avail.x - logoW) * 0.5f));
			ImGuiMCP::Image(logo, ImGuiMCP::ImVec2{ logoW, logoH });
		}
		const std::string title =
			"Faster HDT-SMP  v" + Plugin::VERSION.string() + " (" + std::string(Plugin::AVX_VARIANT) + ")";
		centeredText(ImGuiMCP::ImVec4{ 1.0f, 1.0f, 1.0f, 1.0f }, title.c_str());
		centeredText(kDim, tr("In-game configuration menu for Faster HDT-SMP."));

		ImGuiMCP::NewLine();
		ImGuiMCP::Separator();
		if (linkButton("Documentation"))
			ShellExecuteA(nullptr, "open", "https://github.com/DaymareOn/hdtSMP64/wiki", nullptr, nullptr, SW_SHOWNORMAL);
		if (linkButton("Nexus page"))
			ShellExecuteA(nullptr, "open", "https://www.nexusmods.com/skyrimspecialedition/mods/57339", nullptr, nullptr, SW_SHOWNORMAL);
		if (linkButton("Report a bug"))
			ShellExecuteA(nullptr, "open", "https://github.com/DaymareOn/hdtSMP64/issues", nullptr, nullptr, SW_SHOWNORMAL);

		bugReportSection();
	}

	void PresetsBody()
	{
		if (!g_presetsScanned)
			scanPresets();

		ImGuiMCP::NewLine();  // one blank row after the header, like every page
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
				// Apply the preset's physics fields, but keep the user's mods-dir, node backups, and UI
				// settings (language, font scales).
				GlobalConfig next = preset.cfg;
				const GlobalConfig cur = hdt::readConfig();
				next.modsDir = cur.modsDir;
				next.backupNodeByName = cur.backupNodeByName;
				next.locale = cur.locale;
				next.outputFontScale = cur.outputFontScale;
				next.overlayFontScale = cur.overlayFontScale;
				hdt::applyConfig(next);
				commitReset();
			}
			if (active)
				ImGuiMCP::PopStyleColor(3);
		}
	}

	// ---- Section-item render functions ------------------------------------------------------------------
	// One left-pane item per page. Each draws the shared status header (chrome) then its body. Home is the
	// landing page (first item, shown when clicking "FSMP"): the brand and the web links live there. The page
	// content retranslates live when the language changes; the left-pane item labels are fixed at
	// registration (the framework has no rename), so they re-translate on the next game launch.
	void __stdcall RenderHome()
	{
		chrome();
		HomeBody();
	}
	void __stdcall RenderPresets()
	{
		chrome();
		PresetsBody();
	}
	void __stdcall RenderSimplification()
	{
		chrome();
		SimplificationBody();
	}
	void __stdcall RenderPerformance()
	{
		chrome();
		PerformanceBody();
	}
	void __stdcall RenderWind()
	{
		chrome();
		WindBody();
	}
	void __stdcall RenderCommands()
	{
		chrome();
		CommandsBody();
	}
	void __stdcall RenderMeasures()
	{
		chrome();
		MeasuresBody();
	}
	void __stdcall RenderLogs()
	{
		chrome();
		LogsBody();
	}
	void __stdcall RenderValidation()
	{
		chrome();
		ValidationBody();
	}
}

namespace hdt::FSMPMenu
{
	void Register()
	{
		hdt::loc::load();  // load the localized strings before we register any translated page name

		SKSEMenuFramework::SetSection("FSMP");
		SKSEMenuFramework::AddSectionItem(tr("Home"), RenderHome);
		SKSEMenuFramework::AddSectionItem(tr("Presets"), RenderPresets);
		SKSEMenuFramework::AddSectionItem(tr("Simplification"), RenderSimplification);
		SKSEMenuFramework::AddSectionItem(tr("Performance"), RenderPerformance);
		SKSEMenuFramework::AddSectionItem(tr("Wind"), RenderWind);
		SKSEMenuFramework::AddSectionItem(tr("Commands"), RenderCommands);
		SKSEMenuFramework::AddSectionItem(tr("Measures"), RenderMeasures);
		SKSEMenuFramework::AddSectionItem(tr("Logs"), RenderLogs);
		SKSEMenuFramework::AddSectionItem(tr("Validation"), RenderValidation);

		// A non-blocking overlay window for the Measures page's "show while playing" toggle; starts hidden.
		g_overlay = SKSEMenuFramework::AddWindow(RenderPerfOverlay, false);
		if (g_overlay)
			g_overlay->IsOpen = false;
	}
}
