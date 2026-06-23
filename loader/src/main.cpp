// hdtSMP64 loader stub
//
// This DLL is the only one SKSE auto-loads. It contains NO physics code and NO AVX instructions, so
// it runs on every x64 CPU. At SKSEPlugin_Load it:
//   1. detects the highest instruction-set tier the CPU+OS can safely run,
//   2. applies an optional INI override and a persisted crash "ceiling",
//   3. LoadLibrary's the best matching optimized variant from SKSE/Plugins/FSMP/, and
//   4. forwards the SKSE LoadInterface to that variant's own SKSEPlugin_Load.
//
// Because the variant receives the exact same LoadInterface (and therefore the same SKSE plugin
// handle and name as this loader declares), the FSMP plugin API - which is published purely through
// SKSE messaging under Plugin::NAME - keeps working for other mods unchanged.

// CommonLibSSE-NG umbrella first, in the same order as the physics plugin's PCH: RE/Skyrim.h sets up
// the standard-library prerequisites (std::mutex/span) and the CommonLibSSE `stl` helpers that the
// REL/SKSE headers below rely on. Without it those headers fail to compile.
#include <RE/Skyrim.h>
#include <REL/Relocation.h>
#include <SKSE/SKSE.h>

#include "CpuFeatures.h"
#include "CrashState.h"
#include "VariantSelector.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include <Windows.h>

#include "Plugin.h"

#define DLLEXPORT __declspec(dllexport)

namespace logger = SKSE::log;

using namespace std::literals;

namespace
{
	// How long a session must run without crashing before we trust the loaded variant. The pending
	// attempt marker is cleared after this window; a crash before it elapses lowers the ceiling on
	// the next launch. Chosen generously so early cell loads (the first time physics actually runs)
	// are comfortably inside the window.
	constexpr auto kStabilityWindow = std::chrono::seconds(120);

	// Subfolder (under the loader) that holds the optimized variant DLLs. It is deliberately NOT the
	// Plugins root: SKSE scans Plugins non-recursively, so keeping the variants one level down stops
	// SKSE from auto-loading them (which would double-load and could run an unsupported ISA).
	constexpr auto kVariantSubfolder = L"FSMP";

	// Resolve a module's full path, growing the buffer until it fits (MO2's virtualized paths can be
	// longer than MAX_PATH). Returns empty on failure.
	std::wstring moduleFilePath(HMODULE mod)
	{
		std::wstring buf(MAX_PATH, L'\0');
		for (;;) {
			const DWORD n = GetModuleFileNameW(mod, buf.data(), static_cast<DWORD>(buf.size()));
			if (n == 0) {
				return {};
			}
			if (n < buf.size()) {
				buf.resize(n);
				return buf;
			}
			if (buf.size() >= 0x8000) {
				return {};  // absurdly long; give up rather than loop forever
			}
			buf.resize(buf.size() * 2);
		}
	}

	// The directory this loader DLL lives in, used to locate the variant subfolder and the INI. We
	// resolve our own module by address (a function inside this DLL) rather than assuming the EXE's
	// directory, so it is correct regardless of the process current directory.
	std::filesystem::path loaderDirectory()
	{
		HMODULE self = nullptr;
		if (!GetModuleHandleExW(
				GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
				reinterpret_cast<LPCWSTR>(&moduleFilePath),
				&self)) {
			return {};
		}
		const std::wstring path = moduleFilePath(self);
		if (path.empty()) {
			return {};
		}
		return std::filesystem::path(path).parent_path();
	}

	// Send the loader's own diagnostics to <SKSE log dir>/hdtsmp64_loader.log, separate from the
	// physics plugin's hdtsmp64.log so it is obvious which component reported what. Falls back
	// silently to no logging if the standard directory can't be resolved.
	void initLog()
	{
		auto dir = logger::log_directory();
		if (!dir) {
			return;
		}
		*dir /= fmt::format("{}_loader.log"sv, Plugin::NAME);

		auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(dir->string(), true);
		auto log = std::make_shared<spdlog::logger>("loader log"s, std::move(sink));
		log->set_level(spdlog::level::info);
		log->flush_on(spdlog::level::info);
		spdlog::set_default_logger(std::move(log));
		spdlog::set_pattern("[%H:%M:%S.%e] [%L] %v"s);
	}

	// Load one variant DLL and forward the SKSE load call into it, fully isolated by Structured
	// Exception Handling so a hardware fault (e.g. an illegal instruction) becomes a recoverable
	// "try the next variant" instead of crashing the game. The return contract:
	//     1  -> the variant loaded and its SKSEPlugin_Load returned true (success)
	//     0  -> a clean miss: file absent, missing export, or the variant returned false
	//    -1  -> a structured exception during load or init
	// Kept free of any C++ object with a destructor, as required for __try/__except.
	int tryLoadAndForward(const wchar_t* dllPath, const SKSE::LoadInterface* a_skse) noexcept
	{
		__try {
			HMODULE mod = LoadLibraryW(dllPath);
			if (!mod) {
				return 0;
			}
			using LoadFn = bool(__cdecl*)(const SKSE::LoadInterface*);
			auto fn = reinterpret_cast<LoadFn>(GetProcAddress(mod, "SKSEPlugin_Load"));
			if (!fn) {
				FreeLibrary(mod);
				return 0;
			}
			// On success the module stays resident for the process lifetime. On a `false` return we
			// intentionally do NOT FreeLibrary: a declining variant may already have installed hooks
			// that must not be torn out from under the game.
			return fn(a_skse) ? 1 : 0;
		} __except ((GetExceptionCode() == EXCEPTION_ILLEGAL_INSTRUCTION ||
						GetExceptionCode() == EXCEPTION_PRIV_INSTRUCTION)
					   ? EXCEPTION_EXECUTE_HANDLER
					   : EXCEPTION_CONTINUE_SEARCH) {
			return -1;
		}
	}
}

extern "C" DLLEXPORT bool SKSEAPI SKSEPlugin_Query(const SKSE::QueryInterface* a_skse, SKSE::PluginInfo* a_info)
{
	// Legacy handshake (needed for Skyrim SE 1.5.x, which predates the version-data mechanism). The
	// declared identity must match the physics plugin so other mods still recognise us by name.
	a_info->infoVersion = SKSE::PluginInfo::kVersion;
	a_info->name = Plugin::NAME.data();
	a_info->version = Plugin::VERSION.pack();

	if (a_skse->IsEditor()) {
		return false;
	}

	const auto ver = a_skse->RuntimeVersion();
	if ((REL::Module::IsSE() && ver < SKSE::RUNTIME_SSE_1_5_39) ||
		(REL::Module::IsVR() && ver < SKSE::RUNTIME_LATEST_VR)) {
		return false;
	}

	return true;
}

// Modern handshake. These must mirror the physics plugin's declaration exactly: SKSE validates this
// data on the loader (the only DLL it scans), never on the variants we load manually, so the
// loader is what advertises runtime compatibility and address-library use on behalf of the variant.
extern "C" DLLEXPORT constinit auto SKSEPlugin_Version = []() {
	SKSE::PluginVersionData v;

	v.PluginVersion(Plugin::VERSION);
	v.PluginName(Plugin::NAME);
	v.UsesAddressLibrary();
	v.CompatibleVersions({ SKSE::RUNTIME_SSE_LATEST_SE, SKSE::RUNTIME_SSE_LATEST, SKSE::RUNTIME_1_6_1179, SKSE::RUNTIME_LATEST_VR });
	v.UsesNoStructs();

	return v;
}();

extern "C" DLLEXPORT bool SKSEAPI SKSEPlugin_Load(const SKSE::LoadInterface* a_skse)
{
	initLog();

	if constexpr (Plugin::BUILD_INFO.empty()) {
		logger::info("{} loader v{}"sv, Plugin::NAME, Plugin::VERSION.string());
	} else {
		logger::info("{} loader v{}-{}"sv, Plugin::NAME, Plugin::VERSION.string(), Plugin::BUILD_INFO);
	}

	const std::filesystem::path dir = loaderDirectory();
	if (dir.empty()) {
		logger::error("could not resolve loader directory; cannot locate variant DLLs"sv);
		return false;
	}

	// 1) What can the hardware actually run?
	const loader::Variant cpuMax = loader::detectHighestSupportedVariant();
	logger::info("CPU supports up to: {}"sv, loader::variantToken(cpuMax));

	// 2) Persisted crash history (kept in the real, writable SKSE log dir, not the virtualized Data
	//    tree). Static so the stability timer thread can safely reference it for the process life.
	auto stateDir = logger::log_directory();
	static loader::CrashState crashState(
		(stateDir ? *stateDir : dir) / fmt::format("{}_loader.state"sv, Plugin::NAME));
	crashState.load();
	switch (crashState.consumePendingCrash()) {
	case loader::CrashState::Check::CeilingLowered:
		logger::warn("previous sessions repeatedly unstable; crash ceiling lowered to {}"sv,
			loader::variantToken(crashState.ceiling()));
		break;
	case loader::CrashState::Check::StrikeRecorded:
		logger::warn("previous session ended before stability (strike recorded); keeping ceiling {}"sv,
			loader::variantToken(crashState.ceiling()));
		break;
	case loader::CrashState::Check::CleanLastSession:
		break;
	}

	// 3) Optional override from the (user/MCM-editable) INI next to the loader.
	const loader::ForceResult force = loader::readForcedVariant(dir / fmt::format("{}_loader.ini"sv, Plugin::NAME));
	if (force.explicitChoice) {
		// A deliberate choice clears stale crash history so it isn't second-guessed.
		crashState.resetCeiling();
		if (force.variant) {
			logger::info("INI forces variant: {}"sv, loader::variantToken(*force.variant));
		} else {
			logger::info("INI forces auto-detection"sv);
		}
	}

	// 4) Build the ordered candidate list and try each, stepping down on failure.
	const loader::Selection selection = loader::buildSelection(cpuMax, crashState.ceiling(), force.variant);
	logger::info("selected variant: {} (will fall back to lower tiers on failure)"sv,
		loader::variantToken(selection.start));

	for (const loader::Variant candidate : selection.candidates) {
		const std::filesystem::path dll =
			dir / kVariantSubfolder / fmt::format("{}_{}.dll"sv, Plugin::NAME, loader::variantToken(candidate));

		if (!std::filesystem::exists(dll)) {
			logger::warn("variant DLL missing, skipping: {}"sv, dll.string());
			continue;
		}

		crashState.beginAttempt(candidate);
		logger::info("loading variant {}: {}"sv, loader::variantToken(candidate), dll.string());

		const int result = tryLoadAndForward(dll.c_str(), a_skse);
		if (result == 1) {
			logger::info("variant {} loaded successfully"sv, loader::variantToken(candidate));

			// Clear the pending attempt once we've run stably for a while, so a clean session does
			// not look like a crash next launch. Detached: the load thread must return promptly.
			std::thread([] {
				std::this_thread::sleep_for(kStabilityWindow);
				crashState.markStable();
			}).detach();

			return true;
		}

		if (result < 0) {
			logger::error("variant {} faulted during load/init; blaming it and falling back"sv,
				loader::variantToken(candidate));
			crashState.blame(candidate);
		} else {
			logger::warn("variant {} declined to load; trying next tier"sv, loader::variantToken(candidate));
		}
	}

	logger::critical("no usable hdtSMP64 variant could be loaded"sv);
	return false;
}
