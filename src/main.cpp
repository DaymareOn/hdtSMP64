#include "ActorManager.h"
#include "Events.h"
#include "Hooks.h"
#include "PluginInterfaceImpl.h"
#include "SMPDebug.h"
#include "UI/FSMPMenu.h"
#include "Validator/hdtAssetValidator.h"
#include "WeatherManager.h"
#include "config.h"
#include "dhdtOverrideManager.h"
#include "dhdtPapyrusFunctions.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <atomic>
#include <charconv>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <string_view>
#include <thread>

namespace
{
	std::uint64_t ParsePositiveDecimal(std::string_view a_value, std::uint64_t a_fallback)
	{
		if (a_value.empty()) {
			return a_fallback;
		}

		std::uint64_t parsed = 0;
		const auto [end, error] = std::from_chars(a_value.data(), a_value.data() + a_value.size(), parsed);

		if (error != std::errc{} || end != a_value.data() + a_value.size() || parsed == 0) {
			return a_fallback;
		}

		return static_cast<std::uint64_t>(parsed);
	}
}

void checkOldPlugins()
{
	auto framework = GetModuleHandleA("hdtSSEFramework");
	auto physics = GetModuleHandleA("hdtSSEPhysics");
	auto hh = GetModuleHandleA("hdtSSEHighHeels");

	if (physics) {
		MessageBox(nullptr, TEXT("hdtSSEPhysics.dll is loaded. This is an older version of HDT-SMP and conflicts with hdtSMP64.dll. Please remove it."), TEXT("hdtSMP64"), MB_OK);
	}

	if (framework && !hh) {
		MessageBox(nullptr, TEXT("hdtSSEFramework.dll is loaded but hdtSSEHighHeels.dll is not being used. You no longer need hdtSSEFramework.dll with this version of SMP. Please remove it."), TEXT("hdtSMP64"), MB_OK);
	}
}

namespace
{
	// One node-dump line: to the log (as before) AND to the menu's Output panel, so the `smp dumptree`
	// content itself is readable in the configuration menu without opening the log file.
	template <typename... Args>
	void dumpLine(fmt::format_string<Args...> fmtStr, Args&&... args)
	{
		std::string line = fmt::format(fmtStr, std::forward<Args>(args)...);
		logger::info("{}", line);
		hdt::menuConsoleAppendLines(line);
	}
}

void DumpNodeChildren(RE::NiAVObject* node)
{
	dumpLine(
		"{} {} [{:.2f}, {:.2f}, {:.2f}]",
		node->GetRTTI()->name,
		node->name,
		node->world.translate.x,
		node->world.translate.y,
		node->world.translate.z);

	if (node->extraDataSize > 0) {
		for (uint16_t i = 0; i < node->extraDataSize; i++) {
			dumpLine(
				"{} {}",
				node->extra[i]->GetRTTI()->name,
				node->extra[i]->name);
		}
	}

	RE::NiNode* niNode = node->AsNode();
	if (niNode) {
		auto& children = niNode->GetChildren();
		if (children.size() > 0) {
			for (uint16_t i = 0; i < children.size(); i++) {
				RE::NiPointer<RE::NiAVObject> object = children[i];
				if (object) {
					RE::NiNode* childNode = object->AsNode();
					RE::BSGeometry* geometry = object->AsGeometry();
					if (geometry) {
						logger::info(
							"{} {} [{:.2f}, {:.2f}, {:.2f}] - Geometry",
							object->GetRTTI()->name,
							object->name,
							geometry->world.translate.x,
							geometry->world.translate.y,
							geometry->world.translate.z);

						if (geometry->GetGeometryRuntimeData().skinInstance && geometry->GetGeometryRuntimeData().skinInstance->skinData) {
							for (uint32_t boneIdx = 0; boneIdx < geometry->GetGeometryRuntimeData().skinInstance->skinData->GetBoneCount(); boneIdx++) {
								auto bone = geometry->GetGeometryRuntimeData().skinInstance->bones[boneIdx];
								logger::info(
									"Bone {} - {} {} [{:.2f}, {:.2f}, {:.2f}]",
									boneIdx,
									bone->GetRTTI()->name,
									bone->name,
									bone->world.translate.x,
									bone->world.translate.y,
									bone->world.translate.z);
							}
						}

						RE::BSShaderProperty* shaderProperty = geometry->GetGeometryRuntimeData().shaderProperty.get();
						if (shaderProperty) {
							RE::BSLightingShaderProperty* lightingShader = netimmerse_cast<RE::BSLightingShaderProperty*>(shaderProperty);
							if (lightingShader) {
								RE::BSLightingShaderMaterial* material = static_cast<RE::BSLightingShaderMaterial*>(lightingShader->material);

								if (material) {
									// GetTextures is the game's own polymorphic accessor: each material subtype fills
									// in its loaded texture objects itself, so we never cast to a subtype (the old
									// helper did, and mis-cast a glowmap material to a Facegen one -> garbage deref ->
									// CTD). Zero-initialised and generously oversized so no material can overflow it
									// and any slot the material doesn't set stays null (skipped below). We log both the
									// texture-set PATH (authoritative) and the loaded object's runtime name per slot.
									RE::NiSourceTexture* loaded[64] = {};
									material->GetTextures(loaded);
									for (std::uint32_t slot = 0; slot < RE::BSTextureSet::Textures::kTotal; ++slot) {
										const char* path = material->textureSet ? material->textureSet->GetTexturePath(
																					  static_cast<RE::BSTextureSet::Textures::Texture>(slot)) :
										                                          nullptr;
										const RE::NiSourceTexture* tex = loaded[slot];
										const char* name = tex ? tex->name.c_str() : "";
										if ((path && path[0]) || (name && name[0]))
											logger::info("Texture {} - {} ({})", slot, path ? path : "", name);
									}
								}

								logger::info(
									"Flags - {:08X}",
									lightingShader->flags.underlying());
							}
						}
					} else if (childNode) {
						DumpNodeChildren(childNode);
					} else {
						logger::info(
							"{} {} [{:.2f}, {:.2f}, {:.2f}]",
							object->GetRTTI()->name,
							object->name,
							object->world.translate.x,
							object->world.translate.y,
							object->world.translate.z);
					}
				}
			}
		}
	}
}

namespace hdt
{
	namespace
	{
		// The menu-visible console capture: a bounded ring buffer of recent smp-command output lines. Guarded
		// by a mutex because `smp report` finishes on a worker thread and echoes its result from there while
		// the render thread reads the buffer. The cap bounds memory when a whole validation report or node
		// dump is appended (only the newest lines are kept); the menu renders it with a list clipper, so the
		// size costs nothing per frame.
		constexpr size_t kMenuConsoleMax = 5000;
		std::mutex g_menuConsoleMutex;
		std::deque<std::string> g_menuConsoleLines;

		// Append one already-formatted line under the lock, evicting the oldest past the cap.
		void menuConsolePushLine(std::string line)
		{
			std::lock_guard<std::mutex> lk(g_menuConsoleMutex);
			g_menuConsoleLines.emplace_back(std::move(line));
			while (g_menuConsoleLines.size() > kMenuConsoleMax)
				g_menuConsoleLines.pop_front();
		}
	}

	void smpEcho(const char* fmt, ...)
	{
		char buf[2048];
		va_list ap;
		va_start(ap, fmt);
		const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
		va_end(ap);
		if (n < 0)
			buf[0] = '\0';

		// Print to the game console exactly as before ("%s" guards a stray '%' in the formatted text).
		if (auto* console = RE::ConsoleLog::GetSingleton())
			console->Print("%s", buf);

		// Mirror the line into the menu ring buffer, minus any leading "[Tag]" prefixes ("[HDT-SMP] ", ...):
		// in the shared game console the tag identifies the source mod, but the menu's Output panel shows only
		// FSMP lines, so the tag is pure noise there. Lines that don't start with '[' (e.g. indented usage
		// lines) pass through untouched, keeping their indentation.
		const char* p = buf;
		while (*p == '[') {
			const char* close = std::strchr(p, ']');
			if (!close)
				break;
			p = close + 1;
			while (*p == ' ')
				++p;
		}
		menuConsolePushLine(p);
	}

	void menuConsoleAppendLines(std::string_view text)
	{
		// Split on '\n' (tolerating "\r\n") and push each line; no console print and no prefix stripping ---
		// generated documents (reports, dumps, profiler tables) are appended verbatim.
		size_t start = 0;
		while (start <= text.size()) {
			size_t end = text.find('\n', start);
			if (end == std::string_view::npos)
				end = text.size();
			size_t len = end - start;
			if (len > 0 && text[start + len - 1] == '\r')
				--len;
			if (len > 0 || end < text.size())  // keep interior blank lines, drop a trailing empty tail
				menuConsolePushLine(std::string(text.substr(start, len)));
			start = end + 1;
		}
	}

	std::vector<std::string> menuConsoleSnapshot()
	{
		std::lock_guard<std::mutex> lk(g_menuConsoleMutex);
		return { g_menuConsoleLines.begin(), g_menuConsoleLines.end() };
	}

	void clearMenuConsole()
	{
		std::lock_guard<std::mutex> lk(g_menuConsoleMutex);
		g_menuConsoleLines.clear();
	}
}

void SMPDebug_PrintDetailed(bool includeItems)
{
	static std::map<hdt::ActorManager::SkeletonState, const char*> stateStrings = {
		{ hdt::ActorManager::SkeletonState::e_InactiveNotInScene, "Not in scene" },
		{ hdt::ActorManager::SkeletonState::e_InactiveUnseenByPlayer, "Unseen by player" },
		{ hdt::ActorManager::SkeletonState::e_InactiveTooFar, "Deactivated for performance" },
		{ hdt::ActorManager::SkeletonState::e_ActiveIsPlayer, "Is player character" },
		{ hdt::ActorManager::SkeletonState::e_ActiveNearPlayer, "Is near player" }
	};

	auto skeletons = hdt::ActorManager::instance()->getSkeletons();
	std::vector<int> order(skeletons.size());
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](int a, int b) { return skeletons[a].state < skeletons[b].state; });

	for (int i : order) {
		auto& skeleton = skeletons[i];

		RE::TESObjectREFR* skelOwner = nullptr;
		RE::TESFullName* ownerName = nullptr;

		if (skeleton.skeleton->GetUserData()) {
			skelOwner = skeleton.skeleton->GetUserData();
			if (skelOwner->GetBaseObject()) {
				ownerName = skyrim_cast<RE::TESFullName*>(skelOwner->GetBaseObject());
			}
		}

		hdt::smpEcho(
			"[HDT-SMP] %s skeleton - owner %s (refr formid %08x, base formid %08x) - %s",
			skeleton.state > hdt::ActorManager::SkeletonState::e_SkeletonActive ? "active" : "inactive",
			ownerName ? ownerName->GetFullName() : "unk_name",
			skelOwner ? skelOwner->formID : 0x00000000,
			skelOwner && skelOwner->GetBaseObject() ? skelOwner->GetBaseObject()->formID : 0x00000000,
			stateStrings[skeleton.state]);

		if (includeItems) {
			for (auto armor : skeleton.getArmors()) {
				hdt::smpEcho(
					"[HDT-SMP] -- tracked armor addon %s, %s",
					armor.armorWorn->name.c_str(),
					armor.state() != hdt::ActorManager::ItemState::e_NoPhysics ? armor.state() == hdt::ActorManager::ItemState::e_Active ? "has active physics system" : "has inactive physics system" : "has no physics system");

				if (armor.state() != hdt::ActorManager::ItemState::e_NoPhysics) {
					for (auto mesh : armor.meshes()) {
						hdt::smpEcho(
							"[HDT-SMP] ---- has collision mesh %s",
							mesh->m_name.c_str());
					}
				}
			}

			if (skeleton.head.headNode) {
				for (auto headPart : skeleton.head.headParts) {
					hdt::smpEcho(
						"[HDT-SMP] -- tracked headpart %s, %s",
						headPart.headPart->name.c_str(),
						headPart.state() != hdt::ActorManager::ItemState::e_NoPhysics ? headPart.state() == hdt::ActorManager::ItemState::e_Active ? "has active physics system" : "has inactive physics system" : "has no physics system");

					if (headPart.state() != hdt::ActorManager::ItemState::e_NoPhysics) {
						for (auto mesh : headPart.meshes()) {
							hdt::smpEcho("[HDT-SMP] ---- has collision mesh %s", mesh->m_name.c_str());
						}
					}
				}
			}
		}
	}
}

bool SMPDebug_Execute(
	const RE::SCRIPT_PARAMETER* a_paramInfo,
	RE::SCRIPT_FUNCTION::ScriptData* a_scriptData,
	RE::TESObjectREFR* a_thisObj,
	RE::TESObjectREFR* a_containingObj,
	RE::Script* a_scriptObj,
	RE::ScriptLocals* a_locals,
	[[maybe_unused]] double& a_result,
	uint32_t& a_opcodeOffsetPtr)
{
	char buffer[MAX_PATH];
	memset(buffer, 0, MAX_PATH);
	char buffer2[MAX_PATH];
	memset(buffer2, 0, MAX_PATH);
	char buffer3[MAX_PATH];
	memset(buffer3, 0, MAX_PATH);

	if (!RE::Script::ParseParameters(a_paramInfo, a_scriptData, a_opcodeOffsetPtr, a_thisObj, a_containingObj, a_scriptObj, a_locals, buffer, buffer2, buffer3)) {
		return false;
	}

	logger::debug("SMPCommand: {} {} {}"sv, buffer, buffer2, buffer3);

	return hdt::RunSMPDebugCommand(buffer, buffer2, buffer3, a_thisObj);
}

// The actual smp subcommand dispatch, split out from the console entry point (SMPDebug_Execute) so the
// in-game menu's Commands page can run the same commands without going through the console parser. Defined
// with a qualified name so the moved body keeps its original indentation. a_thisObj is the console's
// targeted reference (used by "dumptree"); the menu passes the player or null.
bool hdt::RunSMPDebugCommand(const char* buffer, const char* buffer2, const char* buffer3,
	RE::TESObjectREFR* a_thisObj)
{
	if (_strnicmp(buffer, "help", MAX_PATH) == 0) {
		hdt::smpEcho("[HDT-SMP] Available smp commands:");
		hdt::smpEcho("  smp help");
		hdt::smpEcho("    Show this command reference.");
		hdt::smpEcho("  smp reset");
		hdt::smpEcho("    Reload SMP config and reset all active physics systems.");
		hdt::smpEcho("  smp dumptree");
		hdt::smpEcho("    Dump the targeted reference's 3D node tree to console.");
		hdt::smpEcho("  smp detail");
		hdt::smpEcho("    Print detailed tracked skeleton/item diagnostics.");
		hdt::smpEcho("  smp list");
		hdt::smpEcho("    Print a compact tracked skeleton summary.");
		hdt::smpEcho("  smp profile [sample_frames] [print_every_frames]");
		hdt::smpEcho("    Toggle physics profiler capture; defaults are 240/240.");
		hdt::smpEcho("  smp on");
		hdt::smpEcho("    Enable SMP simulation.");
		hdt::smpEcho("  smp off");
		hdt::smpEcho("    Disable SMP simulation.");
		hdt::smpEcho("  smp QueryOverride");
		hdt::smpEcho("    Print current dynamic override data.");
		hdt::smpEcho("  smp report [gear] [warnings]");
		hdt::smpEcho("    Run the physics-asset validator in the background and write a report file.");
		hdt::smpEcho("    Default: errors only (no warnings/info).");
		hdt::smpEcho("    gear     = validate equipped gear only.");
		hdt::smpEcho("    warnings = also include warnings and info in the report.");
		return true;
	}

	if (_strnicmp(buffer, "reset", MAX_PATH) == 0) {
		logger::debug("smp reset: reloading config and resetting physics world"sv);
		hdt::smpEcho("running full smp reset");
		hdt::applyConfigReset();
		return true;
	}
	if (_strnicmp(buffer, "dumptree", MAX_PATH) == 0) {
		if (a_thisObj) {
			hdt::smpEcho("dumping targeted reference's node tree");
			DumpNodeChildren(a_thisObj->Get3D1(0));
		} else {
			hdt::smpEcho("error: you must target a reference to dump their node tree");
		}

		return true;
	}

	if (_strnicmp(buffer, "detail", MAX_PATH) == 0) {
		SMPDebug_PrintDetailed(true);
		return true;
	}

	if (_strnicmp(buffer, "list", MAX_PATH) == 0) {
		SMPDebug_PrintDetailed(false);
		return true;
	}

	if (_strnicmp(buffer, "profile", MAX_PATH) == 0) {
		static bool profilerCaptureRequested = false;

		profilerCaptureRequested = !profilerCaptureRequested;

		const auto sampleFrames = ParsePositiveDecimal(buffer2, 240);
		const auto printFrames = ParsePositiveDecimal(buffer3, 240);

		hdt::SkyrimPhysicsWorld::get()->setProfilerCapture(profilerCaptureRequested, sampleFrames, printFrames);

		if (profilerCaptureRequested) {
			hdt::smpEcho(
				"HDT-SMP physics profiler enabled: sample %llu frames, print every %llu frames. Results appear "
				"here (and in the log) after that many frames of gameplay --- the game must run, not sit in a menu.",
				static_cast<unsigned long long>(sampleFrames),
				static_cast<unsigned long long>(printFrames));
			hdt::smpEcho("Check your hdtsmp64.log file for results.");

		} else {
			hdt::smpEcho("HDT-SMP physics profiler disabled");
		}

		return true;
	}

	if (_strnicmp(buffer, "on", MAX_PATH) == 0) {
		hdt::SkyrimPhysicsWorld::get()->disabled = false;
		{
			hdt::smpEcho("HDT-SMP enabled");
		}
		return true;
	}

	if (_strnicmp(buffer, "off", MAX_PATH) == 0) {
		hdt::SkyrimPhysicsWorld::get()->disabled = true;
		{
			hdt::smpEcho("HDT-SMP disabled");
		}
		return true;
	}

	if (_strnicmp(buffer, "QueryOverride", MAX_PATH) == 0) {
		hdt::smpEcho(hdt::Override::OverrideManager::GetSingleton()->queryOverrideData().c_str());
		return true;
	}

	if (_strnicmp(buffer, "report", MAX_PATH) == 0) {
		static std::atomic<bool> s_validationRunning{ false };

		bool gearOnly = false;
		bool includeWarnings = false;  // default: errors only (an explicit 'warnings' opts in)
		auto parseValidateModeArg = [&](const char* arg) {
			if (arg[0] == '\0')
				return true;
			if (_stricmp(arg, "gear") == 0) {
				gearOnly = true;
				return true;
			}
			if (_stricmp(arg, "warnings") == 0) {
				includeWarnings = true;
				return true;
			}
			hdt::smpEcho("[Validator] Unknown report mode: %s", arg);
			hdt::smpEcho("[Validator] Usage: smp report [gear] [warnings]");
			return false;
		};

		if (!parseValidateModeArg(buffer2) || !parseValidateModeArg(buffer3))
			return true;

		if (s_validationRunning.exchange(true)) {
			hdt::smpEcho("[Validator] Validation is already running.");
			return true;
		}
		if (gearOnly && includeWarnings) {
			hdt::smpEcho("[Validator] Equipped gear report (with warnings) started in background. Results will appear when complete.");
		} else if (gearOnly) {
			hdt::smpEcho("[Validator] Equipped gear report (errors only) started in background. Results will appear when complete.");
		} else if (includeWarnings) {
			hdt::smpEcho("[Validator] Report (with warnings) started in background. Results will appear when complete.");
		} else {
			hdt::smpEcho("[Validator] Report (errors only) started in background. Results will appear when complete.");
		}
		std::thread([gearOnly, includeWarnings]() {
			try {
				const char* validationLabel = gearOnly ? "Equipped gear report" : "Report";
				std::string reportPath;
				auto result = hdt::ValidatePhysicsAssets(
					reportPath,
					gearOnly,
					includeWarnings ? hdt::ValidationReportMode::Full : hdt::ValidationReportMode::ErrorsOnly);
				// Show the report CONTENT in the menu's Output panel, not just the fact it was generated.
				// Only the tail that fits the menu ring buffer is appended (a full report can be tens of MB),
				// scanned backwards so the huge file is never split line-by-line from the start. The summary
				// and file path are echoed after it, so they stay visible at the bottom.
				if (!reportPath.empty()) {
					std::ifstream in(reportPath, std::ios::binary);
					if (in) {
						std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
						constexpr size_t kMaxReportLines = 5000;
						size_t lines = 0, pos = bytes.size();
						while (pos > 0 && lines < kMaxReportLines)
							if (bytes[--pos] == '\n')
								++lines;
						if (pos > 0) {
							hdt::menuConsoleAppendLines("(report truncated here; its beginning is in the file)");
							pos += 1;  // skip the newline the scan stopped on
						}
						hdt::menuConsoleAppendLines(std::string_view(bytes).substr(pos));
					}
				}
				// Route through smpEcho so the summary shows in the menu's Output panel too (not just the
				// console). errors-only is the default; 'warnings' opts into the full report (see PR #403).
				if (includeWarnings) {
					hdt::smpEcho(
						"[Validator] %s complete in %.2fs: %d XML(s) found, %d passed, %d failed, %d warning(s)",
						validationLabel,
						result.elapsedSeconds,
						result.totalXMLsFound, result.xmlPassCount, result.xmlErrorCount,
						(int)result.warnings.size());
				} else {
					hdt::smpEcho(
						"[Validator] %s complete in %.2fs: %d XML(s) found, %d failed (errors only)",
						validationLabel,
						result.elapsedSeconds,
						result.totalXMLsFound,
						result.xmlErrorCount);
				}
				if (!reportPath.empty()) {
					if (includeWarnings) {
						hdt::smpEcho("[Validator] Report written to: %s", reportPath.c_str());
					} else {
						hdt::smpEcho("[Validator] Errors-only report written to: %s", reportPath.c_str());
					}
				} else {
					hdt::smpEcho("[Validator] Warning: report file could not be written");
				}
			} catch (const std::exception& e) {
				hdt::smpEcho("[Validator] Report failed with error: %s", e.what());
				logger::error("[Validator] smp report threw: {}", e.what());
			} catch (...) {
				hdt::smpEcho("[Validator] Report failed with an unknown error");
				logger::error("[Validator] smp report threw an unknown exception");
			}
			s_validationRunning.store(false);
		}).detach();
		return true;
	}

	auto skeletons = hdt::ActorManager::instance()->getSkeletons();

	size_t activeSkeletons = 0;
	size_t armors = 0;
	size_t headParts = 0;
	size_t activeArmors = 0;
	size_t activeHeadParts = 0;
	size_t activeCollisionMeshes = 0;

	for (auto skeleton : skeletons) {
		if (skeleton.state > hdt::ActorManager::SkeletonState::e_SkeletonActive)
			activeSkeletons++;

		for (const auto armor : skeleton.getArmors()) {
			armors++;

			if (armor.state() == hdt::ActorManager::ItemState::e_Active) {
				activeArmors++;

				activeCollisionMeshes += armor.meshes().size();
			}
		}

		if (skeleton.head.headNode) {
			for (const auto headpart : skeleton.head.headParts) {
				headParts++;

				if (headpart.state() == hdt::ActorManager::ItemState::e_Active) {
					activeHeadParts++;

					activeCollisionMeshes += headpart.meshes().size();
				}
			}
		}
	}

	hdt::smpEcho("[HDT-SMP] tracked skeletons: %d", skeletons.size());
	hdt::smpEcho("[HDT-SMP] active skeletons: %d", activeSkeletons);
	hdt::smpEcho("[HDT-SMP] tracked armor addons: %d", armors);
	hdt::smpEcho("[HDT-SMP] tracked head parts: %d", headParts);
	hdt::smpEcho("[HDT-SMP] active armor addons: %d", activeArmors);
	hdt::smpEcho("[HDT-SMP] active head parts: %d", activeHeadParts);
	hdt::smpEcho("[HDT-SMP] active collision meshes: %d", activeCollisionMeshes);
	return true;
}

namespace
{
	void InitializeLog()
	{
#ifndef NDEBUG
		auto sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
#else
		auto path = logger::log_directory();
		if (!path) {
			util::report_and_fail("Failed to find standard logging directory"sv);
		}

		*path /= fmt::format("{}.log"sv, Plugin::NAME);
		auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
#endif

		auto log = std::make_shared<spdlog::logger>("global log"s, std::move(sink));
		log->set_level(spdlog::level::level_enum::info);
		log->flush_on(spdlog::level::level_enum::info);

		spdlog::set_default_logger(std::move(log));
		spdlog::set_pattern("[%H:%M:%S.%e] [%L] %v"s);
	}
}

void MessageHandler(SKSE::MessagingInterface::Message* a_msg)
{
	switch (a_msg->type) {
	case SKSE::MessagingInterface::kInputLoaded:
		Events::Register();
		break;
	case SKSE::MessagingInterface::kSaveGame:
		{
			auto data = hdt::Override::OverrideManager::GetSingleton()->Serialize();
			if (!data.str().empty()) {
				std::string save_name = reinterpret_cast<char*>(a_msg->data);
				std::ofstream ofs("Data/SKSE/Plugins/hdtOverrideSaves/" + save_name + ".dhdt", std::ios::out);
				if (ofs && ofs.is_open()) {
					ofs << data.str();
				}
			}
		}
		break;
	case SKSE::MessagingInterface::kPreLoadGame:
		{
			std::string save_name = reinterpret_cast<char*>(a_msg->data);
			save_name = save_name.substr(0, save_name.find_last_of("."));

			std::ifstream ifs("Data/SKSE/Plugins/hdtOverrideSaves/" + save_name + ".dhdt", std::ios::in);
			if (ifs && ifs.is_open()) {
				std::stringstream data;
				data << ifs.rdbuf();
				hdt::Override::OverrideManager::GetSingleton()->Deserialize(data);
			}
		}
		break;
	case SKSE::MessagingInterface::kPostPostLoad:
		{
			Hooks::InstallHighPriority();
			hdt::g_pluginInterface.onPostPostLoad();
			checkOldPlugins();
			// Register our config pages with the SKSE Menu Framework, if it is installed. No-ops otherwise.
			hdt::FSMPMenu::Register();
		}
		break;
	}
}

extern "C" DLLEXPORT bool SKSEAPI SKSEPlugin_Query(const SKSE::QueryInterface* a_skse, SKSE::PluginInfo* a_info)
{
	a_info->infoVersion = SKSE::PluginInfo::kVersion;
	a_info->name = Plugin::NAME.data();
	a_info->version = Plugin::VERSION.pack();

	if (a_skse->IsEditor()) {
		logger::critical("Loaded in editor, marking as incompatible"sv);
		return false;
	}

	const auto ver = a_skse->RuntimeVersion();
	if (REL::Module::IsSE() && ver < SKSE::RUNTIME_SSE_1_5_39 || REL::Module::IsVR() && ver < SKSE::RUNTIME_LATEST_VR) {
		logger::critical(FMT_STRING("Unsupported runtime version {}"), ver.string());
		return false;
	}

	return true;
}

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
#ifndef NDEBUG
	auto start = std::chrono::high_resolution_clock::now();

	while (!IsDebuggerPresent()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// break after 15 seconds of idle.
		if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(15)) {
			break;
		}
	}
#endif

	SKSE::Init(a_skse);

	InitializeLog();

	if constexpr (Plugin::BUILD_INFO.empty()) {
		logger::critical("{} v{} ({})"sv, Plugin::NAME, Plugin::VERSION.string(), Plugin::AVX_VARIANT);
	} else {
		logger::critical("{} v{}-{} ({})"sv, Plugin::NAME, Plugin::VERSION.string(), Plugin::BUILD_INFO, Plugin::AVX_VARIANT);
	}

	hdt::loadConfig();
	hdt::logConfig();

	const auto messaging = SKSE::GetMessagingInterface();
	if (!messaging->RegisterListener("SKSE", MessageHandler)) {
		return false;
	}

	//
	Events::Sources::FrameEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());
	Events::Sources::FrameEventSource::GetSingleton()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

	//
	Events::Sources::FrameSyncEventSource::GetSingleton()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

	//
	Events::Sources::ShutdownEventEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());
	Events::Sources::ShutdownEventEventSource::GetSingleton()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

	//
	Events::Sources::ArmorAttachEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

	//
	Events::Sources::ArmorDetachEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

	//
	Events::Sources::SkinSingleHeadGeometryEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

	//
	Events::Sources::SkinAllHeadGeometryEventSource::GetSingleton()->AddEventSink(hdt::ActorManager::instance());

	//
	SKSE::GetCameraEventSource()->AddEventSink(hdt::SkyrimPhysicsWorld::get());

	Hooks::InstallLowPriority();

	hdt::g_pluginInterface.init(a_skse);

	//
	auto unusedCommand = RE::SCRIPT_FUNCTION::LocateConsoleCommand("ShowRenderPasses");
	if (unusedCommand) {
		static RE::SCRIPT_PARAMETER params[3];
		params[0].paramType = RE::SCRIPT_PARAM_TYPE::kChar;
		params[0].paramName = "String (optional)";
		params[0].optional = 1;
		params[1].paramType = RE::SCRIPT_PARAM_TYPE::kChar;
		params[1].paramName = "String (optional)";
		params[1].optional = 1;
		params[2].paramType = RE::SCRIPT_PARAM_TYPE::kChar;
		params[2].paramName = "String (optional)";
		params[2].optional = 1;

		unusedCommand->functionName = "SMPDebug";
		unusedCommand->shortName = "smp";
		unusedCommand->helpString = "smp <help|reset|report [gear] [warnings]>";
		unusedCommand->referenceFunction = 0;
		unusedCommand->numParams = 3;
		unusedCommand->params = params;
		unusedCommand->executeFunction = SMPDebug_Execute;
		unusedCommand->editorFilter = 0;
	}

	//
	hdt::papyrus::RegisterAllFunctions(SKSE::GetPapyrusInterface());

	return true;
}
