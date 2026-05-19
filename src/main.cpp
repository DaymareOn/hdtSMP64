#include "ActorManager.h"
#include "Events.h"
#include "Hooks.h"
#include "PluginInterfaceImpl.h"
#include "WeatherManager.h"
#include "config.h"
#include "dhdtOverrideManager.h"
#include "dhdtPapyrusFunctions.h"
#include "Validator/hdtAssetValidator.h"
#include "hdtSkyrimPhysicsWorld.h"

#include <atomic>
#include <charconv>
#include <cstdint>
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

	struct FixCommandArgs {
		bool gearOnly = false;
		bool copyOriginal = false;
		bool stateless = false;
		std::string outputDir;
	};

	bool ParseFixCommandArgs(const char* arg1, const char* arg2, const char* arg3, FixCommandArgs& outArgs) {
		auto parseArg = [&](const char* arg) {
			if (arg[0] == '\0')
				return true;
			if (_stricmp(arg, "gear") == 0 && !outArgs.gearOnly) {
				outArgs.gearOnly = true;
				return true;
			}
			if (_stricmp(arg, "debug") == 0 && !outArgs.copyOriginal) {
				outArgs.copyOriginal = true;
				return true;
			}
			if (outArgs.outputDir.empty()) {
				outArgs.outputDir = arg;
				return true;
			}
			return false;
		};
		return parseArg(arg1) && parseArg(arg2) && parseArg(arg3);
	}

	bool ParseFixXmlCommandArgs(const char* arg1, const char* arg2, const char* arg3, const char* arg4, FixCommandArgs& outArgs) {
		auto parseArg = [&](const char* arg) {
			if (arg[0] == '\0')
				return true;
			if (_stricmp(arg, "gear") == 0 && !outArgs.gearOnly) {
				outArgs.gearOnly = true;
				return true;
			}
			if (_stricmp(arg, "debug") == 0 && !outArgs.copyOriginal) {
				outArgs.copyOriginal = true;
				return true;
			}
			if (_stricmp(arg, "stateless") == 0 && !outArgs.stateless) {
				outArgs.stateless = true;
				return true;
			}
			if (outArgs.outputDir.empty()) {
				outArgs.outputDir = arg;
				return true;
			}
			return false;
		};

		return parseArg(arg1) && parseArg(arg2) && parseArg(arg3) && parseArg(arg4);
	}

	// Resolves the mods directory (from argument or config) and derives the
	// FSMP-out output path.  Returns false if no mods dir is available.
	bool ValidateFixOutputDir(std::string& outputDir, const char* usageHint) {
		// If a directory was given on the command line, check whether it is
		// already the output dir (old usage) or the mods dir (new usage).
		// New usage: mods dir → FSMP-out is created inside it.
		if (!outputDir.empty()) {
			namespace fs = std::filesystem;
			fs::path p(outputDir);
			// If the path ends in FSMP-out the user typed the old output path — accept as-is.
			if (p.filename() != "FSMP-out")
				outputDir = (p / "FSMP-out").string();
			return true;
		}
		// Fall back to config.
		if (!hdt::g_validationConfig.outputDir.empty()) {
			outputDir = hdt::g_validationConfig.outputDir;
			return true;
		}
		RE::ConsoleLog::GetSingleton()->Print(
			"[HDT-SMP] Mods directory not set. Usage: %s or set <validation><mods-dir> in config.",
			usageHint);
		return false;
	}

	static std::atomic<bool> s_validationRunning{ false };
	static std::map<std::string, std::atomic<bool>> s_fixRunning;

	void ExecuteFixThread(
		const std::string& typeStr,
		bool gearOnly,
		bool copyOriginal,
		std::string outputDir,
		std::function<void(bool, bool, const std::string&)> executeWork) {
		if (s_fixRunning[typeStr].exchange(true)) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] %s cleanup is already running.", typeStr.c_str());
			return;
		}
		if (s_validationRunning.load()) {
			s_fixRunning[typeStr].store(false);
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Cannot start %s cleanup while a report is running.", typeStr.c_str());
			return;
		}

		const char* startMessage = gearOnly ?
		                               "[HDT-SMP] Equipped gear %s cleanup started in background. Results will appear when complete. Output directory: %s (copy originals: %s)" :
		                               "[HDT-SMP] %s cleanup started in background. Results will appear when complete. Output directory: %s (copy originals: %s)";
		RE::ConsoleLog::GetSingleton()->Print(startMessage, typeStr.c_str(), outputDir.c_str(), copyOriginal ? "on" : "off");

		std::thread([typeStr, gearOnly, copyOriginal, outputDir = std::move(outputDir), executeWork]() {
			try {
				executeWork(gearOnly, copyOriginal, outputDir);
			} catch (const std::exception& e) {
				RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] %s cleanup failed with error: %s", typeStr.c_str(), e.what());
				logger::error("[Validator] smp fix {} threw: {}", typeStr, e.what());
			} catch (...) {
				RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] %s cleanup failed with an unknown error", typeStr.c_str());
				logger::error("[Validator] smp fix {} threw an unknown exception", typeStr);
			}
			s_fixRunning[typeStr].store(false);
		}).detach();
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

RE::NiSourceTexturePtr* GetTextureFromIndex(RE::BSLightingShaderMaterial* material, std::uint32_t index)
{
	switch (index) {
	case RE::BSTextureSet::Texture::kDiffuse:
		return std::addressof(material->diffuseTexture);
	case RE::BSTextureSet::Texture::kNormal:
		return std::addressof(material->normalTexture);
	case RE::BSTextureSet::Texture::kEnvironmentMask:
		{
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kFaceGen) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialFacegen*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->subsurfaceTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kGlowMap) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialFacegen*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->subsurfaceTexture);
			}
			return std::addressof(material->rimSoftLightingTexture);
		}
		break;
	case RE::BSTextureSet::Texture::kGlowMap:
		{
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kFaceGen) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialFacegen*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->detailTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kParallax) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialParallax*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->heightTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kParallax || material->GetFeature() == RE::BSShaderMaterial::Feature::kParallaxOcc) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialParallaxOcc*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->heightTexture);
			}
		}
		break;
	case RE::BSTextureSet::Texture::kHeight:
		{
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEye) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialEye*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->envTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEnvironmentMap) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialEnvmap*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->envTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kMultilayerParallax) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialMultiLayerParallax*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->envTexture);
			}
		}
		break;
	case RE::BSTextureSet::Texture::kEnvironment:
		{
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEye) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialEye*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->envMaskTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kEnvironmentMap) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialEnvmap*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->envTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kMultilayerParallax) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialMultiLayerParallax*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->envMaskTexture);
			}
		}
		break;
	case RE::BSTextureSet::Texture::kMultilayer:
		{
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kFaceGen) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialFacegen*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->tintTexture);
			}
			if (material->GetFeature() == RE::BSShaderMaterial::Feature::kMultilayerParallax) {
				return std::addressof(static_cast<RE::BSLightingShaderMaterialMultiLayerParallax*>(static_cast<RE::BSLightingShaderMaterialBase*>(material))->layerTexture);
			}
		}
		break;
	case RE::BSTextureSet::Texture::kBacklightMask:
		return std::addressof(material->specularBackLightingTexture);
		break;
	}

	return nullptr;
}

void DumpNodeChildren(RE::NiAVObject* node)
{
	logger::info(
		"{} {} [{:.2f}, {:.2f}, {:.2f}]",
		node->GetRTTI()->name,
		node->name,
		node->world.translate.x,
		node->world.translate.y,
		node->world.translate.z);

	if (node->extraDataSize > 0) {
		for (uint16_t i = 0; i < node->extraDataSize; i++) {
			logger::info(
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
							for (uint32_t boneIdx = 0; boneIdx < geometry->GetGeometryRuntimeData().skinInstance->skinData->bones; boneIdx++) {
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

								for (int texIdx = 0; texIdx < RE::BSTextureSet::Textures::kTotal; ++texIdx) {
									RE::BSTextureSet::Textures::Texture textureID = static_cast<RE::BSTextureSet::Textures::Texture>(texIdx);

									const char* texturePath = material->textureSet->GetTexturePath(textureID);
									if (!texturePath) {
										continue;
									}

									const char* textureName = "";
									RE::NiSourceTexturePtr* texture = GetTextureFromIndex(material, textureID);
									if (texture && texture->get()) {
										textureName = texture->get()->name.c_str();
									}

									logger::info(
										"Texture {} - {} ({})",
										texIdx,
										texturePath,
										textureName);
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

		RE::ConsoleLog::GetSingleton()->Print(
			"[HDT-SMP] %s skeleton - owner %s (refr formid %08x, base formid %08x) - %s",
			skeleton.state > hdt::ActorManager::SkeletonState::e_SkeletonActive ? "active" : "inactive",
			ownerName ? ownerName->GetFullName() : "unk_name",
			skelOwner ? skelOwner->formID : 0x00000000,
			skelOwner && skelOwner->GetBaseObject() ? skelOwner->GetBaseObject()->formID : 0x00000000,
			stateStrings[skeleton.state]);

		if (includeItems) {
			for (auto armor : skeleton.getArmors()) {
				RE::ConsoleLog::GetSingleton()->Print(
					"[HDT-SMP] -- tracked armor addon %s, %s",
					armor.armorWorn->name.c_str(),
					armor.state() != hdt::ActorManager::ItemState::e_NoPhysics ? armor.state() == hdt::ActorManager::ItemState::e_Active ? "has active physics system" : "has inactive physics system" : "has no physics system");

				if (armor.state() != hdt::ActorManager::ItemState::e_NoPhysics) {
					for (auto mesh : armor.meshes()) {
						RE::ConsoleLog::GetSingleton()->Print(
							"[HDT-SMP] ---- has collision mesh %s",
							mesh->m_name.c_str());
					}
				}
			}

			if (skeleton.head.headNode) {
				for (auto headPart : skeleton.head.headParts) {
					RE::ConsoleLog::GetSingleton()->Print(
						"[HDT-SMP] -- tracked headpart %s, %s",
						headPart.headPart->name.c_str(),
						headPart.state() != hdt::ActorManager::ItemState::e_NoPhysics ? headPart.state() == hdt::ActorManager::ItemState::e_Active ? "has active physics system" : "has inactive physics system" : "has no physics system");

					if (headPart.state() != hdt::ActorManager::ItemState::e_NoPhysics) {
						for (auto mesh : headPart.meshes()) {
							RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] ---- has collision mesh %s", mesh->m_name.c_str());
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
	char buffer4[MAX_PATH];
	memset(buffer4, 0, MAX_PATH);
	char buffer5[MAX_PATH];
	memset(buffer5, 0, MAX_PATH);
	char buffer6[MAX_PATH];
	memset(buffer6, 0, MAX_PATH);
	char buffer7[MAX_PATH];
	memset(buffer7, 0, MAX_PATH);
	char buffer8[MAX_PATH];
	memset(buffer8, 0, MAX_PATH);

	if (!RE::Script::ParseParameters(
			a_paramInfo, a_scriptData, a_opcodeOffsetPtr, a_thisObj, a_containingObj, a_scriptObj, a_locals,
			buffer, buffer2, buffer3, buffer4, buffer5, buffer6, buffer7, buffer8)) {
		return false;
	}

	logger::debug("SMPCommand: {} {} {} {} {} {} {} {}"sv,
		buffer, buffer2, buffer3, buffer4, buffer5, buffer6, buffer7, buffer8);

	auto printSmpHelp = []() {
		auto* console = RE::ConsoleLog::GetSingleton();
		console->Print("[HDT-SMP] Available smp commands:");
		console->Print("  smp help");
		console->Print("    Show this command reference.");
		console->Print("  smp reset");
		console->Print("    Reload SMP config and reset all active physics systems.");
		console->Print("  smp dumptree");
		console->Print("    Dump the targeted reference 3D node tree to console.");
		console->Print("  smp detail");
		console->Print("    Print detailed tracked skeleton/item diagnostics.");
		console->Print("  smp list");
		console->Print("    Print compact tracked skeleton summary.");
		console->Print("  smp profile [sample_frames] [print_every_frames]");
		console->Print("    Toggle physics profiler capture; defaults are 240/240.");
		console->Print("  smp on");
		console->Print("    Enable SMP simulation.");
		console->Print("  smp off");
		console->Print("    Disable SMP simulation.");
		console->Print("  smp QueryOverride");
		console->Print("    Print current dynamic override data.");
		console->Print("  smp report [gear] [error]");
		console->Print("    Run validator in background and write report file.");
		console->Print("    gear  = validate equipped gear only.");
		console->Print("    error = write errors-only report (no warnings/info)");
		console->Print("  smp fix xml [gear] [stateless] [debug] [output_dir]");
		console->Print("    Clean invalid XML tags and write improved XML copies.");
		console->Print("    stateless = write XML with explicit template dependencies so concrete node order does not matter.");
		console->Print("    debug = also copy original source as *-original.xml for improved files.");
		console->Print("  smp fix nif [gear] [debug] [output_dir]");
		console->Print("    Clean NIF issues and write improved NIF copies.");
		console->Print("    debug = also copy original source as *-original.nif for improved files.");
	};

	if (_strnicmp(buffer, "help", MAX_PATH) == 0) {
		printSmpHelp();
		return true;
	}

	if (_strnicmp(buffer, "reset", MAX_PATH) == 0) {
		logger::debug("smp reset: reloading config and resetting physics world"sv);
		RE::ConsoleLog::GetSingleton()->Print("running full smp reset");
		hdt::loadConfig();
		hdt::logConfig();

		const RE::MenuOpenCloseEvent e{ "", false };
		hdt::ActorManager::instance()->ProcessEvent(&e, nullptr);
		hdt::SkyrimPhysicsWorld::get()->resetSystems();
		return true;
	}
	if (_strnicmp(buffer, "dumptree", MAX_PATH) == 0) {
		if (a_thisObj) {
			RE::ConsoleLog::GetSingleton()->Print("dumping targeted reference's node tree");
			DumpNodeChildren(a_thisObj->Get3D1(0));
		} else {
			RE::ConsoleLog::GetSingleton()->Print("error: you must target a reference to dump their node tree");
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
			RE::ConsoleLog::GetSingleton()->Print(
				"HDT-SMP physics profiler enabled: sample %llu frames, print every %llu frames",
				static_cast<unsigned long long>(sampleFrames),
				static_cast<unsigned long long>(printFrames));
			RE::ConsoleLog::GetSingleton()->Print("Check your hdtsmp64.log file for results.");

		} else {
			RE::ConsoleLog::GetSingleton()->Print("HDT-SMP physics profiler disabled");
		}

		return true;
	}

	if (_strnicmp(buffer, "on", MAX_PATH) == 0) {
		hdt::SkyrimPhysicsWorld::get()->disabled = false;
		{
			RE::ConsoleLog::GetSingleton()->Print("HDT-SMP enabled");
		}
		return true;
	}

	if (_strnicmp(buffer, "off", MAX_PATH) == 0) {
		hdt::SkyrimPhysicsWorld::get()->disabled = true;
		{
			RE::ConsoleLog::GetSingleton()->Print("HDT-SMP disabled");
		}
		return true;
	}

	if (_strnicmp(buffer, "QueryOverride", MAX_PATH) == 0) {
		RE::ConsoleLog::GetSingleton()->Print(hdt::Override::OverrideManager::GetSingleton()->queryOverrideData().c_str());
		return true;
	}

	if (_strnicmp(buffer, "report", MAX_PATH) == 0) {
		bool gearOnly = false;
		bool errorReport = false;
		auto parseValidateModeArg = [&](const char* arg) {
			if (arg[0] == '\0')
				return true;
			if (_stricmp(arg, "gear") == 0) {
				gearOnly = true;
				return true;
			}
			if (_stricmp(arg, "error") == 0) {
				errorReport = true;
				return true;
			}
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Unknown report mode: %s", arg);
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Usage: smp report [gear] [error]");
			return false;
		};

		if (!parseValidateModeArg(buffer2) || !parseValidateModeArg(buffer3))
			return true;
		if (buffer4[0] != '\0') {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Usage: smp report [gear] [error]");
			return true;
		}

		if (s_validationRunning.exchange(true)) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Validation is already running.");
			return true;
		}
		for (auto& [key, flag] : s_fixRunning) {
			if (flag.load()) {
				s_validationRunning.store(false);
				RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Cannot start report while a fix command is running.");
				return true;
			}
		}
		if (gearOnly && errorReport) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Equipped gear report (error report) started in background. Results will appear when complete.");
		} else if (gearOnly) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Equipped gear report started in background. Results will appear when complete.");
		} else if (errorReport) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Report (error report) started in background. Results will appear when complete.");
		} else {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Report started in background. Results will appear when complete.");
		}
		std::thread([gearOnly, errorReport]() {
			try {
				const char* validationLabel = gearOnly ? "Equipped gear report" : "Report";
				std::string reportPath;
				auto result = hdt::ValidatePhysicsAssets(
					reportPath,
					gearOnly,
					errorReport ? hdt::ValidationReportMode::ErrorsOnly : hdt::ValidationReportMode::Full);
				auto* console = RE::ConsoleLog::GetSingleton();
				if (errorReport) {
					console->Print(
						"[HDT-SMP] %s complete in %.2fs: %d XML(s) found, %d failed (error report: errors only)",
						validationLabel,
						result.elapsedSeconds,
						result.totalXMLsFound,
						result.xmlErrorCount);
				} else {
					console->Print(
						"[HDT-SMP] %s complete in %.2fs: %d XML(s) found, %d passed, %d failed, %d warning(s)",
						validationLabel,
						result.elapsedSeconds,
						result.totalXMLsFound, result.xmlPassCount, result.xmlErrorCount,
						(int)result.warnings.size());
				}
				if (!reportPath.empty()) {
					if (errorReport) {
						console->Print("[HDT-SMP] Errors-only report written to: %s", reportPath.c_str());
					} else {
						console->Print("[HDT-SMP] Report written to: %s", reportPath.c_str());
					}
				} else {
					console->Print("[HDT-SMP] Warning: report file could not be written");
				}
			} catch (const std::exception& e) {
				RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Report failed with error: %s", e.what());
				logger::error("[Validator] smp report threw: {}", e.what());
			} catch (...) {
				RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Report failed with an unknown error");
				logger::error("[Validator] smp report threw an unknown exception");
			}
			s_validationRunning.store(false);
		}).detach();
		return true;
	}

	const bool isFixNIFSplitAlias = _strnicmp(buffer, "fix", MAX_PATH) == 0 && _stricmp(buffer2, "nif") == 0;
	if (isFixNIFSplitAlias) {
		const bool hasTooManyArgs = buffer6[0] != '\0';
		if (hasTooManyArgs) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Usage: smp fix nif [gear] [debug] [output_dir]");
			return true;
		}

		FixCommandArgs args;
		if (!ParseFixCommandArgs(buffer3, buffer4, buffer5, args)) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Usage: smp fix nif [gear] [debug] [output_dir]");
			return true;
		}

		if (!ValidateFixOutputDir(args.outputDir, "smp fix nif [gear] [debug] [output_dir]"))
			return true;

		ExecuteFixThread("NIF", args.gearOnly, args.copyOriginal, args.outputDir,
			[](bool gearOnly, bool copyOriginal, const std::string& outputDir) {
				auto result = hdt::ImprovePhysicsNIFs(outputDir, gearOnly, copyOriginal);
				auto* console = RE::ConsoleLog::GetSingleton();
				console->Print("[HDT-SMP] %s NIF cleanup: %d NIF(s) processed, %d related TRI(s), %d cleaned file(s) written to %s",
					gearOnly ? "Equipped gear" : "All",
					result.totalNIFsFound,
					result.totalTRIFilesFound,
					result.nifImprovedCount,
					outputDir.c_str());
				if (result.decimationCandidatesAttempted > 0) {
					console->Print("[HDT-SMP] NIF decimation bridge: discovered=%d attempted=%d applied=%d skipped-no-change=%d skipped-unsafe=%d",
						result.decimationCandidatesDiscovered,
						result.decimationCandidatesAttempted,
						result.decimationCandidatesApplied,
						result.decimationCandidatesSkippedNoChange,
						result.decimationCandidatesSkippedUnsafe);
					for (const auto& reason : result.decimationSkipReasonHistogram)
						console->Print("[HDT-SMP] NIF decimation bridge skip-reason: %s", reason.c_str());
				}
				for (const auto& err : result.errors) {
					console->Print("[HDT-SMP] NIF cleanup error: %s", err.c_str());
				}
				logger::info("[Validator] NIF cleanup done: gearOnly={}, nifs={}, tris={}, improved={}, decimation(discovered={},attempted={},applied={},skip-no-change={},skip-unsafe={}), output={}",
					gearOnly,
					result.totalNIFsFound,
					result.totalTRIFilesFound,
					result.nifImprovedCount,
					result.decimationCandidatesDiscovered,
					result.decimationCandidatesAttempted,
					result.decimationCandidatesApplied,
					result.decimationCandidatesSkippedNoChange,
					result.decimationCandidatesSkippedUnsafe,
					outputDir);
			});
		return true;
	}

	const bool isFixXMLSplitAlias = _strnicmp(buffer, "fix", MAX_PATH) == 0 && _stricmp(buffer2, "xml") == 0;
	if (isFixXMLSplitAlias) {
		const bool hasTooManyArgs = buffer7[0] != '\0';
		if (hasTooManyArgs) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Usage: smp fix xml [gear] [stateless] [debug] [output_dir]");
			return true;
		}

		FixCommandArgs args;
		if (!ParseFixXmlCommandArgs(buffer3, buffer4, buffer5, buffer6, args)) {
			RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] Usage: smp fix xml [gear] [stateless] [debug] [output_dir]");
			return true;
		}

		if (!ValidateFixOutputDir(args.outputDir, "smp fix xml [gear] [stateless] [debug] [output_dir]"))
			return true;

		const std::string modeLabel = args.stateless ? "XML (stateless)" : "XML";
		ExecuteFixThread(modeLabel, args.gearOnly, args.copyOriginal, args.outputDir,
			[stateless = args.stateless](bool gearOnly, bool copyOriginal, const std::string& outputDir) {
				auto result = hdt::ImprovePhysicsXMLs(outputDir, gearOnly, copyOriginal, stateless);
				auto* console = RE::ConsoleLog::GetSingleton();
				console->Print("[HDT-SMP] %s XML cleanup%s: %d XML(s) scanned, %d cleaned file(s) written to %s",
					gearOnly ? "Equipped gear" : "Full",
					stateless ? " (stateless)" : "",
					result.totalXMLsFound,
					result.xmlImprovedCount,
					outputDir.c_str());
				for (const auto& err : result.errors) {
					console->Print("[HDT-SMP] XML cleanup error: %s", err.c_str());
				}
				logger::info("[Validator] XML cleanup done: gearOnly={}, stateless={}, {} XML(s) scanned, {} improved, output={}",
					gearOnly, stateless, result.totalXMLsFound, result.xmlImprovedCount, outputDir);
			});
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

	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] tracked skeletons: %d", skeletons.size());
	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active skeletons: %d", activeSkeletons);
	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] tracked armor addons: %d", armors);
	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] tracked head parts: %d", headParts);
	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active armor addons: %d", activeArmors);
	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active head parts: %d", activeHeadParts);
	RE::ConsoleLog::GetSingleton()->Print("[HDT-SMP] active collision meshes: %d", activeCollisionMeshes);
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
			hdt::g_pluginInterface.onPostPostLoad();
			checkOldPlugins();
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

	Hooks::Install();

	hdt::g_pluginInterface.init(a_skse);

	//
	auto unusedCommand = RE::SCRIPT_FUNCTION::LocateConsoleCommand("ShowRenderPasses");
	if (unusedCommand) {
		static RE::SCRIPT_PARAMETER params[8];
		for (auto& param : params) {
			param.paramType = RE::SCRIPT_PARAM_TYPE::kChar;
			param.paramName = "String (optional)";
			param.optional = 1;
		}

		unusedCommand->functionName = "SMPDebug";
		unusedCommand->shortName = "smp";
		unusedCommand->helpString = "smp <help|reset|dumptree|detail|list|profile [sample_frames] [print_every_frames]|on|off|QueryOverride|report [gear] [error]|fix xml [gear] [stateless] [debug] [output_dir]|fix nif [gear] [debug] [output_dir]>";
		unusedCommand->referenceFunction = 0;
		unusedCommand->numParams = 8;
		unusedCommand->params = params;
		unusedCommand->executeFunction = SMPDebug_Execute;
		unusedCommand->editorFilter = 0;
	}

	//
	hdt::papyrus::RegisterAllFunctions(SKSE::GetPapyrusInterface());

	return true;
}
