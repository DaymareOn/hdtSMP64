#include "config.h"
#include "hdtAssetValidator.h"
#include "Hooks.h"
#include "XmlReader.h"
#include "Validator/hdtAssetValidator.h"
#include "hdtSkyrimPhysicsWorld.h"

namespace hdt
{
	int g_logLevel;

	static void solver(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag:
				if (reader.GetLocalName() == "numIterations") {
					SkyrimPhysicsWorld::get()->getSolverInfo().m_numIterations = btClamped(reader.readInt(), 4, 128);
				} else if (reader.GetLocalName() == "erp") {
					SkyrimPhysicsWorld::get()->getSolverInfo().m_erp = btClamped(reader.readFloat(), 0.01f, 1.0f);
				} else if (reader.GetLocalName() == "min-fps") {
					SkyrimPhysicsWorld::get()->min_fps = (btClamped(reader.readInt(), 1, 300));
					SkyrimPhysicsWorld::get()->m_timeTick = 1.0f / SkyrimPhysicsWorld::get()->min_fps;
				} else if (reader.GetLocalName() == "maxSubSteps") {
					SkyrimPhysicsWorld::get()->m_maxSubSteps = btClamped(reader.readInt(), 1, 60);
				} else {
					logger::warn("Unknown config : {}", reader.GetLocalName());
					reader.skipCurrentElement();
				}
				break;
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void wind(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag:
				if (reader.GetLocalName() == "windStrength") {
					SkyrimPhysicsWorld::get()->m_windStrength = btClamped(reader.readFloat(), 0.f, 1000.f);
				} else if (reader.GetLocalName() == "enabled") {
					SkyrimPhysicsWorld::get()->m_enableWind = reader.readBool();
				} else if (reader.GetLocalName() == "distanceForNoWind") {
					SkyrimPhysicsWorld::get()->m_distanceForNoWind = btClamped(reader.readFloat(), 0.f, 10000.f);
				} else if (reader.GetLocalName() == "distanceForMaxWind") {
					SkyrimPhysicsWorld::get()->m_distanceForMaxWind = btClamped(reader.readFloat(), 0.f, 10000.f);
				} else {
					logger::warn("Unknown config : {}", reader.GetLocalName());
					reader.skipCurrentElement();
				}
				break;
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void smp(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag:
				if (reader.GetLocalName() == "logLevel") {
					// Inverted so: 0 = critical, 1 = err, 2 = warn, 3 = info, 4 = debug, 5 = trace.
					g_logLevel = 5 - std::clamp(reader.readInt(), 0, 5);
					spdlog::set_level(static_cast<spdlog::level::level_enum>(g_logLevel));
					spdlog::flush_on(static_cast<spdlog::level::level_enum>(g_logLevel));
				} else if (reader.GetLocalName() == "backupNodeByName") {
					// Parse the string return value from reader.readText(); so we can have single strings instead of the group, example text -> "Virtual Hands, Virtual Body, Virtual Belly"... said text in a array like so -> { "Virtual Hands", "Virtual Body", "Virtual Belly"

					std::stringstream ss(reader.readText());
					std::string item;

					while (std::getline(ss, item, ',')) {
						// Remove leading space
						if (!item.empty() && item[0] == ' ') {
							item.erase(0, 1);
						}

						Hooks::BipedAnimHooks::BackupNodes.push_back(item);
					}
				} else if (reader.GetLocalName() == "enableNPCFaceParts") {
					ActorManager::instance()->m_skinNPCFaceParts = reader.readBool();
				} else if (reader.GetLocalName() == "disableSMPHairWhenWigEquipped") {
					ActorManager::instance()->m_disableSMPHairWhenWigEquipped = reader.readBool();
				} else if (reader.GetLocalName() == "clampRotations") {
					SkyrimPhysicsWorld::get()->m_clampRotations = reader.readBool();
				} else if (reader.GetLocalName() == "rotationSpeedLimit") {
					SkyrimPhysicsWorld::get()->m_rotationSpeedLimit = reader.readFloat();
				} else if (reader.GetLocalName() == "unclampedResets") {
					SkyrimPhysicsWorld::get()->m_unclampedResets = reader.readBool();
				} else if (reader.GetLocalName() == "unclampedResetAngle") {
					SkyrimPhysicsWorld::get()->m_unclampedResetAngle = reader.readFloat();
				} else if (reader.GetLocalName() == "budgetMs") {
					SkyrimPhysicsWorld::get()->m_budgetMs = std::clamp(reader.readFloat(), 0.1f, 20.0f);
				} else if (reader.GetLocalName() == "useRealTime") {
					SkyrimPhysicsWorld::get()->m_useRealTime = reader.readBool();
				} else if (reader.GetLocalName() == "minCullingDistance") {
					ActorManager::instance()->m_minCullingDistance = reader.readFloat();
				} else if (reader.GetLocalName() == "maximumActiveSkeletons") {
					ActorManager::instance()->m_maxActiveSkeletons = reader.readInt();
				} else if (reader.GetLocalName() == "autoAdjustMaxSkeletons") {
					ActorManager::instance()->m_autoAdjustMaxSkeletons = reader.readBool();
				} else if (reader.GetLocalName() == "sampleSize") {
					SkyrimPhysicsWorld::get()->m_sampleSize = std::max(reader.readInt(), 1);
				} else if (reader.GetLocalName() == "disable1stPersonViewPhysics") {
					ActorManager::instance()->m_disable1stPersonViewPhysics = reader.readBool();
				} else {
					logger::warn("Unknown config : {}", reader.GetLocalName());
					reader.skipCurrentElement();
				}
				break;
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void validation(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag:
				if (reader.GetLocalName() == "warn-triangle-count") {
					g_validationConfig.warnTriangleCount = reader.readInt();
				} else if (reader.GetLocalName() == "mods-dir") {
					g_validationConfig.modsDir = reader.readText();
					// Derive outputDir = modsDir/FSMP-out automatically.
					if (!g_validationConfig.modsDir.empty()) {
						namespace fs = std::filesystem;
						g_validationConfig.outputDir =
							(fs::path(g_validationConfig.modsDir) / "FSMP-out").string();
					}
				} else if (reader.GetLocalName() == "output-dir") {
					// Legacy tag — accepted for backward compat but modsDir stays empty.
					g_validationConfig.outputDir = reader.readText();
				} else if (reader.GetLocalName() == "decimate-collision-meshes-offline") {
					g_validationConfig.decimateCollisionMeshesOffline = reader.readBool();
				} else if (reader.GetLocalName() == "decimation-target-vertex-ratio") {
					g_validationConfig.decimationTargetVertexRatio = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-target-vertex-count") {
					g_validationConfig.decimationTargetVertexCount = reader.readInt();
				} else if (reader.GetLocalName() == "decimation-qem-cost-threshold") {
					g_validationConfig.decimationQemCostThreshold = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-short-edge-ratio") {
					g_validationConfig.decimationShortEdgeRatio = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-skin-weight-penalty") {
					g_validationConfig.decimationSkinWeightPenalty = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-max-skin-weight-drift") {
					g_validationConfig.decimationMaxSkinWeightDrift = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-max-volume-loss-percent") {
					g_validationConfig.decimationMaxVolumeLossPercent = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-max-local-volume-change-percent") {
					g_validationConfig.decimationMaxLocalVolumeChangePercent = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-max-normal-deviation-degrees") {
					g_validationConfig.decimationMaxNormalDeviationDegrees = reader.readFloat();
				} else if (reader.GetLocalName() == "decimation-max-point-removals") {
					g_validationConfig.decimationMaxPointRemovals = reader.readInt();
				} else if (reader.GetLocalName() == "decimation-max-edge-collapses") {
					g_validationConfig.decimationMaxEdgeCollapses = reader.readInt();
				} else if (reader.GetLocalName() == "decimation-preserve-boundary") {
					g_validationConfig.decimationPreserveBoundary = reader.readBool();
				} else if (reader.GetLocalName() == "decimation-preserve-features") {
					g_validationConfig.decimationPreserveFeatures = reader.readBool();
				} else {
					logger::warn("Unknown config : {}", reader.GetLocalName());
					reader.skipCurrentElement();
				}
				break;
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	static void config(XMLReader& reader)
	{
		while (reader.Inspect()) {
			switch (reader.GetInspected()) {
			case XMLReader::Inspected::StartTag:
				if (reader.GetLocalName() == "solver") {
					solver(reader);
				} else if (reader.GetLocalName() == "wind") {
					wind(reader);
				} else if (reader.GetLocalName() == "smp") {
					smp(reader);
				} else if (reader.GetLocalName() == "validation") {
					validation(reader);
				} else {
					logger::warn("Unknown config : {}", reader.GetLocalName());
					reader.skipCurrentElement();
				}
				break;
			case XMLReader::Inspected::EndTag:
				return;
			}
		}
	}

	void loadConfig()
	{
		auto bytes = readAllFile2("data/skse/plugins/hdtSkinnedMeshConfigs/configs.xml");
		if (bytes.empty()) {
			return;
		}

		// Store original locale
		char saved_locale[32];
		strcpy_s(saved_locale, std::setlocale(LC_NUMERIC, nullptr));

		// Set locale to en_US
		std::setlocale(LC_NUMERIC, "en_US");

		XMLReader reader((uint8_t*)bytes.data(), bytes.size());

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
				if (reader.GetLocalName() == "configs") {
					config(reader);
				} else {
					logger::warn("Unknown config : {}", reader.GetLocalName());
					reader.skipCurrentElement();
				}
			}
		}

		// Restore original locale
		std::setlocale(LC_NUMERIC, saved_locale);
	}

	void logConfig()
	{
		auto* w = SkyrimPhysicsWorld::get();
		auto* a = ActorManager::instance();

#define LOG(name, val) logger::debug("config: " name " = {}", val)
		LOG("solver.numIterations", w->getSolverInfo().m_numIterations);
		LOG("solver.erp", w->getSolverInfo().m_erp);
		LOG("solver.min-fps", w->min_fps);
		LOG("solver.maxSubSteps", w->m_maxSubSteps);

		LOG("wind.windStrength", w->m_windStrength);
		LOG("wind.enabled", w->m_enableWind);
		LOG("wind.distanceForNoWind", w->m_distanceForNoWind);
		LOG("wind.distanceForMaxWind", w->m_distanceForMaxWind);

		LOG("smp.logLevel", 5 - g_logLevel);

		for (auto& item : Hooks::BipedAnimHooks::BackupNodes)
			logger::debug("config: smp.backupNodeByName += {}", item);

		LOG("smp.enableNPCFaceParts", a->m_skinNPCFaceParts);
		LOG("smp.disableSMPHairWhenWigEquipped", a->m_disableSMPHairWhenWigEquipped);
		LOG("smp.clampRotations", w->m_clampRotations);
		LOG("smp.rotationSpeedLimit", w->m_rotationSpeedLimit);
		LOG("smp.unclampedResets", w->m_unclampedResets);
		LOG("smp.unclampedResetAngle", w->m_unclampedResetAngle);
		LOG("smp.budgetMS", w->m_budgetMs);
		LOG("smp.useRealTime", w->m_useRealTime);
		LOG("smp.minCullingDistance", a->m_minCullingDistance);
		LOG("smp.maximumActiveSkeletons", a->m_maxActiveSkeletons);
		LOG("smp.autoAdjustMaxSkeletons", a->m_autoAdjustMaxSkeletons);
		LOG("smp.sampleSize", w->m_sampleSize);
		LOG("smp.disable1stPersonViewPhysics", a->m_disable1stPersonViewPhysics);
#undef LOG
	}
}
