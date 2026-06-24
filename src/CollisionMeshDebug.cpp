#include "CollisionMeshDebug.h"

#include "ActorManager.h"
#include "NetImmerseUtils.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace hdt::CollisionMeshDebug
{

	namespace
	{
		struct CollisionMeshTarget
		{
			RE::BSGeometry* geometry = nullptr;
			RE::BSLightingShaderProperty* shaderTemplate = nullptr;
		};

		struct GeometryDebugState
		{
			std::uint32_t actorFormID = 0;
			RE::NiAVObject::NiAVObjectFlags originalFlags{};
			RE::NiPointer<RE::BSShaderProperty> debugShaderProperty;
			bool debugEnabled = false;
		};

		struct CollisionMeshCollection
		{
			std::size_t systems = 0;
			std::unordered_map<RE::BSGeometry*, CollisionMeshTarget> geometries;
		};

		// Non-owning geometry keys. Only dereference them after rediscovering the
		// same geometry in the current actor collection..
		std::unordered_map<RE::BSGeometry*, GeometryDebugState> g_geometryStates;

		// Skyrim's renderer seems to keep shader pointers queued after we hide a debug
		// mesh. Keep retired debug shaders alive instead of releasing them from
		// under the render pipeline or it'll crash.
		// Todo: Figure out a clean solution to this
		std::vector<RE::NiPointer<RE::BSShaderProperty>> g_retiredDebugShaders;

		void retireState(std::unordered_map<RE::BSGeometry*, GeometryDebugState>::iterator stateIt)
		{
			if (stateIt->second.debugShaderProperty) {
				g_retiredDebugShaders.emplace_back(std::move(stateIt->second.debugShaderProperty));
			}
			g_geometryStates.erase(stateIt);
		}

		RE::BSLightingShaderProperty* findCollisionDebugShaderTemplate(RE::NiAVObject* model)
		{
			RE::BSLightingShaderProperty* result = nullptr;
			if (!model) {
				return result;
			}

			RE::BSVisit::TraverseScenegraphGeometries(model, [&](RE::BSGeometry* geometry) -> RE::BSVisit::BSVisitControl {
				if (!geometry) {
					return RE::BSVisit::BSVisitControl::kContinue;
				}

				auto* shaderProperty = geometry->GetGeometryRuntimeData().shaderProperty.get();
				auto* lightingShader = netimmerse_cast<RE::BSLightingShaderProperty*>(shaderProperty);
				if (lightingShader && lightingShader->material) {
					result = lightingShader;
					return RE::BSVisit::BSVisitControl::kStop;
				}

				return RE::BSVisit::BSVisitControl::kContinue;
			});

			return result;
		}

		RE::NiPointer<RE::BSShaderProperty> createCollisionDebugShader(RE::BSLightingShaderProperty* templateShader)
		{
			// Todo: Verify this is the correct malloc for renderer stuff
			auto* shader = RE::malloc<RE::BSLightingShaderProperty>();
			if (!shader) {
				return {};
			}

			std::memset(shader, 0, sizeof(RE::BSLightingShaderProperty));
			shader->Ctor();
			SKSE::stl::emplace_vtable<RE::BSLightingShaderProperty>(shader);

			RE::NiPointer<RE::BSLightingShaderProperty> result;
			result.reset(shader);

			RE::BSShaderMaterial* material = nullptr;
			if (templateShader && templateShader->material) {
				material = templateShader->material->Create();
				if (material) {
					material->CopyMembers(templateShader->material);
				}
			}

			if (!material) {
				material = RE::BSLightingShaderMaterialBase::CreateMaterial(RE::BSShaderMaterial::Feature::kDefault);
			}

			auto* graphicsState = RE::BSGraphics::State::GetSingleton();
			if (material && material->GetType() == RE::BSShaderMaterial::Type::kLighting) {
				auto* lightingMaterial = static_cast<RE::BSLightingShaderMaterialBase*>(material);

				// This is the alpha for, well the material. The physical mesh. Doesn't effect the wire frame
				lightingMaterial->materialAlpha = 0.1f;
				lightingMaterial->specularColor = RE::NiColor(0.0f, 1.0f, 0.85f);
				lightingMaterial->specularPower = 64.0f;
				lightingMaterial->specularColorScale = 2.0f;

				if (graphicsState) {
					const auto& runtimeData = graphicsState->GetRuntimeData();
					lightingMaterial->diffuseTexture = runtimeData.defaultTextureWhite;
					lightingMaterial->normalTexture = runtimeData.defaultTextureNormalMap;
				}
			}

			if (material) {
				result->SetMaterial(material, true);
			}

			if (templateShader) {
				result->flags = templateShader->flags;
			}

			using ShaderFlag = RE::BSShaderProperty::EShaderPropertyFlag;
			result->flags.set(ShaderFlag::kSkinned);
			result->flags.set(ShaderFlag::kTwoSided);
			result->flags.set(ShaderFlag::kWireframe);
			result->flags.set(ShaderFlag::kZBufferTest);
			result->flags.set(ShaderFlag::kNoFade);
			result->flags.reset(ShaderFlag::kCastShadows);
			result->flags.reset(ShaderFlag::kReceiveShadows);
			result->flags.reset(ShaderFlag::kRefraction);
			result->flags.reset(ShaderFlag::kTempRefraction);
			result->alpha = 1.0f;
			result->lastRenderPassState = (std::numeric_limits<std::int32_t>::max)();

			auto shaderData = RE::make_smart<RE::BSEffectShaderData>();
			shaderData->fillColor = RE::NiColorA(1.0f, 1.0f, 0.85f, 1.0f);
			shaderData->rimColor = RE::NiColorA(1.0f, 0.35f, 1.0f, 1.0f);
			if (graphicsState) {
				shaderData->baseTexture = graphicsState->GetRuntimeData().defaultTextureWhite;
			}
			result->SetEffectShaderData(shaderData);

			return result;
		}

		std::size_t disableState(std::unordered_map<RE::BSGeometry*, GeometryDebugState>::iterator stateIt, RE::BSGeometry* geometry)
		{
			auto& state = stateIt->second;
			if (!state.debugEnabled || !geometry) {
				return 0;
			}

			geometry->GetFlags() = state.originalFlags;
			geometry->GetFlags().set(RE::NiAVObject::Flag::kHidden);
			geometry->GetFlags().set(RE::NiAVObject::Flag::kNotVisible);
			geometry->GetFlags().reset(RE::NiAVObject::Flag::kAlwaysDraw);
			geometry->SetMaterialNeedsUpdate(true);

			state.debugEnabled = false;
			return 1;
		}

		std::size_t disableActorStates(std::uint32_t actorFormID, const std::unordered_map<RE::BSGeometry*, CollisionMeshTarget>& currentGeometries)
		{
			std::size_t affected = 0;
			for (auto it = g_geometryStates.begin(); it != g_geometryStates.end();) {
				if (it->second.actorFormID != actorFormID) {
					++it;
					continue;
				}

				if (currentGeometries.contains(it->first)) {
					affected += disableState(it, it->first);
					++it;
				} else {
					const auto current = it++;
					retireState(current);
				}
			}

			return affected;
		}

		void pruneActorStatesExcept(std::uint32_t actorFormID, const std::unordered_map<RE::BSGeometry*, CollisionMeshTarget>& currentGeometries)
		{
			for (auto it = g_geometryStates.begin(); it != g_geometryStates.end();) {
				if (it->second.actorFormID != actorFormID || currentGeometries.contains(it->first)) {
					++it;
					continue;
				}

				const auto current = it++;
				retireState(current);
			}
		}

		bool hasActorStates(std::uint32_t actorFormID)
		{
			return std::any_of(g_geometryStates.begin(), g_geometryStates.end(), [actorFormID](const auto& entry) {
				return entry.second.actorFormID == actorFormID && entry.second.debugEnabled;
			});
		}

		std::size_t enableGeometry(std::uint32_t actorFormID, const CollisionMeshTarget& target)
		{
			auto* geometry = target.geometry;
			if (!geometry) {
				return 0;
			}

			auto& runtimeData = geometry->GetGeometryRuntimeData();
			if (auto existing = g_geometryStates.find(geometry); existing != g_geometryStates.end()) {
				if (existing->second.actorFormID != actorFormID) {
					retireState(existing);
				} else {
					auto& state = existing->second;
					if (runtimeData.shaderProperty != state.debugShaderProperty) {
						if (runtimeData.shaderProperty) {
							logger::debug("Skipping collision mesh visualization for {} because it already has a shader property", geometry->name.c_str());
							return 0;
						}

						runtimeData.shaderProperty = state.debugShaderProperty;
						state.debugShaderProperty->SetupGeometry(geometry);
						state.debugShaderProperty->FinishSetupGeometry(geometry);
					}

					geometry->GetFlags().reset(RE::NiAVObject::Flag::kHidden);
					geometry->GetFlags().reset(RE::NiAVObject::Flag::kNotVisible);
					geometry->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
					geometry->SetMaterialNeedsUpdate(true);
					state.debugEnabled = true;
					return 1;
				}
			}

			if (runtimeData.shaderProperty) {
				logger::debug("Skipping collision mesh visualization for {} because it already has a shader property", geometry->name.c_str());
				return 0;
			}

			auto debugShader = createCollisionDebugShader(target.shaderTemplate);
			if (!debugShader) {
				logger::warn("Failed to create collision mesh debug shader for {}", geometry->name.c_str());
				return 0;
			}

			GeometryDebugState state;
			state.actorFormID = actorFormID;
			state.originalFlags = geometry->GetFlags();
			state.debugShaderProperty = debugShader;
			state.debugEnabled = true;

			runtimeData.shaderProperty = state.debugShaderProperty;
			geometry->GetFlags().reset(RE::NiAVObject::Flag::kHidden);
			geometry->GetFlags().reset(RE::NiAVObject::Flag::kNotVisible);
			geometry->GetFlags().set(RE::NiAVObject::Flag::kAlwaysDraw);
			geometry->SetMaterialNeedsUpdate(true);
			state.debugShaderProperty->SetupGeometry(geometry);
			state.debugShaderProperty->FinishSetupGeometry(geometry);

			g_geometryStates.emplace(geometry, std::move(state));
			return 1;
		}

		void collectItemGeometries(const auto& item, RE::NiAVObject* model, CollisionMeshCollection& collection)
		{
			if (item.state() == ActorManager::ItemState::e_NoPhysics || !model) {
				return;
			}

			++collection.systems;
			const auto shaderTemplate = findCollisionDebugShaderTemplate(model);
			for (const auto& [xmlName, meshNames] : item.physicsFile.second) {
				(void)xmlName;

				for (const auto& meshName : meshNames) {
					auto* geometry = castBSTriShape(findObject(model, meshName.c_str()));
					if (!geometry || !geometry->GetGeometryRuntimeData().skinInstance) {
						continue;
					}

					if (collection.geometries.contains(geometry)) {
						continue;
					}

					CollisionMeshTarget target;
					target.geometry = geometry;
					target.shaderTemplate = shaderTemplate;
					collection.geometries.emplace(geometry, std::move(target));
				}
			}
		}

		bool skeletonBelongsToActor(const auto& skeleton, const RE::Actor& actor)
		{
			auto* owner = skeleton.skeletonOwner.get();
			if (!owner && skeleton.skeleton) {
				owner = skeleton.skeleton->GetUserData();
			}

			return owner && (owner == std::addressof(actor) || owner->formID == actor.formID);
		}

		CollisionMeshCollection collectActorCollisionMeshes(const RE::Actor& actor)
		{
			CollisionMeshCollection collection;
			auto& skeletons = ActorManager::instance()->getSkeletons();

			for (auto& skeleton : skeletons) {
				if (!skeletonBelongsToActor(skeleton, actor)) {
					continue;
				}

				for (const auto& armor : skeleton.getArmors()) {
					collectItemGeometries(armor, armor.armorWorn.get(), collection);
				}

				if (skeleton.head.headNode) {
					for (const auto& headPart : skeleton.head.headParts) {
						collectItemGeometries(headPart, skeleton.head.headNode.get(), collection);
					}
				}
			}

			return collection;
		}
	}

	ApplyResult applyToSelectedActor(std::optional<bool> requestedEnabled, RE::TESObjectREFR* commandTarget)
	{
		ApplyResult result;

		RE::NiPointer<RE::TESObjectREFR> consoleTarget;
		auto* target = commandTarget;
		if (!target) {
			consoleTarget = RE::Console::GetSelectedRef();
			target = consoleTarget.get();
		}

		if (!target) {
			result.status = ApplyStatus::NoSelectedReference;
			return result;
		}

		auto* actor = target->As<RE::Actor>();
		if (!actor) {
			result.status = ApplyStatus::SelectedReferenceIsNotActor;
			return result;
		}

		result.actorFormID = actor->formID;
		const auto lock = ActorManager::instance()->lockGuard();
		const auto collection = collectActorCollisionMeshes(*actor);
		result.systems = collection.systems;
		pruneActorStatesExcept(actor->formID, collection.geometries);

		const auto hasEnabledStates = hasActorStates(actor->formID);
		result.enabled = requestedEnabled.value_or(!hasEnabledStates);

		if (result.enabled) {
			for (const auto& [geometry, targetMesh] : collection.geometries) {
				(void)geometry;
				result.geometries += enableGeometry(actor->formID, targetMesh);
			}
		} else {
			result.geometries = disableActorStates(actor->formID, collection.geometries);
		}

		if (!result.systems && !result.geometries) {
			result.status = ApplyStatus::NoTrackedPhysics;
		} else if (collection.geometries.empty() && !result.geometries) {
			result.status = ApplyStatus::NoCollisionMeshes;
		}

		return result;
	}

	void disableAll()
	{
		while (!g_geometryStates.empty()) {
			retireState(g_geometryStates.begin());
		}
	}
}
