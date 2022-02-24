#include "skse64/GameMenus.h"
#include "skse64/GameReferences.h"
#include "skse64/ObScript.h"
#include "skse64/PluginAPI.h"
#include "skse64_common/skse_version.h"
#include "skse64_common/SafeWrite.h"

#include "ActorManager.h"
#include "config.h"
#include "EventDebugLogger.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "Hooks.h"
#include "HookEvents.h"

#include <numeric>

#include <shlobj_core.h>
#include "skse64/GameRTTI.h"
#include "skse64_common/BranchTrampoline.h"

#ifdef CUDA
#include "hdtSkinnedMesh/hdtCudaInterface.h"
#include "hdtSkinnedMesh/hdtFrameTimer.h"
#endif

namespace hdt
{
	IDebugLog gLog;
	EventDebugLogger g_eventDebugLogger;
	PluginHandle g_PluginHandle;

	class FreezeEventHandler : public BSTEventSink<MenuOpenCloseEvent>
	{
	public:
		FreezeEventHandler()
		{
		}

		EventResult ReceiveEvent(MenuOpenCloseEvent* evn, EventDispatcher<MenuOpenCloseEvent>* dispatcher) override
		{
			auto mm = MenuManager::GetSingleton();

			if (evn && evn->opening && (!strcmp(evn->menuName.data, "Loading Menu") || !strcmp(
				evn->menuName.data, "RaceSex Menu")))
			{
				_DMESSAGE("loading menu/racesexmenu detected, scheduling physics reset on world un-suspend");
				SkyrimPhysicsWorld::get()->suspend(true);
			}

			if (evn && !evn->opening && !strcmp(evn->menuName.data, "RaceSex Menu"))
			{
				_DMESSAGE("racemenu closed, reloading meshes");
				ActorManager::instance()->onEvent(*evn);
			}

			return kEvent_Continue;
		}
	} g_freezeEventHandler;

	void checkOldPlugins()
	{
		auto framework = GetModuleHandleA("hdtSSEFramework");
		auto physics = GetModuleHandleA("hdtSSEPhysics");
		auto hh = GetModuleHandleA("hdtSSEHighHeels");

		if (physics)
		{
			MessageBox(nullptr, TEXT(
				"hdtSSEPhysics.dll is loaded. This is an older verson of HDT-SMP and conflicts with hdtSMP64.dll. Please remove it."),
				TEXT("hdtSMP64"), MB_OK);
		}

		if (framework && !hh)
		{
			MessageBox(nullptr, TEXT(
				"hdtSSEFramework.dll is loaded but hdtSSEHighHeels.dll is not being used. You no longer need hdtSSEFramework.dll with this version of SMP. Please remove it."),
				TEXT("hdtSMP64"), MB_OK);
		}
	}

	NiTexturePtr* GetTextureFromIndex(BSLightingShaderMaterial* material, UInt32 index)
	{
		switch (index)
		{
		case 0:
			return &material->texture1;
			break;
		case 1:
			return &material->texture2;
			break;
		case 2:
		{
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_FaceGen)
			{
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->unkB0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_GlowMap)
			{
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->unkB0;
			}
			return &material->texture3;
		}
		break;
		case 3:
		{
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_FaceGen)
			{
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->unkA8;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Parallax)
			{
				return &static_cast<BSLightingShaderMaterialParallax*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Parallax || material->GetShaderType() ==
				BSShaderMaterial::kShaderType_ParallaxOcc)
			{
				return &static_cast<BSLightingShaderMaterialParallaxOcc*>(material)->unkA0;
			}
		}
		break;
		case 4:
		{
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Eye)
			{
				return &static_cast<BSLightingShaderMaterialEye*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_EnvironmentMap)
			{
				return &static_cast<BSLightingShaderMaterialEnvmap*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_MultilayerParallax)
			{
				return &static_cast<BSLightingShaderMaterialMultiLayerParallax*>(material)->unkA8;
			}
		}
		break;
		case 5:
		{
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_Eye)
			{
				return &static_cast<BSLightingShaderMaterialEye*>(material)->unkA8;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_EnvironmentMap)
			{
				return &static_cast<BSLightingShaderMaterialEnvmap*>(material)->unkA0;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_MultilayerParallax)
			{
				return &static_cast<BSLightingShaderMaterialMultiLayerParallax*>(material)->unkB0;
			}
		}
		break;
		case 6:
		{
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_FaceGen)
			{
				return &static_cast<BSLightingShaderMaterialFacegen*>(material)->renderedTexture;
			}
			if (material->GetShaderType() == BSShaderMaterial::kShaderType_MultilayerParallax)
			{
				return &static_cast<BSLightingShaderMaterialMultiLayerParallax*>(material)->unkA0;
			}
		}
		break;
		case 7:
			return &material->texture4;
			break;
		}

		return nullptr;
	}

	void DumpNodeChildren(NiAVObject* node)
	{
		_MESSAGE("{%s} {%s} {%X} [%f, %f, %f]", node->GetRTTI()->name, node->m_name, node, node->m_worldTransform.pos.x, node->m_worldTransform.pos.y, node->m_worldTransform.pos.z);
		if (node->m_extraDataLen > 0)
		{
			gLog.Indent();
			for (UInt16 i = 0; i < node->m_extraDataLen; i++)
			{
				_MESSAGE("{%s} {%s} {%X}", node->m_extraData[i]->GetRTTI()->name, node->m_extraData[i]->m_pcName, node);
			}
			gLog.Outdent();
		}

		NiNode* niNode = node->GetAsNiNode();
		if (niNode && niNode->m_children.m_emptyRunStart > 0)
		{
			gLog.Indent();
			for (int i = 0; i < niNode->m_children.m_emptyRunStart; i++)
			{
				NiAVObject* object = niNode->m_children.m_data[i];
				if (object)
				{
					NiNode* childNode = object->GetAsNiNode();
					BSGeometry* geometry = object->GetAsBSGeometry();
					if (geometry)
					{
						_MESSAGE("{%s} {%s} {%X} [%f, %f, %f] - Geometry", object->GetRTTI()->name, object->m_name, object, geometry->m_worldTransform.pos.x, geometry->m_worldTransform.pos.y, geometry->m_worldTransform.pos.z);
						if (geometry->m_spSkinInstance && geometry->m_spSkinInstance->m_spSkinData)
						{
							gLog.Indent();
							for (int i = 0; i < geometry->m_spSkinInstance->m_spSkinData->m_uiBones; i++)
							{
								auto bone = geometry->m_spSkinInstance->m_ppkBones[i];
								_MESSAGE("Bone %d - {%s} {%s} {%X} [%f, %f, %f]", i, bone->GetRTTI()->name, bone->m_name, bone, bone->m_worldTransform.pos.x, bone->m_worldTransform.pos.y, bone->m_worldTransform.pos.z);
							}
							gLog.Outdent();
						}
						NiPointer<BSShaderProperty> shaderProperty = niptr_cast<BSShaderProperty>(
							geometry->m_spEffectState);
						if (shaderProperty)
						{
							BSLightingShaderProperty* lightingShader =
								ni_cast(shaderProperty, BSLightingShaderProperty);
							if (lightingShader)
							{
								BSLightingShaderMaterial* material = static_cast<BSLightingShaderMaterial*>(
									lightingShader->material);

								gLog.Indent();
								for (int i = 0; i < BSTextureSet::kNumTextures; ++i)
								{
									const char* texturePath = material->textureSet->GetTexturePath(i);
									if (!texturePath)
									{
										continue;
									}

									const char* textureName = "";
									NiTexturePtr* texture = GetTextureFromIndex(material, i);
									if (texture && texture->get())
									{
										textureName = texture->get()->name;
									}

									_MESSAGE("Texture %d - %s (%s)", i, texturePath, textureName);
								}
								_MESSAGE("Flags - %08X %08X", lightingShader->shaderFlags1, lightingShader->shaderFlags2);
								gLog.Outdent();
							}
						}
					}
					else if (childNode)
					{
						DumpNodeChildren(childNode);
					}
					else
					{
						_MESSAGE("{%s} {%s} {%X} [%f, %f, %f]", object->GetRTTI()->name, object->m_name, object, object->m_worldTransform.pos.x, object->m_worldTransform.pos.y, object->m_worldTransform.pos.z);
					}
				}
			}
			gLog.Outdent();
		}
	}

	void SMPDebug_PrintDetailed(bool includeItems)
	{
		ActorManager::instance()->SMPDebug_PrintDetailed(includeItems);
	}

	bool SMPDebug_Execute(const ObScriptParam* paramInfo, ScriptData* scriptData, TESObjectREFR* thisObj,
		TESObjectREFR* containingObj, Script* scriptObj, ScriptLocals* locals, double& result,
		UInt32& opcodeOffsetPtr)
	{
		char buffer[MAX_PATH];
		memset(buffer, 0, MAX_PATH);
		char buffer2[MAX_PATH];
		memset(buffer2, 0, MAX_PATH);

#ifdef ANNIVERSARY_EDITION
		if (!ObScript_ExtractArgs(paramInfo, scriptData, opcodeOffsetPtr, thisObj, containingObj, scriptObj, locals, buffer, buffer2))
#else
		if (!ObjScript_ExtractArgs(paramInfo, scriptData, opcodeOffsetPtr, thisObj, containingObj, scriptObj, locals, buffer, buffer2))
#endif
		{
			return false;
		}

		if (_strnicmp(buffer, "reset", MAX_PATH) == 0)
		{
			Console_Print("running full smp reset");
			hdt::loadConfig();
			SkyrimPhysicsWorld::get()->resetTransformsToOriginal();
			const MenuOpenCloseEvent e { false };
			ActorManager::instance()->onEvent(e);
			SkyrimPhysicsWorld::get()->resetSystems();
			return true;
		}
#ifdef CUDA
		if (_strnicmp(buffer, "gpu", MAX_PATH) == 0)
		{
			CudaInterface::enableCuda = !CudaInterface::enableCuda;
			if (CudaInterface::instance()->hasCuda())
			{
				Console_Print("CUDA collision enabled");
			}
			else
			{
				Console_Print("CUDA collision disabled");
			}
			return true;
		}
		if (_strnicmp(buffer, "timing", MAX_PATH) == 0)
		{
			FrameTimer::instance()->reset(200);
			Console_Print("Started frame timing");
			return true;
		}
#endif
		if (_strnicmp(buffer, "dumptree", MAX_PATH) == 0)
		{
			if (thisObj)
			{
				Console_Print("dumping targeted reference's node tree");
				DumpNodeChildren(thisObj->GetNiRootNode(0));
			}
			else
			{
				Console_Print("error: you must target a reference to dump their node tree");
			}

			return true;
		}
		if (_strnicmp(buffer, "detail", MAX_PATH) == 0)
		{
			SMPDebug_PrintDetailed(true);
			return true;
		}
		if (_strnicmp(buffer, "list", MAX_PATH) == 0)
		{
			SMPDebug_PrintDetailed(false);
			return true;
		}
		if (_strnicmp(buffer, "on", MAX_PATH) == 0)
		{
			SkyrimPhysicsWorld::get()->disabled = false;
			{
				Console_Print("HDT-SMP enabled");
			}
			return true;
		}
		if (_strnicmp(buffer, "off", MAX_PATH) == 0)
		{
			SkyrimPhysicsWorld::get()->disabled = true;
			{
				Console_Print("HDT-SMP disabled");
			}
			return true;
		}

		if (_strnicmp(buffer, "QueryOverride", MAX_PATH) == 0) {
			Console_Print(hdt::Override::OverrideManager::GetSingleton()->queryOverrideData().c_str());
			return true;
		}

		ActorManager::instance()->SMPDebug_Execute();
		return true;
	}
}

extern "C" {
#ifdef ANNIVERSARY_EDITION
	__declspec(dllexport) SKSEPluginVersionData SKSEPlugin_Version =
	{
		SKSEPluginVersionData::kVersion,
		2,
		"hdtSMP64",
		"hydrogensaysHDT",
		"",
		0,	// not version independent
		{ RUNTIME_VERSION_1_6_353, 0 },
		0,	// works with any version of the script extender. you probably do not need to put anything here
	};
#else
	bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
	{
		// populate info structure
		info->infoVersion = PluginInfo::kInfoVersion;
		info->name = "hdtSSEPhysics";
		info->version = 1;

		hdt::gLog.OpenRelative(CSIDL_MYDOCUMENTS,
#ifndef SKYRIMVR
			"\\My Games\\Skyrim Special Edition\\SKSE\\hdtSMP64.log"

#else
			"\\My Games\\Skyrim VR\\SKSE\\hdtSMP64.log"
#endif
		);
		hdt::gLog.SetLogLevel(IDebugLog::LogLevel::kLevel_Message);

		_MESSAGE("hdtSMP64 2.0");

		if (skse->isEditor)
		{
			return false;
		}

		if (skse->runtimeVersion != CURRENT_RELEASE_RUNTIME)
		{
			_FATALERROR("attempted to load plugin into unsupported game version, exiting");
			return false;
		}

		if (!g_branchTrampoline.Create(1024 * 1))
		{
			_FATALERROR("couldn't create branch trampoline. this is fatal. skipping remainder of init process.");
			return false;
		}

		if (!g_localTrampoline.Create(1024 * 1, nullptr))
		{
			_FATALERROR("couldn't create codegen buffer. this is fatal. skipping remainder of init process.");
			return false;
		}

		hdt::g_PluginHandle = skse->GetPluginHandle();

		return true;
	}
#endif

	bool SKSEPlugin_Load(const SKSEInterface* skse)
	{
#ifdef ANNIVERSARY_EDITION
		hdt::gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim Special Edition\\SKSE\\hdtSMP64.log");
		hdt::gLog.SetLogLevel(IDebugLog::LogLevel::kLevel_Message);
		_MESSAGE("hdtSMP64 2.0");

		if (!g_branchTrampoline.Create(1024 * 1))
		{
			_FATALERROR("Couldn't create branch trampoline. This is fatal. Skipping remainder of init process.");
			return false;
		}

		if (!g_localTrampoline.Create(1024 * 1, nullptr))
		{
			_FATALERROR("Couldn't create codegen buffer. This is fatal. Skipping remainder of init process.");
			return false;
		}
#endif // ANNIVERSARY_EDITION

		hdt::g_frameEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_frameEventDispatcher.addListener(hdt::SkyrimPhysicsWorld::get());
		hdt::g_shutdownEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_shutdownEventDispatcher.addListener(hdt::SkyrimPhysicsWorld::get());
		hdt::g_armorAttachEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_skinSingleHeadGeometryEventDispatcher.addListener(hdt::ActorManager::instance());
		hdt::g_skinAllHeadGeometryEventDispatcher.addListener(hdt::ActorManager::instance());

		hdt::hookAll();

		const auto messageInterface = reinterpret_cast<SKSEMessagingInterface*>(skse->QueryInterface(kInterface_Messaging));
		if (messageInterface)
		{
			const auto cameraDispatcher = static_cast<EventDispatcher<SKSECameraEvent>*>(messageInterface->
				GetEventDispatcher(SKSEMessagingInterface::kDispatcher_CameraEvent));

			if (cameraDispatcher)
				cameraDispatcher->AddEventSink(hdt::SkyrimPhysicsWorld::get());

			messageInterface->RegisterListener(hdt::g_PluginHandle, "SKSE", [](SKSEMessagingInterface::Message* msg)
				{
					if (msg && msg->type == SKSEMessagingInterface::kMessage_InputLoaded)
					{
						MenuManager* mm = MenuManager::GetSingleton();
						if (mm)
							mm->MenuOpenCloseEventDispatcher()->AddEventSink(&hdt::g_freezeEventHandler);
						hdt::checkOldPlugins();
						hdt::loadConfig();
#ifdef DEBUG
						hdt::g_armorAttachEventDispatcher.addListener(&hdt::g_eventDebugLogger);
						GetEventDispatcherList()->unk1B8.AddEventSink(&hdt::g_eventDebugLogger);
						GetEventDispatcherList()->unk840.AddEventSink(&hdt::g_eventDebugLogger);
#endif
					}

				});
		}

		const SKSESerializationInterface* srlz_intfc = reinterpret_cast<SKSESerializationInterface*>(skse->QueryInterface(kInterface_Serialization));
		if (srlz_intfc) {
			//Initialize all Serializer Module
			hdt::Override::OverrideManager::GetSingleton();

			srlz_intfc->SetUniqueID(hdt::g_PluginHandle, 'FHDT');

			srlz_intfc->SetSaveCallback(hdt::g_PluginHandle, hdt::SerializerBase::Save);

			srlz_intfc->SetLoadCallback(hdt::g_PluginHandle, hdt::SerializerBase::Load);
		}

		ObScriptCommand* hijackedCommand = nullptr;
		for (ObScriptCommand* iter = g_firstConsoleCommand; iter->opcode < kObScript_NumConsoleCommands +
			kObScript_ConsoleOpBase; ++iter)
		{
			if (!strcmp(iter->longName, "ShowRenderPasses"))
			{
				hijackedCommand = iter;
				break;
			}
		}
		if (hijackedCommand)
		{
			static ObScriptParam params[1];
			params[0].typeID = ObScriptParam::kType_String;
			params[0].typeStr = "String (optional)";
			params[0].isOptional = 1;

			ObScriptCommand cmd = *hijackedCommand;

			cmd.longName = "SMPDebug";
			cmd.shortName = "smp";
			cmd.helpText = "smp <reset>";
			cmd.needsParent = 0;
			cmd.numParams = 1;
			cmd.params = params;
			cmd.execute = hdt::SMPDebug_Execute;
			cmd.flags = 0;
			SafeWriteBuf(reinterpret_cast<uintptr_t>(hijackedCommand), &cmd, sizeof(cmd));
		}

		hdt::papyrus::RegisterAllFunctions(reinterpret_cast<SKSEPapyrusInterface*>(skse->QueryInterface(kInterface_Papyrus)));

		return true;
	}
}
