
#include "WeatherManager.h"
#include "ActorManager.h"
#include "hdtSkyrimPhysicsWorld.h"
#include "hdtDefaultBBP.h"
#include "skse64/GameRTTI.h"
#include "skse64/NiSerialization.h"
#include <cinttypes>
#include "Offsets.h"
#include "skse64/GameStreams.h"
#include "skse64/GameData.h"

namespace hdt
{
	ActorManager::ActorManager()
	{
	}

	ActorManager::~ActorManager()
	{
	}

	ActorManager* ActorManager::instance()
	{
		static ActorManager s;
		return &s;
	}

	IDStr ActorManager::armorPrefix(ActorManager::IDType id)
	{
		char buffer[128];
		sprintf_s(buffer, "hdtSSEPhysics_AutoRename_Armor_%08X ", id);
		return IDStr(buffer);
	}

	IDStr ActorManager::headPrefix(ActorManager::IDType id)
	{
		char buffer[128];
		sprintf_s(buffer, "hdtSSEPhysics_AutoRename_Head_%08X ", id);
		return IDStr(buffer);
	}

	inline bool isFirstPersonSkeleton(NiNode* npc)
	{
		if (!npc) return false;
		return findNode(npc, "Camera1st [Cam1]") ? true : false;
	}

	NiNode* getNpcNode(NiNode* skeleton)
	{
		// TODO: replace this with a generic skeleton fixing configuration option
		// hardcode an exception for lurker skeletons because they are made incorrectly
		auto shouldFix = false;
		if (skeleton->m_owner && skeleton->m_owner->baseForm)
		{
			auto npcForm = DYNAMIC_CAST(skeleton->m_owner->baseForm, TESForm, TESNPC);
			if (npcForm && npcForm->race.race
				&& !strcmp(npcForm->race.race->models[0].GetModelName(), "Actors\\DLC02\\BenthicLurker\\Character Assets\\skeleton.nif"))
				shouldFix = true;
		}
		return findNode(skeleton, shouldFix ? "NPC Root [Root]" : "NPC");
	}

	void ActorManager::fixArmorNameMaps()
	{
		auto& skeletons = instance()->getSkeletons();
		for (auto& skeleton : skeletons)
			if (skeleton.mustFixOneArmorMap)
			{
				auto& armors = skeleton.getArmors();
				for (auto& armor : armors)
				{
					if (armor.mustFixNameMap)
					{
						if (armor.armorWorn)
							if (armor.armorWorn->m_name)
							{
								std::string armorNewMeshName(armor.armorWorn->m_name);
								if (!armorNewMeshName.empty() && armor.armorCurrentMeshName.compare(armorNewMeshName) != 0)
								{
									auto& armorNameMap = armor.physicsFile.second;
									hdt::DefaultBBP::NameMap tempNameMap;
									for (auto& [setName, set] : armorNameMap)
										// ... and we found the old mesh name in the armor nameMap,...
										if (armor.armorCurrentMeshName.compare(setName) == 0)
										{
											// We add the new mesh name to the list of mesh names for the original mesh name (sic).
											set.insert({ armorNewMeshName });
											// We plan a new entry in the armor nameMap.
											tempNameMap.insert({ armorNewMeshName, { armorNewMeshName } });
											// This armor is fixed.
											armor.mustFixNameMap = false;
											armor.armorCurrentMeshName = armorNewMeshName;
										}
									// We add the planned entries.
									for (auto& [setName, set] : tempNameMap)
										armorNameMap.insert({ setName, set });
								}
							}
					}
				}
				skeleton.mustFixOneArmorMap = false;
			}
	}

	void ActorManager::onEvent(const ArmorAttachEvent& e)
	{
		// No armor is ever attached to a lurker skeleton, thus we don't need to test.
		if (e.skeleton == nullptr || !findNode(e.skeleton, "NPC"))
		{
			return;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) return;

		fixArmorNameMaps();

		auto& skeleton = getSkeletonData(e.skeleton);
		if (e.hasAttached)
		{
			// Check override data for current armoraddon
			if (e.skeleton->m_owner)
			{
				auto actor_formID = e.skeleton->m_owner->formID;
				if (actor_formID) {
					auto old_physics_file = skeleton.getArmors().back().physicsFile.first;
					std::string physics_file_path_override = hdt::Override::OverrideManager::GetSingleton()->checkOverride(actor_formID, old_physics_file);
					if (!physics_file_path_override.empty()) {
						//Console_Print("[DynamicHDT] -- ArmorAddon %s is overridden ", e.attachedNode->m_name);
						skeleton.getArmors().back().physicsFile.first = physics_file_path_override;
					}
				}
			}


			skeleton.attachArmor(e.armorModel, e.attachedNode);
		}
		else
		{
			skeleton.addArmor(e.armorModel);
		}
	}

	void ActorManager::onEvent(const ArmorDetachEvent& e)
	{
		if (!e.actor || !e.hasDetached || !instance()->m_disableSMPHairWhenWigEquipped)
			return;

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) return;

		fixArmorNameMaps();

		Skeleton* s = get3rdPersonSkeleton(e.actor);
		setHeadActiveIfNoHairArmor(e.actor, s);
	}

	// @brief To avoid calculating headparts when they're hidden by a wig,
	// we mark the head as not active when there is an armor on hair or long hair slots.
	// We do this during the attach/detach armor events, and on the events leading to scanning the head.
	// Then when checking which skeletons are active to calculate the frame,
	// we only allow the activation of headparts that are on active heads.
	// @param Actor * actor is expected not null.
	void ActorManager::setHeadActiveIfNoHairArmor(Actor* actor, Skeleton* skeleton)
	{
		const int hairslot = 1 << 1;
		const int longhairslot = 1 << 11;
		auto worn = papyrusActor::GetWornForm(actor, hairslot | longhairslot);

		if (skeleton)
			skeleton->head.isActive = !worn;
	}

	// @brief This happens on a closing RaceSex menu, and on 'smp reset'.
	void ActorManager::onEvent(const MenuOpenCloseEvent&)
	{
		// The ActorManager members are protected from parallel events by ActorManager.m_lock.
		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) return;

		_DMESSAGE("Processing MenuOpenCloseEvent.");

		fixArmorNameMaps();

		setSkeletonsActive();

		for (auto& i : m_skeletons)
			i.reloadMeshes();
	}

	void ActorManager::onEvent(const FrameEvent& e)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

		fixArmorNameMaps();

		setSkeletonsActive(true);
	}

	//NiAVObject* Actor::CalculateLOS_1405FD2C0(Actor *aActor, NiPoint3 *aTargetPosition, NiPoint3 *aRayHitPosition, float aViewCone)
	//Used to ray cast from the actor. Will return nonNull if it hits something with position at aTargetPosition.
	//Pass in 2pi to aViewCone to ignore LOS of actor.
	typedef NiAVObject* (*_Actor_CalculateLOS)(Actor* aActor, NiPoint3* aTargetPosition, NiPoint3* aRayHitPosition, float aViewCone);
	RelocAddr<_Actor_CalculateLOS> Actor_CalculateLOS(offset::Actor_CalculateLOS);

	inline NiNode* ActorManager::getCameraNode()
	{
#ifdef SKYRIMVR
		// Camera info taken from Shizof's cpbc under MIT. https://www.nexusmods.com/skyrimspecialedition/mods/21224?tab=files
		if (!(*g_thePlayer)->loadedState)
			return nullptr;
		return (*g_thePlayer)->loadedState->node;
#else
		return PlayerCamera::GetSingleton()->cameraNode;
#endif
	}

	// @brief This function is called by different events, with different locking needs, and is therefore extracted from the events.
	void ActorManager::setSkeletonsActive(const bool updateMetrics)
	{
		if (m_shutdown) return;

		// We get the player character and its cell.
		// TODO Isn't there a more performing way to find the PC?? A singleton? And if it's the right way, why isn't it in utils functions?
		auto& playerCharacter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [](Skeleton& s) { return s.isPlayerCharacter(); });
		auto playerCell = (playerCharacter != m_skeletons.end() && playerCharacter->skeleton->m_parent) ? playerCharacter->skeleton->m_parent->m_parent : nullptr;

		const auto cameraNode = getCameraNode();
		if (!cameraNode)
			return;
		// We get the camera, its position and orientation.
		const auto cameraTransform = cameraNode->m_worldTransform;
		const auto cameraPosition = cameraTransform.pos;
		const auto cameraOrientation = cameraTransform.rot * NiPoint3(0., 1., 0.); // The camera matrix is relative to the world.
		this->m_cameraPositionDuringFrame = cameraPosition;

		std::for_each(m_skeletons.begin(), m_skeletons.end(), [&](Skeleton& skel)
			{
				skel.calculateDistanceAndOrientationDifferenceFromSource(cameraPosition, cameraOrientation);
			});

		// We sort the skeletons depending on the angle and distance.
		std::sort(m_skeletons.begin(), m_skeletons.end(),
			[](auto&& a_lhs, auto&& a_rhs) {
				auto cr = a_rhs.m_cosAngleFromCameraDirectionTimesSkeletonDistance;
				auto cl = a_lhs.m_cosAngleFromCameraDirectionTimesSkeletonDistance;
				auto dr = a_rhs.m_distanceFromCamera2;
				auto dl = a_lhs.m_distanceFromCamera2;
				return
				// If one of the skeletons is at distance zero (1st person player) from the camera
				(btFuzzyZero(dl) || btFuzzyZero(dr))
				// then it is first.
				? (dl < dr)

				// If one of the skeletons is exacly on the side of the camera (cos = 0)
				: (btFuzzyZero(cl) || btFuzzyZero(cr))
				// then it is last.
				? abs(cl) > abs(cr)

				// If both are on the same side of the camera (product of cos > 0):
				// we want first the smallest angle (so the highest cosinus), and the smallest distance,
				// so we want the smallest distance / cosinus.
				// cl = cosinus * distance, dl = distance� => distance / cosinus = dl/cl
				// So we want dl/cl < dr/cr.
				// Moreover, this test manages the case where one of the skeletons is behind the camera and the other in front of the camera too;
				// the one behind the camera is last (the one with cos(angle) = cr < 0).
				: (dl * cr < dr * cl);
			});

		// We set which skeletons are active and we count them.
		activeSkeletons = 0;
		for (auto& i : m_skeletons)
		{
			if (i.skeleton->m_uiRefCount == 1)
			{
				i.clear();
				i.skeleton = nullptr;
			}
			else if (i.hasPhysics && i.updateAttachedState(playerCell, activeSkeletons >= maxActiveSkeletons)) {
				activeSkeletons++;
				//check wind obstructions
				const auto world = SkyrimPhysicsWorld::get();
				const auto wind = getWindDirection();
				if (world->m_enableWind && wind && !(btFuzzyZero(hdt::magnitude(*wind)))) {
					const auto owner = DYNAMIC_CAST(i.skeletonOwner.get(), TESForm, Actor);
					if (owner) {
						auto windray = *wind * -1; // reverse wind raycast to find obstruction
						NiPoint3 hitLocation;
						//Raycast for object in direction of wind
						const auto object = Actor_CalculateLOS(owner, &windray, &hitLocation, 6.28);
						if (object) { //object found
							auto diff = (owner->pos - hitLocation);
							diff.z = 0;	//remove z component difference
							const auto dist = hdt::magnitude(diff);
							// wind is a linear reduction, with a minimum floor since objects may have a minimum distance
							// windfactor = 0 when dist <= m_distanceForNoWind, = 1 when dist >= m_distanceForMaxWind, and is linear with dist between these 2 values.
							const auto windFactor = std::clamp((dist - world->m_distanceForNoWind) / (world->m_distanceForMaxWind - world->m_distanceForNoWind), 0.f, 1.f);
							if (!btFuzzyZero(windFactor - i.getWindFactor())) {
								_DMESSAGE("%s blocked by %s with distance %2.2g; setting windFactor %2.2g.", i.name(), object->m_name, dist, windFactor);
								i.updateWindFactor(windFactor);
							}
						}
					}
					else {
						_DMESSAGE("%s is active skeleton, but failed to cast to Actor, no wind obstruction check possible.", i.name());
					}
				}
			}
		}

		m_skeletons.erase(
			std::remove_if(m_skeletons.begin(), m_skeletons.end(), [](Skeleton& i) { return !i.skeleton; }),
			m_skeletons.end());

		for (auto& i : m_skeletons)
		{
			i.cleanArmor();
			i.cleanHead();
		}

		const auto world = SkyrimPhysicsWorld::get();

		// We share the same doMetrics condition here and in hdtSkyrimPhysicsWorld to avoid any gap between both.
		// The evaluation is done here rather than in hdtSkyrimPhysicsWorld because this event is called first.
		world->m_doMetrics = updateMetrics &&                    // do not do metrics on a MenuOpenCloseEvent
			                 !world->isSuspended() &&            // do not do metrics while paused
		                     frameCount++ % world->min_fps == 0; // check every min-fps frames (i.e., a stable 60 fps should wait for 1 second)

		if (world->m_doMetrics)
		{
			const auto averageProcessingTimeInMainLoop = world->m_averageSMPProcessingTimeInMainLoop;
			// 30% of processing time is in hdt per profiling;
			// Setting it higher provides more time for hdt processing and can activate more skeletons.
			const auto target_time = world->m_timeTick * world->m_percentageOfFrameTime;
			auto averageTimePerSkeletonInMainLoop = 0.f;
			if (activeSkeletons > 0) {
				averageTimePerSkeletonInMainLoop = averageProcessingTimeInMainLoop / activeSkeletons;
			}
			_VMESSAGE("msecs/activeSkeleton %2.2g activeSkeletons/maxActive/total %d/%d/%d processTimeInMainLoop/targetTime %2.2g/%2.2g", averageTimePerSkeletonInMainLoop, activeSkeletons, maxActiveSkeletons, m_skeletons.size(), averageProcessingTimeInMainLoop, target_time);
			if (m_autoAdjustMaxSkeletons) {
				maxActiveSkeletons += target_time > averageProcessingTimeInMainLoop ? 2 : -2;
				// clamp the value to the m_maxActiveSkeletons value
				maxActiveSkeletons = std::clamp(maxActiveSkeletons, 1, m_maxActiveSkeletons);
				frameCount = 1;
			}
			else if (maxActiveSkeletons != m_maxActiveSkeletons)
				maxActiveSkeletons = m_maxActiveSkeletons;
		}
	}

	void ActorManager::onEvent(const ShutdownEvent&)
	{
		m_shutdown = true;
		std::lock_guard<decltype(m_lock)> l(m_lock);

		m_skeletons.clear();
	}

	void ActorManager::onEvent(const SkinSingleHeadGeometryEvent& e)
	{
		// This case never happens to a lurker skeleton, thus we don't need to test.
		auto npc = findNode(e.skeleton, "NPC");
		if (!npc) return;

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) return;

		fixArmorNameMaps();

		auto& skeleton = getSkeletonData(e.skeleton);
		skeleton.npc = getNpcNode(e.skeleton);

		skeleton.processGeometry(e.headNode, e.geometry);

		auto headPartIter = std::find_if(skeleton.head.headParts.begin(), skeleton.head.headParts.end(),
			[e](const Head::HeadPart& p)
			{
				return p.headPart == e.geometry;
			});

		if (headPartIter != skeleton.head.headParts.end())
		{
			if (headPartIter->origPartRootNode)
			{
				_DMESSAGE("Renaming nodes in original part %s back.", headPartIter->origPartRootNode->m_name);

				for (auto& entry : skeleton.head.renameMap)
				{
					// This case never happens to a lurker skeleton, thus we don't need to test.
					auto node = findNode(headPartIter->origPartRootNode, entry.second->cstr());
					if (node)
					{
						_DMESSAGE("Rename node %s -> %s.", entry.second->cstr(), entry.first->cstr());
						setNiNodeName(node, entry.first->cstr());
					}
				}
			}
			headPartIter->origPartRootNode = nullptr;
		}

		if (!skeleton.head.isFullSkinning)
			skeleton.scanHead();
	}

	void ActorManager::onEvent(const SkinAllHeadGeometryEvent& e)
	{
		// This case never happens to a lurker skeleton, thus we don't need to test.
		auto npc = findNode(e.skeleton, "NPC");
		if (!npc) return;

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) return;

		fixArmorNameMaps();

		auto& skeleton = getSkeletonData(e.skeleton);
		skeleton.npc = npc;
		if (e.skeleton->m_owner)
			skeleton.skeletonOwner = e.skeleton->m_owner;

		if (e.hasSkinned)
		{
			skeleton.scanHead();
			skeleton.head.isFullSkinning = false;
			if (skeleton.head.npcFaceGeomNode)
			{
				_DMESSAGE("NPC face geometry no longer needed, clearing reference.");
				skeleton.head.npcFaceGeomNode = nullptr;
			}
		}
		else
		{
			skeleton.head.isFullSkinning = true;
		}
	}

	void ActorManager::PhysicsItem::setPhysics(Ref<SkyrimSystem>& system, bool active)
	{
		clearPhysics();
		m_physics = system;
		if (active)
		{
			SkyrimPhysicsWorld::get()->addSkinnedMeshSystem(m_physics);
		}
	}

	void ActorManager::PhysicsItem::clearPhysics()
	{
		if (state() == ItemState::e_Active)
		{
			m_physics->m_world->removeSkinnedMeshSystem(m_physics);
		}
		m_physics = nullptr;
	}

	ActorManager::ItemState ActorManager::PhysicsItem::state() const
	{
		return m_physics ? (m_physics->m_world ? ItemState::e_Active : ItemState::e_Inactive) : ItemState::e_NoPhysics;
	}

	const std::vector<Ref<SkinnedMeshBody>>& ActorManager::PhysicsItem::meshes() const
	{
		return m_physics->meshes();
	}

	void ActorManager::PhysicsItem::updateActive(bool active)
	{
		if (active && state() == ItemState::e_Inactive)
		{
			SkyrimPhysicsWorld::get()->addSkinnedMeshSystem(m_physics);
		}
		else if (!active && state() == ItemState::e_Active)
		{
			m_physics->m_world->removeSkinnedMeshSystem(m_physics);
		}
	}

	void ActorManager::PhysicsItem::setWindFactor(float a_windFactor)
	{
		if (m_physics)
			m_physics->m_windFactor = a_windFactor;
	}

	std::vector<ActorManager::Skeleton>& ActorManager::getSkeletons()
	{
		return m_skeletons;
	}

	bool ActorManager::skeletonNeedsParts(NiNode* skeleton)
	{
		return !isFirstPersonSkeleton(skeleton);
		/*
		auto iter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [=](Skeleton& i)
		{
			return i.skeleton == skeleton;
		});
		if (iter != m_skeletons.end())
		{
			return (iter->head.headNode == 0);
		}
		*/
	}

	ActorManager::Skeleton& ActorManager::getSkeletonData(NiNode* skeleton)
	{
		auto iter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [=](Skeleton& i)
			{
				return i.skeleton == skeleton;
			});
		if (iter != m_skeletons.end())
		{
			return *iter;
		}
		if (!isFirstPersonSkeleton(skeleton))
		{
			auto ownerIter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [=](Skeleton& i)
				{
					return !isFirstPersonSkeleton(i.skeleton) && i.skeletonOwner && skeleton->m_owner && i.skeletonOwner ==
						skeleton->m_owner;
				});
			if (ownerIter != m_skeletons.end())
			{
				_DMESSAGE("New skeleton found for formid %08x.", skeleton->m_owner->formID);
				ownerIter->cleanHead(true);
			}
		}
		m_skeletons.push_back(Skeleton());
		m_skeletons.back().skeleton = skeleton;
		return m_skeletons.back();
	}

	ActorManager::Skeleton* ActorManager::get3rdPersonSkeleton(Actor* actor)
	{
		for (auto& i : m_skeletons)
		{
			const auto owner = DYNAMIC_CAST(i.skeletonOwner.get(), TESForm, Actor);
			if (actor == owner && i.skeleton && !isFirstPersonSkeleton(i.skeleton))
				return &i;
		}
		return 0;
	}

	void ActorManager::Skeleton::doSkeletonMerge(NiNode* dst, NiNode* src, IString* prefix,
		std::unordered_map<IDStr, IDStr>& map)
	{
		for (int i = 0; i < src->m_children.m_arrayBufLen; ++i)
		{
			auto srcChild = castNiNode(src->m_children.m_data[i]);
			if (!srcChild) continue;

			if (!srcChild->m_name)
			{
				doSkeletonMerge(dst, srcChild, prefix, map);
				continue;
			}

			// FIXME: This was previously only in doHeadSkeletonMerge.
			// But surely non-head skeletons wouldn't have this anyway?
			if (!strcmp(srcChild->m_name, "BSFaceGenNiNodeSkinned"))
			{
				_DMESSAGE("Skipping facegen ninode in skeleton merge.");
				continue;
			}

			// TODO check it's not a lurker skeleton
			auto dstChild = findNode(dst, srcChild->m_name);
			if (dstChild)
			{
				doSkeletonMerge(dstChild, srcChild, prefix, map);
			}
			else
			{
				dst->AttachChild(cloneNodeTree(srcChild, prefix, map), false);
			}
		}
	}

	NiNode* ActorManager::Skeleton::cloneNodeTree(NiNode* src, IString* prefix, std::unordered_map<IDStr, IDStr>& map)
	{
		NiCloningProcess c;
		auto ret = static_cast<NiNode*>(src->CreateClone(c));
		src->ProcessClone(&c);

		// FIXME: cloneHeadNodeTree just did this for ret, not both. Don't know if that matters. Armor parts need it on both.
		renameTree(src, prefix, map);
		renameTree(ret, prefix, map);

		return ret;
	}

	void ActorManager::Skeleton::renameTree(NiNode* root, IString* prefix, std::unordered_map<IDStr, IDStr>& map)
	{
		if (root->m_name)
		{
			std::string newName(prefix->cstr(), prefix->size());
			newName += root->m_name;
			if (map.insert(std::make_pair<IDStr, IDStr>(root->m_name, newName)).second)
				_DMESSAGE("Rename Bone %s -> %s.", root->m_name, newName.c_str());
			setNiNodeName(root, newName.c_str());
		}

		for (int i = 0; i < root->m_children.m_arrayBufLen; ++i)
		{
			auto child = castNiNode(root->m_children.m_data[i]);
			if (child)
				renameTree(child, prefix, map);
		}
	}

	void ActorManager::Skeleton::doSkeletonClean(NiNode* dst, IString* prefix)
	{
		for (int i = dst->m_children.m_arrayBufLen - 1; i >= 0; --i)
		{
			auto child = castNiNode(dst->m_children.m_data[i]);
			if (!child) continue;

			if (child->m_name && !strncmp(child->m_name, prefix->cstr(), prefix->size()))
			{
				dst->RemoveAt(i++);
			}
			else
			{
				doSkeletonClean(child, prefix);
			}
		}
	}

	// returns the name of the skeleton owner
	std::string ActorManager::Skeleton::name()
	{
		if (skeleton->m_owner && skeleton->m_owner->baseForm) {
			auto bname = DYNAMIC_CAST(skeleton->m_owner->baseForm, TESForm, TESFullName);
			if (bname)
				return bname->GetName();
		}
		return "";
	}

	void ActorManager::Skeleton::addArmor(NiNode* armorModel)
	{
		IDType id = armors.size() ? armors.back().id + 1 : 0;
		auto prefix = armorPrefix(id);
		// FIXME we probably could simplify this by using findNode as surely we don't merge Armors with lurkers skeleton?
		npc = getNpcNode(skeleton);
		auto physicsFile = DefaultBBP::instance()->scanBBP(armorModel);

		armors.push_back(Armor());
		armors.back().id = id;
		armors.back().prefix = prefix;
		armors.back().physicsFile = physicsFile;

		doSkeletonMerge(npc, armorModel, prefix, armors.back().renameMap);
	}

	void ActorManager::Skeleton::attachArmor(NiNode* armorModel, NiAVObject* attachedNode)
	{
		if (armors.size() == 0 || armors.back().hasPhysics())
			_MESSAGE("Not attaching armor - no record or physics already exists");

		Armor& armor = armors.back();

		// The name of the attachedNode provided here will have been changed by the Skyrim exe between this event and the next.
		armor.armorWorn = attachedNode;
		// That's why we set here the need to fix this armor in fixArmorNameMaps() (see its comment)
		// to avoid this name change breaking processes like 'smp reset' when looking for the armor name in the armor nameMap.
		armor.armorCurrentMeshName = attachedNode->m_name ? attachedNode->m_name : "";
		armor.mustFixNameMap = true;
		mustFixOneArmorMap = true;

		if (!isFirstPersonSkeleton(skeleton))
		{
			std::unordered_map<IDStr, IDStr> renameMap = armor.renameMap;
			// FIXME we probably could simplify this by using findNode as surely we don't attach Armors to lurkers skeleton?
			auto system = SkyrimSystemCreator().createOrUpdateSystem(getNpcNode(skeleton), attachedNode, &armor.physicsFile, std::move(renameMap), nullptr);

			if (system)
			{
				armor.setPhysics(system, isActive);
				hasPhysics = true;
			}
		}

		if (instance()->m_disableSMPHairWhenWigEquipped && skeleton && skeleton->m_owner)
		{
			TESForm* form = LookupFormByID(skeleton->m_owner->formID);
			Actor* actor = DYNAMIC_CAST(form, TESForm, Actor);
			if (actor)
				ActorManager().setHeadActiveIfNoHairArmor(actor, this);
		}
	}

	void ActorManager::Skeleton::cleanArmor()
	{
		for (auto& i : armors)
		{
			if (!i.armorWorn) continue;
			if (i.armorWorn->m_parent) continue;

			i.clearPhysics();
			if (npc) doSkeletonClean(npc, i.prefix);
			i.prefix = nullptr;
		}

		armors.erase(std::remove_if(armors.begin(), armors.end(), [](Armor& i) { return !i.prefix; }), armors.end());
	}

	void ActorManager::Skeleton::cleanHead(bool cleanAll)
	{
		for (auto& headPart : head.headParts)
		{
			if (!headPart.headPart->m_parent || cleanAll)
			{
				if (cleanAll)
					_DMESSAGE("Cleaning headpart %s due to clean all.", headPart.headPart->m_name);
				else
					_DMESSAGE("Headpart %s disconnected.", headPart.headPart->m_name);

				auto renameIt = this->head.renameMap.begin();

				while (renameIt != this->head.renameMap.end())
				{
					bool erase = false;

					if (headPart.renamedBonesInUse.count(renameIt->first) != 0)
					{
						auto findNode = this->head.nodeUseCount.find(renameIt->first);
						if (findNode != this->head.nodeUseCount.end())
						{
							findNode->second -= 1;
							_DMESSAGE("Decrementing use count by 1, it is now %d.", findNode->second);
							if (findNode->second <= 0)
							{
								_DMESSAGE("Node no longer in use, cleaning from skeleton.");
								auto removeObj = findObject(npc, renameIt->second->cstr());
								if (removeObj)
								{
									_DMESSAGE("Found node %s, removing.", removeObj->m_name);
									auto parent = removeObj->m_parent;
									if (parent)
									{
										parent->RemoveChild(removeObj);
										removeObj->DecRef();
									}
								}
								this->head.nodeUseCount.erase(findNode);
								erase = true;
							}
						}
					}

					if (erase)
						renameIt = this->head.renameMap.erase(renameIt);
					else
						++renameIt;
				}

				headPart.headPart = nullptr;
				headPart.origPartRootNode = nullptr;
				headPart.clearPhysics();
				headPart.renamedBonesInUse.clear();
			}
		}

		head.headParts.erase(std::remove_if(head.headParts.begin(), head.headParts.end(),
			[](Head::HeadPart& i) { return !i.headPart; }), head.headParts.end());
	}

	void ActorManager::Skeleton::clear()
	{
		std::for_each(armors.begin(), armors.end(), [](Armor& armor) { armor.clearPhysics(); });
		SkyrimPhysicsWorld::get()->removeSystemByNode(npc);
		cleanHead();
		head.headParts.clear();
		head.headNode = nullptr;
		armors.clear();
	}

	void ActorManager::Skeleton::calculateDistanceAndOrientationDifferenceFromSource(NiPoint3 sourcePosition, NiPoint3 sourceOrientation)
	{
		if (isPlayerCharacter())
		{
			m_distanceFromCamera2 = 0.f;
			return;
		}

		auto pos = position();
		if (!pos.has_value())
		{
			m_distanceFromCamera2 = std::numeric_limits<float>::max();
			return;
		}

		// We calculate the vector between camera and the skeleton feets.
		const auto camera2SkeletonVector = pos.value() - sourcePosition;
		// This is the distance (squared) between the camera and the skeleton feets.
		m_distanceFromCamera2 = camera2SkeletonVector.x * camera2SkeletonVector.x + camera2SkeletonVector.y * camera2SkeletonVector.y + camera2SkeletonVector.z * camera2SkeletonVector.z;
		// This is |camera2SkeletonVector|*cos(angle between both vectors)
		m_cosAngleFromCameraDirectionTimesSkeletonDistance = camera2SkeletonVector.x * sourceOrientation.x + camera2SkeletonVector.y * sourceOrientation.y + camera2SkeletonVector.z * sourceOrientation.z;
	}

	// Is called to print messages only
	bool ActorManager::Skeleton::checkPhysics()
	{
		hasPhysics = false;
		std::for_each(armors.begin(), armors.end(), [=](Armor& armor) {
			if (armor.state() != ItemState::e_NoPhysics)
				hasPhysics = true;
			});
		if (!hasPhysics)
			std::for_each(head.headParts.begin(), head.headParts.end(), [=](Head::HeadPart& headPart) {
			if (headPart.state() != ItemState::e_NoPhysics)
				hasPhysics = true;
				});
		_MESSAGE("%s isDrawn %d: %d", name(), hasPhysics);

		return hasPhysics;
	}

	bool ActorManager::Skeleton::isActiveInScene() const
	{
		// TODO: do this better
		// When entering/exiting an interior, NPCs are detached from the scene but not unloaded, so we need to check two levels up.
		// This properly removes exterior cell armors from the physics world when entering an interior, and vice versa.
		return skeleton->m_parent && skeleton->m_parent->m_parent && skeleton->m_parent->m_parent->m_parent;
	}

	bool ActorManager::Skeleton::isPlayerCharacter() const
	{
		constexpr UInt32 playerFormID = 0x14;
		return skeletonOwner == *g_thePlayer.GetPtr() || (skeleton->m_owner && skeleton->m_owner->formID == playerFormID);
	}

	bool ActorManager::Skeleton::isInPlayerView()
	{
		// This function is called only when the skeleton isn't the player character.
		// This might change in the future; in that case this test will have to be enabled.
		//if (isPlayerCharacter())
		//	return true;

		// We always enable the skeletons that are just around the camera.
		// It's useful if for example the skeleton origin is very near, behind the camera,
		// but some parts or the skeleton are in front of the camera and need to be animated.
		auto i = ActorManager::instance();
		float minDistance = i->m_minCullingDistance;
		if (m_distanceFromCamera2 < minDistance * minDistance)
			return true;

		// We don't enable the skeletons behind the camera or on its side.
		if (m_cosAngleFromCameraDirectionTimesSkeletonDistance <= 0)
			return false;

		// We enable only the skeletons that can see the PC or the camera
		const auto owner = DYNAMIC_CAST(this->skeletonOwner.get(), TESForm, Actor);
		if (owner) {
			NiPoint3 hitLocation;
			const auto object = Actor_CalculateLOS(owner, &(i->m_cameraPositionDuringFrame), &hitLocation, 6.28);
			return object ? false : true; // If object, we hit something on the path
		}
		return true; // should never happen, a skeleton without owner?
	}

	std::optional<NiPoint3> ActorManager::Skeleton::position() const
	{
		if (npc)
		{
			// This works for lurker skeletons.
			auto rootNode = findNode(npc, "NPC Root [Root]");
			if (rootNode) return std::optional<NiPoint3>(rootNode->m_worldTransform.pos);
		}
		return std::optional<NiPoint3>();
	}

	void ActorManager::Skeleton::updateWindFactor(float a_windFactor)
	{
		this->currentWindFactor = a_windFactor;
		std::for_each(armors.begin(), armors.end(), [=](Armor& armor) { armor.setWindFactor(a_windFactor); });
		std::for_each(head.headParts.begin(), head.headParts.end(), [=](Head::HeadPart& headPart) { headPart.setWindFactor(a_windFactor); });
	}

	float ActorManager::Skeleton::getWindFactor()
	{
		return this->currentWindFactor;
	}

	bool ActorManager::Skeleton::updateAttachedState(const NiNode* playerCell, bool deactivate = false)
	{
		// 1- Skeletons that aren't active in any scene are always detached, unless they are in the
		// same cell as the player character (workaround for issue in Ancestor Glade).
		// 2- Player character is always attached.
		// 3- Otherwise, attach only if both the camera and this skeleton have a position,
		// the distance between them is below the threshold value,
		// and the angle difference between the camera orientation and the skeleton orientation is below the threshold value.
		isActive = false;
		state = SkeletonState::e_InactiveNotInScene;

		if (deactivate)
			state = SkeletonState::e_InactiveTooFar;
		else if (isActiveInScene() || skeleton->m_parent && skeleton->m_parent->m_parent == playerCell)
		{
			if (isPlayerCharacter())
			{
				// That setting defines whether we don't set the PC skeleton as active
				// when it is in 1st person view, to avoid calculating physics uselessly.
				if (!(instance()->m_disable1stPersonViewPhysics // disabling?
					&& PlayerCamera::GetSingleton()->cameraState == PlayerCamera::GetSingleton()->cameraStates[0])) // 1st person view
				{
					isActive = true;
					state = SkeletonState::e_ActiveIsPlayer;
				}
			}
			else if (isInPlayerView())
			{
				isActive = true;
				state = SkeletonState::e_ActiveNearPlayer;
			}
			else
				state = SkeletonState::e_InactiveUnseenByPlayer;
		}

		// We update the activity state of armors and head parts, and add and remove SkinnedMeshSystems to these parts in consequence.
		// We set headparts as not active if the head isn't active (for example because it's hidden by a wig).
		std::for_each(armors.begin(), armors.end(), [=](Armor& armor) { armor.updateActive(isActive); });
		const bool isHeadActive = head.isActive;
		std::for_each(head.headParts.begin(), head.headParts.end(), [=](Head::HeadPart& headPart) { headPart.updateActive(isHeadActive && isActive); });
		return isActive;
	}

	void ActorManager::Skeleton::reloadMeshes()
	{
		for (auto& i : armors)
		{
			i.clearPhysics();

			if (!isFirstPersonSkeleton(skeleton))
			{
				std::unordered_map<IDStr, IDStr> renameMap = i.renameMap;
				auto system = SkyrimSystemCreator().createOrUpdateSystem(npc, i.armorWorn, &i.physicsFile, std::move(renameMap), nullptr);

				if (system)
				{
					i.setPhysics(system, isActive);
					hasPhysics = true;
				}
			}
		}
		scanHead();
	}

	void ActorManager::Skeleton::scanHead()
	{
		if (isFirstPersonSkeleton(this->skeleton))
		{
			_DMESSAGE("Not scanning head of first person skeleton.");
			return;
		}

		if (!this->head.headNode)
		{
			_DMESSAGE("Actor has no head node.");
			return;
		}

		std::unordered_set<std::string> physicsDupes;

		if (instance()->m_disableSMPHairWhenWigEquipped && skeleton && skeleton->m_owner)
		{
			TESForm* form = LookupFormByID(skeleton->m_owner->formID);
			Actor* actor = DYNAMIC_CAST(form, TESForm, Actor);
			if (actor)
				ActorManager().setHeadActiveIfNoHairArmor(actor, this);
		}

		for (auto& headPart : this->head.headParts)
		{
			// always regen physics for all head parts
			headPart.clearPhysics();

			if (headPart.physicsFile.first.empty())
			{
				_DMESSAGE("No physics file for headpart %s.", headPart.headPart->m_name);
				continue;
			}

			if (physicsDupes.count(headPart.physicsFile.first))
			{
				_DMESSAGE("Previous head part generated physics system for file %s, skipping.",
					headPart.physicsFile.first.c_str());
				continue;
			}

			std::unordered_map<IDStr, IDStr> renameMap = this->head.renameMap;

			_DMESSAGE("Try create system for headpart %s physics file %s.", headPart.headPart->m_name,
				headPart.physicsFile.first.c_str());
			physicsDupes.insert(headPart.physicsFile.first);
			auto system = SkyrimSystemCreator().createOrUpdateSystem(npc, this->head.headNode, &headPart.physicsFile, std::move(renameMap), nullptr);

			if (system)
			{
				_DMESSAGE("Success.");
				headPart.setPhysics(system, isActive);
				hasPhysics = true;
			}
		}
	}

	typedef bool (*_TESNPC_GetFaceGeomPath)(TESNPC* a_npc, char* a_buf);
	RelocAddr<_TESNPC_GetFaceGeomPath> TESNPC_GetFaceGeomPath(offset::TESNPC_GetFaceGeomPath);

	void ActorManager::Skeleton::processGeometry(BSFaceGenNiNode* headNode, BSGeometry* geometry)
	{
		if (this->head.headNode && this->head.headNode != headNode)
		{
			_DMESSAGE("Completely new head attached to skeleton, clearing tracking.");
			for (auto& headPart : this->head.headParts)
			{
				headPart.clearPhysics();
				headPart.headPart = nullptr;
				headPart.origPartRootNode = nullptr;
			}

			this->head.headParts.clear();

			if (npc)
				doSkeletonClean(npc, this->head.prefix);

			this->head.prefix = nullptr;
			this->head.headNode = nullptr;
			this->head.renameMap.clear();
			this->head.nodeUseCount.clear();
		}

		// clean swapped out headparts
		cleanHead();

		this->head.headNode = headNode;
		++this->head.id;
		this->head.prefix = headPrefix(this->head.id);

		auto it = std::find_if(this->head.headParts.begin(), this->head.headParts.end(),
			[geometry](const Head::HeadPart& p)
			{
				return p.headPart == geometry;
			});

		if (it != this->head.headParts.end())
		{
			_DMESSAGE("Geometry is already added as head part.");
			return;
		}

		this->head.headParts.push_back(Head::HeadPart());

		head.headParts.back().headPart = geometry;
		head.headParts.back().clearPhysics();

		// Skinning
		_DMESSAGE("Skinning geometry to skeleton.");

		if (!geometry->m_spSkinInstance || !geometry->m_spSkinInstance->m_spSkinData)
		{
			_ERROR("Geometry is missing skin instance - how?");
			return;
		}

		auto fmd = static_cast<BSFaceGenModelExtraData*>(geometry->GetExtraData("FMD"));

		BSGeometry* origGeom = nullptr;
		NiGeometry* origNiGeom = nullptr;

		if (fmd && fmd->m_model && fmd->m_model->unk10 && fmd->m_model->unk10->unk08)
		{
			_DMESSAGE("Original part node found via facegen extra model data.");
			auto origRootNode = fmd->m_model->unk10->unk08->GetAsNiNode();
			head.headParts.back().physicsFile = DefaultBBP::instance()->scanBBP(origRootNode);
			head.headParts.back().origPartRootNode = origRootNode;
			for (int i = 0; i < origRootNode->m_children.m_size; i++)
			{
				if (origRootNode->m_children.m_data[i])
				{
					const auto geo = origRootNode->m_children.m_data[i]->GetAsBSGeometry();

					if (geo)
					{
						origGeom = geo;
						break;
					}
				}
			}
		}
		else
		{
			_DMESSAGE("No facegen extra model data available, loading original facegeometry.");
			if (!head.npcFaceGeomNode)
			{
				if (skeleton->m_owner && skeleton->m_owner->baseForm)
				{
					auto npc = DYNAMIC_CAST(skeleton->m_owner->baseForm, TESForm, TESNPC);
					if (npc)
					{
						char filePath[MAX_PATH];
						if (TESNPC_GetFaceGeomPath(npc, filePath))
						{
							_DMESSAGE("Loading facegeometry from path %s.", filePath);
							static const int MAX_SIZE = sizeof(NiStream) + 0x200;
							UInt8 niStreamMemory[MAX_SIZE];
							memset(niStreamMemory, 0, MAX_SIZE);
							NiStream* niStream = (NiStream*)niStreamMemory;
							CALL_MEMBER_FN(niStream, ctor)();

							BSResourceNiBinaryStream binaryStream(filePath);
							if (!binaryStream.IsValid())
							{
								_ERROR("Somehow NPC facegeometry was not found.");
								CALL_MEMBER_FN(niStream, dtor)();
							}
							else
							{
								niStream->LoadStream(&binaryStream);
								if (niStream->m_rootObjects.m_data[0])
								{
									auto rootFadeNode = niStream->m_rootObjects.m_data[0]->GetAsBSFadeNode();
									if (rootFadeNode)
									{
										_DMESSAGE("NPC facegeometry root fadeNode found.");
										head.npcFaceGeomNode = rootFadeNode;
									}
									else
										_DMESSAGE("NPC facegeometry root wasn't fadeNode as expected.");

								}
								CALL_MEMBER_FN(niStream, dtor)();
							}
						}
					}
				}
			}
			else
				_DMESSAGE("Using cached facegeometry.");
			if (head.npcFaceGeomNode)
			{
				head.headParts.back().physicsFile = DefaultBBP::instance()->scanBBP(head.npcFaceGeomNode);
				auto obj = findObject(head.npcFaceGeomNode, geometry->m_name);
				if (obj)
				{
					auto ob = obj->GetAsBSGeometry();
					if (ob) origGeom = ob;
					else {
						auto on = obj->GetAsNiGeometry();
						if (on) origNiGeom = on;
					}
				}
			}
		}

		bool hasMerged = false;
		bool hasRenames = false;

		for (int boneIdx = 0; boneIdx < geometry->m_spSkinInstance->m_spSkinData->m_uiBones; boneIdx++)
		{
			BSFixedString boneName("");

			// skin the way the game does via FMD
			if (boneIdx <= 7)
			{
				if (fmd)
					boneName = fmd->bones[boneIdx];
			}

			if (!*boneName.c_str())
			{
				if (origGeom)
				{
					boneName = origGeom->m_spSkinInstance->m_ppkBones[boneIdx]->m_name;
				}
				else if (origNiGeom)
				{
					boneName = origNiGeom->m_spSkinInstance->m_ppkBones[boneIdx]->m_name;
				}
			}

			auto renameIt = this->head.renameMap.find(boneName.c_str());

			if (renameIt != this->head.renameMap.end())
			{
				_DMESSAGE("Found renamed bone %s -> %s.", boneName, renameIt->second->cstr());
				boneName = renameIt->second->cstr();
				hasRenames = true;
			}

			auto boneNode = findNode(this->npc, boneName);

			if (!boneNode && !hasMerged)
			{
				_DMESSAGE("Bone not found on skeleton, trying skeleton merge.");
				if (this->head.headParts.back().origPartRootNode)
				{
					doSkeletonMerge(npc, head.headParts.back().origPartRootNode, head.prefix, head.renameMap);
				}
				else if (this->head.npcFaceGeomNode)
				{
					// Facegen data doesn't have any tree structure to the skeleton. We need to make any new
					// nodes children of the head node, so that they move properly when there's no physics.
					// This case never happens to a lurker skeleton, thus we don't need to test.
					auto headNode = findNode(head.npcFaceGeomNode, "NPC Head [Head]");
					if (headNode)
					{
						NiTransform invTransform;
						headNode->m_localTransform.Invert(invTransform);
						for (int i = 0; i < head.npcFaceGeomNode->m_children.m_arrayBufLen; ++i)
						{
							Ref<NiNode> child = castNiNode(head.npcFaceGeomNode->m_children.m_data[i]);
							// This case never happens to a lurker skeleton, thus we don't need to test.
							if (child && !findNode(npc, child->m_name))
							{
								child->m_localTransform = invTransform * child->m_localTransform;
								head.npcFaceGeomNode->RemoveAt(i);
								headNode->AttachChild(child, false);
							}
						}
					}
					doSkeletonMerge(npc, this->head.npcFaceGeomNode, head.prefix, head.renameMap);
				}
				hasMerged = true;

				auto postMergeRenameIt = this->head.renameMap.find(boneName.c_str());

				if (postMergeRenameIt != this->head.renameMap.end())
				{
					_DMESSAGE("Found renamed bone %s -> %s.", boneName, postMergeRenameIt->second->cstr());
					boneName = postMergeRenameIt->second->cstr();
					hasRenames = true;
				}

				boneNode = findNode(this->npc, boneName);
			}

			if (!boneNode)
			{
				_ERROR("Bone %s not found after skeleton merge, geometry cannot be fully skinned.", boneName);
				continue;
			}

			geometry->m_spSkinInstance->m_ppkBones[boneIdx] = boneNode;
			geometry->m_spSkinInstance->m_worldTransforms[boneIdx] = &boneNode->m_worldTransform;
		}

		geometry->m_spSkinInstance->m_pkRootParent = headNode;

		if (hasRenames)
		{
			for (auto& entry : head.renameMap)
			{
				if ((this->head.headParts.back().origPartRootNode && findObject(this->head.headParts.back().origPartRootNode, entry.first->cstr())) ||
					(this->head.npcFaceGeomNode && findObject(this->head.npcFaceGeomNode, entry.first->cstr())))
				{
					auto findNode = this->head.nodeUseCount.find(entry.first);
					if (findNode != this->head.nodeUseCount.end())
					{
						findNode->second += 1;
						_DMESSAGE("Incrementing use count by 1, it is now %d.", findNode->second);
					}
					else
					{
						this->head.nodeUseCount.insert(std::make_pair(entry.first, 1));
						_DMESSAGE("First use of bone, count 1.");
					}
					head.headParts.back().renamedBonesInUse.insert(entry.first);
				}
			}
		}

		_DMESSAGE("Done skinning part.");
	}
}
