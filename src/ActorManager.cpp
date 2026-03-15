
#include "ActorManager.h"
#include "WeatherManager.h"
#include "dhdtOverrideManager.h"
#include "hdtDefaultBBP.h"
#include "hdtSkyrimPhysicsWorld.h"

namespace hdt
{
	// NiAVObject* Actor::CalculateLOS_1405FD2C0(Actor *aActor, NiPoint3 *aTargetPosition, NiPoint3 *aRayHitPosition, float aViewCone)
	// Used to ray cast from the actor. Will return nonNull if it hits something with position at aTargetPosition.
	// Pass in 2pi to aViewCone to ignore LOS of actor.

	typedef RE::NiAVObject* (*_Actor_CalculateLOS)(RE::Actor* aActor, RE::NiPoint3* aTargetPosition, RE::NiPoint3* aRayHitPosition, float aViewCone);
	typedef bool (*_TESNPC_GetFaceGeomPath)(RE::TESNPC* a_npc, char* a_buf);
	typedef RE::NiStream* (*_NiStream_constructor)(RE::NiStream*);
	typedef RE::NiStream* (*_NiStream_deconstructor)(RE::NiStream*);

	REL::Relocation<_Actor_CalculateLOS> Actor_CalculateLOS{ REL::VariantID(36754, 37770, 0x0605B10) };          // 0x5FD2C0
	REL::Relocation<_TESNPC_GetFaceGeomPath> TESNPC_GetFaceGeomPath{ REL::VariantID(24222, 24726, 0x0372B30) };  // 0x363210
	REL::Relocation<_NiStream_constructor> NiStream_constructor{ REL::VariantID(68971, 70324, 0x0C9EC40) };      // 0xC59690
	REL::Relocation<_NiStream_deconstructor> NiStream_deconstructor{ REL::VariantID(68972, 70325, 0x0C9EEA0) };  // 0xC598F0

	static bool IsHair(RE::TESBoundObject* a_ref)
	{
		if (a_ref) {
			auto bipedForm = skyrim_cast<RE::BGSBipedObjectForm*>(a_ref);
			if (bipedForm) {
				return bipedForm->bipedModelData.bipedObjectSlots.any(RE::BIPED_MODEL::BipedObjectSlot::kHair, RE::BIPED_MODEL::BipedObjectSlot::kLongHair);
			}
		}

		return false;
	};

	class HairVisitor  // public RE::InventoryChanges::IItemChangeVisitor
	{
	public:
		HairVisitor(RE::TESBoundObject*& a_dstObject) :
			_object(a_dstObject)
		{}

		~HairVisitor()
		{}

		RE::BSContainer::ForEachResult Visit(RE::InventoryEntryData* a_entryData)
		{
			if (a_entryData && a_entryData->extraLists && IsHair(a_entryData->object)) {
				for (auto it : *a_entryData->extraLists) {
					if (it) {
						if (it->HasType<RE::ExtraWorn>() || it->HasType<RE::ExtraWornLeft>()) {
							_object = a_entryData->object;
							return RE::BSContainer::ForEachResult::kStop;
						}
					}
				}
			}

			//
			return RE::BSContainer::ForEachResult::kContinue;
		}

	private:
		RE::TESBoundObject*& _object;
	};

	bool isFirstPersonSkeleton(RE::NiNode* npc)
	{
		if (!npc) {
			return false;
		}

		return findNode(npc, "Camera1st [Cam1]") ? true : false;
	}

	RE::NiNode* getNpcNode(RE::NiNode* skeleton)
	{
		// TODO: replace this with a generic skeleton fixing configuration option
		// hardcode an exception for lurker skeletons because they are made incorrectly
		auto shouldFix = false;
		if (skeleton->GetUserData() && skeleton->GetUserData()->GetObjectReference()) {
			auto npcForm = skyrim_cast<RE::TESNPC*>(skeleton->GetUserData()->GetObjectReference());
			if (npcForm && npcForm->race && !strcmp(npcForm->race->skeletonModels[0].GetModel(), "Actors\\DLC02\\BenthicLurker\\Character Assets\\skeleton.nif")) {
				shouldFix = true;
			}
		}

		return findNode(skeleton, shouldFix ? "NPC Root [Root]" : "NPC");
	}

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

	RE::BSEventNotifyControl ActorManager::ProcessEvent(const Events::ArmorAttachEvent* e, RE::BSTEventSource<Events::ArmorAttachEvent>*)
	{
		// No armor is ever attached to a lurker skeleton, thus we don't need to test.
		if (e->skeleton == nullptr || !findNode(e->skeleton, "NPC")) {
			return RE::BSEventNotifyControl::kContinue;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) {
			return RE::BSEventNotifyControl::kContinue;
		}

		fixArmorNameMaps();

		auto& skeleton = getSkeletonData(e->skeleton);
		if (e->hasAttached) {
			// Check override data for current armoraddon
			if (e->skeleton->GetUserData()) {
				auto actor_formID = e->skeleton->GetUserData()->formID;
				if (actor_formID) {
					std::string physics_file_path_override = hdt::Override::OverrideManager::GetSingleton()->checkOverride(actor_formID, skeleton.getArmors().back().physicsFile.first);
					if (!physics_file_path_override.empty()) {
						RE::ConsoleLog::GetSingleton()->Print("[DynamicHDT] -- ArmorAddon %s is overridden ", e->attachedNode->name.c_str());
						skeleton.getArmors().back().physicsFile.first = physics_file_path_override;
					}
				}
			}

			skeleton.attachArmor(e->armorModel, e->attachedNode);
		} else {
			skeleton.addArmor(e->armorModel);
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl ActorManager::ProcessEvent(const Events::ArmorDetachEvent* e, RE::BSTEventSource<Events::ArmorDetachEvent>*)
	{
		if (!e->actor || !e->hasDetached || !instance()->m_disableSMPHairWhenWigEquipped) {
			return RE::BSEventNotifyControl::kContinue;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) {
			return RE::BSEventNotifyControl::kContinue;
		}

		fixArmorNameMaps();

		Skeleton* s = get3rdPersonSkeleton(e->actor);
		setHeadActiveIfNoHairArmor(e->actor, s);

		return RE::BSEventNotifyControl::kContinue;
	}

	// @brief This happens on a closing RaceSex menu, and on 'smp reset'.
	RE::BSEventNotifyControl ActorManager::ProcessEvent(const RE::MenuOpenCloseEvent*, RE::BSTEventSource<RE::MenuOpenCloseEvent>*)
	{
		// The ActorManager members are protected from parallel events by ActorManager.m_lock.
		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) {
			return RE::BSEventNotifyControl::kContinue;
		}

		logger::debug("Processing MenuOpenCloseEvent.");

		fixArmorNameMaps();

		setSkeletonsActive();

		for (auto& i : m_skeletons) {
			i.reloadMeshes();
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl ActorManager::ProcessEvent(const Events::FrameEvent*, RE::BSTEventSource<Events::FrameEvent>*)
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);

		fixArmorNameMaps();

		setSkeletonsActive(true);

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl ActorManager::ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*)
	{
		m_shutdown = true;
		std::lock_guard<decltype(m_lock)> l(m_lock);

		m_skeletons.clear();

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl ActorManager::ProcessEvent(const Events::SkinSingleHeadGeometryEvent* e, RE::BSTEventSource<Events::SkinSingleHeadGeometryEvent>*)
	{
		// This case never happens to a lurker skeleton, thus we don't need to test.
		auto npc = findNode(e->skeleton, "NPC");
		if (npc == nullptr) {
			return RE::BSEventNotifyControl::kContinue;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) {
			return RE::BSEventNotifyControl::kContinue;
		}

		fixArmorNameMaps();

		auto& skeleton = getSkeletonData(e->skeleton);
		skeleton.npc = hdt::make_nismart(getNpcNode(e->skeleton));

		skeleton.processGeometry(e->headNode, e->geometry);

		auto headPartIter = std::find_if(skeleton.head.headParts.begin(), skeleton.head.headParts.end(), [e](const Head::HeadPart& p) {
			return p.headPart == e->geometry;
		});

		if (headPartIter != skeleton.head.headParts.end()) {
			if (headPartIter->origPartRootNode) {
				logger::debug("Renaming nodes in original part {} back.", headPartIter->origPartRootNode->name);

				for (auto& entry : skeleton.head.renameMap) {
					// This case never happens to a lurker skeleton, thus we don't need to test.
					auto node = findNode(headPartIter->origPartRootNode.get(), entry.second->cstr());
					if (node) {
						logger::debug("Rename node {} -> {}.", entry.second->cstr(), entry.first->cstr());
						setNiNodeName(node, entry.first->cstr());
					}
				}
			}
			headPartIter->origPartRootNode = nullptr;
		}

		if (!skeleton.head.isFullSkinning) {
			skeleton.scanHead();
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl ActorManager::ProcessEvent(const Events::SkinAllHeadGeometryEvent* e, RE::BSTEventSource<Events::SkinAllHeadGeometryEvent>*)
	{
		// This case never happens to a lurker skeleton, thus we don't need to test.
		auto npc = findNode(e->skeleton, "NPC");
		if (!npc) {
			return RE::BSEventNotifyControl::kContinue;
		}

		std::lock_guard<decltype(m_lock)> l(m_lock);
		if (m_shutdown) {
			return RE::BSEventNotifyControl::kContinue;
		}

		fixArmorNameMaps();

		auto& skeleton = getSkeletonData(e->skeleton);
		skeleton.npc = hdt::make_nismart(npc);
		if (e->skeleton->GetUserData()) {
			skeleton.skeletonOwner.reset(e->skeleton->GetUserData());
		}

		if (e->hasSkinned) {
			skeleton.scanHead();
			skeleton.head.isFullSkinning = false;
			if (skeleton.head.npcFaceGeomNode) {
				logger::debug("NPC face geometry no longer needed, clearing reference.");
				skeleton.head.npcFaceGeomNode = nullptr;
			}
		} else {
			skeleton.head.isFullSkinning = true;
		}

		return RE::BSEventNotifyControl::kContinue;
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

	void ActorManager::fixArmorNameMaps()
	{
		auto& skeletons = instance()->getSkeletons();
		for (auto& skeleton : skeletons)
			if (skeleton.mustFixOneArmorMap) {
				auto& armors = skeleton.getArmors();
				for (auto& armor : armors) {
					if (armor.mustFixNameMap) {
						if (armor.armorWorn)
							if (armor.armorWorn->name.size()) {
								std::string armorNewMeshName(armor.armorWorn->name);
								if (!armorNewMeshName.empty() && armor.armorCurrentMeshName.compare(armorNewMeshName) != 0) {
									auto& armorNameMap = armor.physicsFile.second;
									hdt::DefaultBBP::NameMap_t tempNameMap;
									for (auto& [setName, set] : armorNameMap)
										// ... and we found the old mesh name in the armor nameMap,...
										if (armor.armorCurrentMeshName.compare(setName) == 0) {
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

	// @brief To avoid calculating headparts when they're hidden by a wig,
	// we mark the head as not active when there is an armor on hair or long hair slots.
	// We do this during the attach/detach armor events, and on the events leading to scanning the head.
	// Then when checking which skeletons are active to calculate the frame,
	// we only allow the activation of headparts that are on active heads.
	// @param Actor * actor is expected not null.
	void ActorManager::setHeadActiveIfNoHairArmor([[maybe_unused]] RE::Actor* actor, [[maybe_unused]] Skeleton* skeleton)
	{
		RE::TESBoundObject* ref = nullptr;

		//
		RE::ExtraContainerChanges* extraContainerchanges = actor->extraList.GetByType<RE::ExtraContainerChanges>();
		if (extraContainerchanges && extraContainerchanges->changes && extraContainerchanges->changes->entryList) {
			//
			HairVisitor visitor(ref);

			//
			for (auto it : *extraContainerchanges->changes->entryList) {
				if (visitor.Visit(it) == RE::BSContainer::ForEachResult::kStop) {
					break;
				}
			}
		}

		//
		if (skeleton) {
			skeleton->head.isActive = ref == nullptr;
		}
	}

	inline RE::NiNode* ActorManager::getCameraNode()
	{
		return RE::PlayerCamera::GetSingleton()->cameraRoot.get();
	}

	// @brief This function is called by different events, with different locking needs, and is therefore extracted from the events.
	void ActorManager::setSkeletonsActive(const bool updateMetrics)
	{
		if (m_shutdown)
			return;

		// We get the player character and its cell.
		// TODO Isn't there a more performing way to find the PC?? A singleton? And if it's the right way, why isn't it in utils functions?
		const auto& playerCharacter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [](const Skeleton& s) { return s.isPlayerCharacter(); });
		auto playerCell = (playerCharacter != m_skeletons.end() && playerCharacter->skeleton->parent) ? playerCharacter->skeleton->parent->parent : nullptr;

		const auto cameraNode = getCameraNode();
		if (!cameraNode)
			return;

		// We get the camera, its position and orientation.
		const auto cameraTransform = cameraNode->world;
		const auto cameraPosition = cameraTransform.translate;
		const auto cameraOrientation = cameraTransform.rotate * RE::NiPoint3(0., 1., 0.);  // The camera matrix is relative to the world.
		this->m_cameraPositionDuringFrame = cameraPosition;

		std::for_each(m_skeletons.begin(), m_skeletons.end(), [&](Skeleton& skel) {
			skel.calculateDistanceAndOrientationDifferenceFromSource(cameraPosition, cameraOrientation);
		});

		// We sort the skeletons depending on the angle and distance.
		std::sort(m_skeletons.begin(), m_skeletons.end(), [](auto&& a_lhs, auto&& a_rhs) {
			auto cr = a_rhs.m_cosAngleFromCameraDirectionTimesSkeletonDistance;
			auto cl = a_lhs.m_cosAngleFromCameraDirectionTimesSkeletonDistance;
			auto dr = a_rhs.m_distanceFromCamera2;
			auto dl = a_lhs.m_distanceFromCamera2;
			return
				// If one of the skeletons is at distance zero (1st person player) from the camera
				(btFuzzyZero(dl) || btFuzzyZero(dr))
					// then it is first.
					?
					(dl < dr)
					// If one of the skeletons is exactly on the side of the camera (cos = 0)
					:
					(btFuzzyZero(cl) || btFuzzyZero(cr))
						// then it is last.
						?
					abs(cl) > abs(cr)
					// If both are on the same side of the camera (product of cos > 0):
					// we want first the smallest angle (so the highest cosinus), and the smallest distance,
					// so we want the smallest distance / cosinus.
					// cl = cosinus * distance, dl = distance� => distance / cosinus = dl/cl
					// So we want dl/cl < dr/cr.
					// Moreover, this test manages the case where one of the skeletons is behind the camera and the other in front of the camera too;
					// the one behind the camera is last (the one with cos(angle) = cr < 0).
					:
					(dl * cr < dr * cl);
		});

		// We set which skeletons are active and we count them.
		activeSkeletons = 0;
		for (auto& i : m_skeletons) {
			if (i.skeleton->_refCount == 1) {
				i.clear();
				i.skeleton = nullptr;
			} else if (i.hasPhysics && i.updateAttachedState(playerCell, activeSkeletons >= maxActiveSkeletons)) {
				activeSkeletons++;
				//check wind obstructions
				const auto world = SkyrimPhysicsWorld::get();
				const auto wind = getWindDirection();
				if (world->m_enableWind && wind && !(btFuzzyZero(hdt::magnitude(*wind)))) {
					const auto owner = skyrim_cast<RE::Actor*>(i.skeletonOwner.get());
					if (owner) {
						auto windray = *wind * -1;  // reverse wind raycast to find obstruction
						RE::NiPoint3 hitLocation;
						//Raycast for object in direction of wind
						const auto object = Actor_CalculateLOS(owner, &windray, &hitLocation, 6.28);
						if (object) {
							// object found
							auto diff = (owner->data.location - hitLocation);
							diff.z = 0;  //remove z component difference
							const auto dist = hdt::magnitude(diff);
							// wind is a linear reduction, with a minimum floor since objects may have a minimum distance
							// windfactor = 0 when dist <= m_distanceForNoWind, = 1 when dist >= m_distanceForMaxWind, and is linear with dist between these 2 values.
							const auto windFactor = std::clamp((dist - world->m_distanceForNoWind) / (world->m_distanceForMaxWind - world->m_distanceForNoWind), 0.f, 1.f);
							if (!btFuzzyZero(windFactor - i.getWindFactor())) {
								logger::debug("{} blocked by {} with distance {:.2f}; setting windFactor {:.2f}.", i.name(), object->name, dist, windFactor);
								i.updateWindFactor(windFactor);
							}
						}
					} else {
						logger::debug("{} is active skeleton, but failed to cast to Actor, no wind obstruction check possible.", i.name());
					}
				}
			}
		}

		m_skeletons.erase(std::remove_if(m_skeletons.begin(), m_skeletons.end(), [](Skeleton& i) { return !i.skeleton; }), m_skeletons.end());

		for (auto& i : m_skeletons) {
			i.cleanArmor();
			i.cleanHead();
		}

		const auto world = SkyrimPhysicsWorld::get();

		// We share the same doMetrics condition here and in hdtSkyrimPhysicsWorld to avoid any gap between both.
		// The evaluation is done here rather than in hdtSkyrimPhysicsWorld because this event is called first.
		world->m_doMetrics = updateMetrics &&                     // do not do metrics on a MenuOpenCloseEvent
		                     !world->isSuspended() &&             // do not do metrics while paused
		                     frameCount++ % world->min_fps == 0;  // check every min-fps frames (i.e., a stable 60 fps should wait for 1 second)

		if (world->m_doMetrics) {
			const auto averageProcessingTimeInMainLoop = world->m_averageSMPProcessingTimeInMainLoop;
			// 30% of processing time is in hdt per profiling;
			// Setting it higher provides more time for hdt processing and can activate more skeletons.
			const auto target_time = world->m_timeTick * world->m_percentageOfFrameTime;
			auto averageTimePerSkeletonInMainLoop = 0.f;
			if (activeSkeletons > 0) {
				averageTimePerSkeletonInMainLoop = averageProcessingTimeInMainLoop / activeSkeletons;
			}

			logger::trace(
				"msecs/activeSkeleton {:.2f} activeSkeletons/maxActive/total {}/{}/{} processTimeInMainLoop/targetTime {:.2f}/{:.2f}",
				averageTimePerSkeletonInMainLoop,
				activeSkeletons,
				maxActiveSkeletons,
				m_skeletons.size(),
				averageProcessingTimeInMainLoop,
				target_time);

			if (m_autoAdjustMaxSkeletons) {
				maxActiveSkeletons += target_time > averageProcessingTimeInMainLoop ? 2 : -2;
				// clamp the value to the m_maxActiveSkeletons value
				maxActiveSkeletons = std::clamp(maxActiveSkeletons, 1, m_maxActiveSkeletons);
				frameCount = 1;
			} else if (maxActiveSkeletons != m_maxActiveSkeletons)
				maxActiveSkeletons = m_maxActiveSkeletons;
		}
	}

	void ActorManager::PhysicsItem::setPhysics(RE::BSTSmartPointer<SkyrimSystem>& system, bool active)
	{
		clearPhysics();
		m_physics = system;
		if (active) {
			SkyrimPhysicsWorld::get()->addSkinnedMeshSystem(m_physics.get());
		}
	}

	void ActorManager::PhysicsItem::clearPhysics()
	{
		if (state() == ItemState::e_Active) {
			m_physics->m_world->removeSkinnedMeshSystem(m_physics.get());
		}
		m_physics = nullptr;
	}

	ActorManager::ItemState ActorManager::PhysicsItem::state() const
	{
		return m_physics ? (m_physics->m_world ? ItemState::e_Active : ItemState::e_Inactive) : ItemState::e_NoPhysics;
	}

	const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& ActorManager::PhysicsItem::meshes() const
	{
		return m_physics->meshes();
	}

	void ActorManager::PhysicsItem::updateActive(bool active)
	{
		if (active && state() == ItemState::e_Inactive) {
			SkyrimPhysicsWorld::get()->addSkinnedMeshSystem(m_physics.get());
		} else if (!active && state() == ItemState::e_Active) {
			m_physics->m_world->removeSkinnedMeshSystem(m_physics.get());
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

	bool ActorManager::skeletonNeedsParts(RE::NiNode* skeleton)
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

	ActorManager::Skeleton& ActorManager::getSkeletonData(RE::NiNode* skeleton)
	{
		auto iter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [=](Skeleton& i) {
			return i.skeleton == skeleton;
		});

		if (iter != m_skeletons.end()) {
			return *iter;
		}

		if (!isFirstPersonSkeleton(skeleton)) {
			auto ownerIter = std::find_if(m_skeletons.begin(), m_skeletons.end(), [=](Skeleton& i) {
				return !isFirstPersonSkeleton(i.skeleton.get()) && i.skeletonOwner && skeleton->GetUserData() && i.skeletonOwner.get() == skeleton->GetUserData();
			});

			if (ownerIter != m_skeletons.end()) {
				logger::debug("New skeleton found for formid {:08X}.", skeleton->GetUserData()->formID);
				ownerIter->cleanHead(true);
			}
		}

		m_skeletons.push_back(Skeleton());
		m_skeletons.back().skeleton = hdt::make_nismart(skeleton);
		return m_skeletons.back();
	}

	ActorManager::Skeleton* ActorManager::get3rdPersonSkeleton(RE::Actor* actor)
	{
		for (auto& i : m_skeletons) {
			const auto owner = skyrim_cast<RE::Actor*>(i.skeletonOwner.get());
			if (actor == owner && i.skeleton && !isFirstPersonSkeleton(i.skeleton.get())) {
				return &i;
			}
		}
		return 0;
	}

	void ActorManager::Skeleton::doSkeletonMerge(RE::NiNode* dst, RE::NiNode* src, IString* prefix, std::unordered_map<IDStr, IDStr>& map)
	{
		const auto& children = src->GetChildren();

		for (uint16_t i = 0; i < children.size(); ++i) {
			auto srcChild = castNiNode(children[i].get());
			if (!srcChild)
				continue;

			if (!srcChild->name.size()) {
				doSkeletonMerge(dst, srcChild, prefix, map);
				continue;
			}

			// FIXME: This was previously only in doHeadSkeletonMerge.
			// But surely non-head skeletons wouldn't have this anyway?
			if (!strcmp(srcChild->name.c_str(), "BSFaceGenNiNodeSkinned")) {
				logger::debug("Skipping facegen ninode in skeleton merge.");
				continue;
			}

			// TODO check it's not a lurker skeleton
			auto dstChild = findNode(dst, srcChild->name);
			if (dstChild) {
				doSkeletonMerge(dstChild, srcChild, prefix, map);
			} else {
				dst->AttachChild(cloneNodeTree(srcChild, prefix, map), false);
			}
		}
	}

	RE::NiNode* ActorManager::Skeleton::cloneNodeTree(RE::NiNode* src, IString* prefix, std::unordered_map<IDStr, IDStr>& map)
	{
		//
		RE::NiCloningProcess c;

		//
		c.appendChar = '$';
		c.copyType = 1;
		c.scale = { 1.0f, 1.0f, 1.0f };

		auto ret = static_cast<RE::NiNode*>(src->CreateClone(c));
		src->ProcessClone(c);

		// FIXME: cloneHeadNodeTree just did this for ret, not both. Don't know if that matters. Armor parts need it on both.
		renameTree(src, prefix, map);
		renameTree(ret, prefix, map);

		return ret;
	}

	void ActorManager::Skeleton::renameTree(RE::NiNode* root, IString* prefix, std::unordered_map<IDStr, IDStr>& map)
	{
		if (root->name.size()) {
			std::string newName(prefix->cstr(), prefix->size());
			newName += root->name;
			if (map.insert(std::make_pair<IDStr, IDStr>(root->name.c_str(), newName)).second) {
				logger::debug("Rename Bone {} -> {}.", root->name, newName.c_str());
			}

			setNiNodeName(root, newName.c_str());
		}

		auto& children = root->GetChildren();
		for (uint16_t i = 0; i < children.size(); ++i) {
			auto child = castNiNode(children[i].get());
			if (child) {
				renameTree(child, prefix, map);
			}
		}
	}

	void ActorManager::Skeleton::doSkeletonClean(RE::NiNode* dst, IString* prefix)
	{
		auto& children = dst->GetChildren();

		for (uint16_t i = children.size(); i-- > 0;) {
			auto child = castNiNode(children[i].get());
			if (!child)
				continue;

			if (child->name.size() && !strncmp(child->name.data(), prefix->cstr(), prefix->size())) {
				dst->DetachChildAt2(i++);
			} else {
				doSkeletonClean(child, prefix);
			}
		}
	}

	// returns the name of the skeleton owner
	std::string ActorManager::Skeleton::name()
	{
		if (skeleton->GetUserData() && skeleton->GetUserData()->GetObjectReference()) {
			auto bname = skyrim_cast<RE::TESFullName*>(skeleton->GetUserData()->GetObjectReference());
			if (bname) {
				return bname->GetFullName();
			}
		}
		return "";
	}

	// Logs a warning once per NIF path when VR NiStream Type B stubs are found.
	// Uses a static set to avoid duplicate warnings for the same NIF across frames.
	static void logBrokenNifOnce(const char* nifPath, RE::NiAVObject* root)
	{
		if (!REL::Module::IsVR() || !nifPath || !root)
			return;
		static std::unordered_set<std::string> warned;
		if (warned.count(nifPath))
			return;
		std::vector<RE::NiAVObject*> stack = { root };
		while (!stack.empty()) {
			auto obj = stack.back();
			stack.pop_back();
			if (!obj || !isValidNiObject(obj))
				continue;
			if (isVRNiStreamStub(obj)) {
				warned.insert(nifPath);
				logger::warn(
					"[VR NiStream] NIF '{}' contains SE-format blocks VR cannot fully instantiate "
					"(Type B stubs, broken vtable[43]). Run through Cathedral Assets Optimizer (CAO) for Skyrim VR.",
					nifPath);
				return;
			}
			auto node = obj->AsNode();
			if (node)
				for (auto& c : node->GetChildren())
					if (c)
						stack.push_back(c.get());
		}
	}

	void ActorManager::Skeleton::addArmor(RE::NiNode* armorModel)
	{
		if (armorModel)
			logBrokenNifOnce(armorModel->name.c_str(), armorModel);

		IDType id = armors.size() ? armors.back().id + 1 : 0;
		auto prefix = armorPrefix(id);
		// FIXME we probably could simplify this by using findNode as surely we don't merge Armors with lurkers skeleton?
		npc = hdt::make_nismart(getNpcNode(skeleton.get()));
		auto physicsFile = DefaultBBP::instance()->scanBBP(armorModel);

		armors.push_back(Armor());
		armors.back().id = id;
		armors.back().prefix = prefix;
		armors.back().physicsFile = physicsFile;

		doSkeletonMerge(npc.get(), armorModel, prefix, armors.back().renameMap);
	}

	void ActorManager::Skeleton::attachArmor([[maybe_unused]] RE::NiNode* armorModel, RE::NiAVObject* attachedNode)
	{
		if (armors.size() == 0 || armors.back().hasPhysics()) {
			logger::trace("Not attaching armor - no record or physics already exists");
		}

		Armor& armor = armors.back();

		// The name of the attachedNode provided here will have been changed by the Skyrim exe between this event and the next.
		armor.armorWorn = hdt::make_nismart(attachedNode);
		// That's why we set here the need to fix this armor in fixArmorNameMaps() (see its comment)
		// to avoid this name change breaking processes like 'smp reset' when looking for the armor name in the armor nameMap.
		armor.armorCurrentMeshName = attachedNode->name.size() ? attachedNode->name : "";
		armor.mustFixNameMap = true;
		mustFixOneArmorMap = true;

		if (!isFirstPersonSkeleton(skeleton.get())) {
			// FIXME we probably could simplify this by using findNode as surely we don't attach Armors to lurkers skeleton?
			auto renameMap = armor.renameMap;
			auto system = SkyrimSystemCreator().createOrUpdateSystem(getNpcNode(skeleton.get()), attachedNode, &armor.physicsFile, std::move(renameMap), nullptr);
			if (system) {
				armor.setPhysics(system, isActive);
				hasPhysics = true;
			}
		}

		if (instance()->m_disableSMPHairWhenWigEquipped && skeleton && skeleton->GetUserData()) {
			RE::Actor* actor = RE::TESForm::LookupByID<RE::Actor>(skeleton->GetUserData()->formID);
			if (actor) {
				setHeadActiveIfNoHairArmor(actor, this);
			}
		}
	}

	void ActorManager::Skeleton::cleanArmor()
	{
		for (auto& i : armors) {
			if (!i.armorWorn) {
				continue;
			}

			if (i.armorWorn->parent) {
				continue;
			}

			i.clearPhysics();
			if (npc) {
				doSkeletonClean(npc.get(), i.prefix.get());
			}

			i.prefix = nullptr;
		}

		armors.erase(std::remove_if(armors.begin(), armors.end(), [](Armor& i) { return !i.prefix; }), armors.end());
	}

	void ActorManager::Skeleton::cleanHead(bool cleanAll)
	{
		for (auto& headPart : head.headParts) {
			if (!headPart.headPart->parent || cleanAll) {
				if (cleanAll) {
					logger::debug("Cleaning headpart {} due to clean all.", headPart.headPart->name);
				} else {
					logger::debug("Headpart {} disconnected.", headPart.headPart->name);
				}

				auto renameIt = this->head.renameMap.begin();
				while (renameIt != this->head.renameMap.end()) {
					bool erase = false;

					if (headPart.renamedBonesInUse.count(renameIt->first) != 0) {
						auto findNode = this->head.nodeUseCount.find(renameIt->first);
						if (findNode != this->head.nodeUseCount.end()) {
							findNode->second -= 1;
							logger::debug("Decrementing use count by 1, it is now {}.", findNode->second);
							if (findNode->second <= 0) {
								logger::debug("Node no longer in use, cleaning from skeleton.");
								auto removeObj = findObject(npc.get(), renameIt->second->cstr());
								if (removeObj) {
									logger::debug("Found node {}, removing.", removeObj->name);
									auto parent = removeObj->parent;
									if (parent) {
										parent->DetachChild2(removeObj);
										removeObj->DecRefCount();
									}
								}
								this->head.nodeUseCount.erase(findNode);
								erase = true;
							}
						}
					}

					if (erase) {
						renameIt = this->head.renameMap.erase(renameIt);
					} else {
						++renameIt;
					}
				}

				headPart.headPart = nullptr;
				headPart.origPartRootNode = nullptr;
				headPart.clearPhysics();
				headPart.renamedBonesInUse.clear();
			}
		}

		head.headParts.erase(std::remove_if(head.headParts.begin(), head.headParts.end(), [](Head::HeadPart& i) { return !i.headPart; }), head.headParts.end());
	}

	void ActorManager::Skeleton::clear()
	{
		std::for_each(armors.begin(), armors.end(), [](Armor& armor) { armor.clearPhysics(); });
		SkyrimPhysicsWorld::get()->removeSystemByNode(npc.get());
		cleanHead();
		head.headParts.clear();
		head.headNode = nullptr;
		armors.clear();
	}

	void ActorManager::Skeleton::calculateDistanceAndOrientationDifferenceFromSource(RE::NiPoint3 sourcePosition, RE::NiPoint3 sourceOrientation)
	{
		if (isPlayerCharacter()) {
			m_distanceFromCamera2 = 0.f;
			return;
		}

		auto pos = position();
		if (!pos.has_value()) {
			m_distanceFromCamera2 = std::numeric_limits<float>::max();
			return;
		}

		// We calculate the vector between camera and the skeleton feet.
		const auto camera2SkeletonVector = pos.value() - sourcePosition;
		// This is the distance (squared) between the camera and the skeleton feet.
		m_distanceFromCamera2 = camera2SkeletonVector.x * camera2SkeletonVector.x + camera2SkeletonVector.y * camera2SkeletonVector.y + camera2SkeletonVector.z * camera2SkeletonVector.z;
		// This is |camera2SkeletonVector|*cos(angle between both vectors)
		m_cosAngleFromCameraDirectionTimesSkeletonDistance = camera2SkeletonVector.x * sourceOrientation.x + camera2SkeletonVector.y * sourceOrientation.y + camera2SkeletonVector.z * sourceOrientation.z;
	}

	// Is called to print messages only
	bool ActorManager::Skeleton::checkPhysics()
	{
		hasPhysics = false;
		std::for_each(armors.begin(), armors.end(), [=](Armor& armor) {
			if (armor.state() != ItemState::e_NoPhysics) {
				hasPhysics = true;
			}
		});

		if (!hasPhysics) {
			std::for_each(head.headParts.begin(), head.headParts.end(), [=](Head::HeadPart& headPart) {
				if (headPart.state() != ItemState::e_NoPhysics) {
					hasPhysics = true;
				}
			});
		}

		logger::trace("{} isDrawn {}", name(), hasPhysics);

		return hasPhysics;
	}

	bool ActorManager::Skeleton::isActiveInScene() const
	{
		// TODO: do this better
		// When entering/exiting an interior, NPCs are detached from the scene but not unloaded, so we need to check two levels up.
		// This properly removes exterior cell armors from the physics world when entering an interior, and vice versa.
		return skeleton->parent && skeleton->parent->parent && skeleton->parent->parent->parent;
	}

	bool ActorManager::Skeleton::isPlayerCharacter() const
	{
		constexpr uint32_t playerFormID = 0x14;
		return skeletonOwner.get() == RE::PlayerCharacter::GetSingleton() || (skeleton->GetUserData() && skeleton->GetUserData()->formID == playerFormID);
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
		const auto owner = skyrim_cast<RE::Actor*>(this->skeletonOwner.get());
		if (owner) {
			RE::NiPoint3 hitLocation;
			const auto object = Actor_CalculateLOS(owner, &(i->m_cameraPositionDuringFrame), &hitLocation, 6.28);
			return object ? false : true;  // If object, we hit something on the path
		}
		return true;  // should never happen, a skeleton without owner?
	}

	std::optional<RE::NiPoint3> ActorManager::Skeleton::position() const
	{
		if (npc) {
			// This works for lurker skeletons.
			auto rootNode = findNode(npc.get(), "NPC Root [Root]");
			if (rootNode) {
				return std::optional<RE::NiPoint3>(rootNode->world.translate);
			}
		}

		return std::optional<RE::NiPoint3>();
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

	bool ActorManager::Skeleton::updateAttachedState(const RE::NiNode* playerCell, bool deactivate = false)
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
		else if (isActiveInScene() || skeleton->parent && skeleton->parent->parent == playerCell) {
			if (isPlayerCharacter()) {
				// That setting defines whether we don't set the PC skeleton as active
				// when it is in 1st person view, to avoid calculating physics uselessly.
				if (!(instance()->m_disable1stPersonViewPhysics                                                                                    // disabling?
						&& RE::PlayerCamera::GetSingleton()->currentState == RE::PlayerCamera::GetSingleton()->GetRuntimeData().cameraStates[0]))  // 1st person view
				{
					isActive = true;
					state = SkeletonState::e_ActiveIsPlayer;
				}
			} else if (isInPlayerView()) {
				isActive = true;
				state = SkeletonState::e_ActiveNearPlayer;
			} else
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
		for (auto& i : armors) {
			i.clearPhysics();
			if (!isFirstPersonSkeleton(skeleton.get())) {
				auto renameMap = i.renameMap;
				auto system = SkyrimSystemCreator().createOrUpdateSystem(npc.get(), i.armorWorn.get(), &i.physicsFile, std::move(renameMap), nullptr);
				if (system) {
					i.setPhysics(system, isActive);
					hasPhysics = true;
				}
			}
		}
		scanHead();
	}

	void ActorManager::Skeleton::scanHead()
	{
		if (isFirstPersonSkeleton(this->skeleton.get())) {
			logger::debug("Not scanning head of first person skeleton.");
			return;
		}

		if (!this->head.headNode) {
			logger::debug("Actor has no head node.");
			return;
		}

		std::unordered_set<std::string> physicsDupes;

		if (instance()->m_disableSMPHairWhenWigEquipped && skeleton && skeleton->GetUserData()) {
			RE::Actor* actor = RE::TESForm::LookupByID<RE::Actor>(skeleton->GetUserData()->formID);
			if (actor) {
				setHeadActiveIfNoHairArmor(actor, this);
			}
		}

		for (auto& headPart : this->head.headParts) {
			// always regen physics for all head parts
			headPart.clearPhysics();

			if (headPart.physicsFile.first.empty()) {
				logger::debug("No physics file for headpart {}.", headPart.headPart->name);
				continue;
			}

			if (physicsDupes.count(headPart.physicsFile.first)) {
				logger::debug("Previous head part generated physics system for file {}, skipping.", headPart.physicsFile.first.c_str());
				continue;
			}

			logger::debug("Try create system for headpart {} physics file {}.", headPart.headPart->name, headPart.physicsFile.first.c_str());
			physicsDupes.insert(headPart.physicsFile.first);
			auto renameMap = this->head.renameMap;
			auto system = SkyrimSystemCreator().createOrUpdateSystem(npc.get(), this->head.headNode.get(), &headPart.physicsFile, std::move(renameMap), nullptr);
			if (system) {
				logger::debug("Success.");
				headPart.setPhysics(system, isActive);
				hasPhysics = true;
			}
		}
	}

	void ActorManager::Skeleton::processGeometry(RE::BSFaceGenNiNode* headNode, RE::BSGeometry* geometry)
	{
		if (this->head.headNode && this->head.headNode != headNode) {
			logger::debug("Completely new head attached to skeleton, clearing tracking.");
			for (auto& headPart : this->head.headParts) {
				headPart.clearPhysics();
				headPart.headPart = nullptr;
				headPart.origPartRootNode = nullptr;
			}

			this->head.headParts.clear();

			if (npc)
				doSkeletonClean(npc.get(), this->head.prefix.get());

			this->head.prefix = nullptr;
			this->head.headNode = nullptr;
			this->head.renameMap.clear();
			this->head.nodeUseCount.clear();
		}

		// clean swapped out headparts
		cleanHead();

		this->head.headNode = hdt::make_nismart(headNode);
		++this->head.id;
		this->head.prefix = headPrefix(this->head.id);

		auto it = std::find_if(this->head.headParts.begin(), this->head.headParts.end(), [geometry](const Head::HeadPart& p) {
			return p.headPart == geometry;
		});

		if (it != this->head.headParts.end()) {
			logger::debug("Geometry is already added as head part.");
			return;
		}

		this->head.headParts.push_back(Head::HeadPart());

		head.headParts.back().headPart = hdt::make_nismart(geometry);
		head.headParts.back().clearPhysics();

		// Skinning
		logger::debug("Skinning geometry to skeleton.");

		if (!geometry->GetGeometryRuntimeData().skinInstance || !geometry->GetGeometryRuntimeData().skinInstance->skinData) {
			logger::error("Geometry is missing skin instance - how?");
			return;
		}

		auto fmd = geometry->GetExtraData<RE::BSFaceGenModelExtraData>("FMD");

		RE::BSGeometry* origGeom = nullptr;
		RE::NiGeometry* origNiGeom = nullptr;

		if (fmd && fmd->m_model && fmd->m_model->modelMeshData && fmd->m_model->modelMeshData->faceNode) {
			logger::debug("Original part node found via facegen extra model data.");
			auto origRootNode = fmd->m_model->modelMeshData->faceNode->AsNode();
			head.headParts.back().physicsFile = DefaultBBP::instance()->scanBBP(origRootNode);
			head.headParts.back().origPartRootNode = hdt::make_nismart(origRootNode);

			auto& children = origRootNode->GetChildren();
			for (uint16_t i = 0; i < children.size(); i++) {
				if (children[i]) {
					const auto geo = children[i]->AsGeometry();

					if (geo) {
						origGeom = geo;
						break;
					}
				}
			}
		} else {
			logger::debug("No facegen extra model data available, loading original facegeometry.");
			if (!head.npcFaceGeomNode && !head.npcFaceGeomNodeBroken) {
				if (skeleton->GetUserData() && skeleton->GetUserData()->GetObjectReference()) {
					auto skeletonNpc = skyrim_cast<RE::TESNPC*>(skeleton->GetUserData()->GetObjectReference());
					if (skeletonNpc) {
						char filePath[MAX_PATH];
						if (TESNPC_GetFaceGeomPath(skeletonNpc, filePath)) {
							logger::debug("Loading facegeometry from path {}.", filePath);
							uint8_t niStreamMemory[sizeof(RE::NiStream)];
							memset(niStreamMemory, 0, sizeof(RE::NiStream));
							RE::NiStream* niStream = (RE::NiStream*)niStreamMemory;
							NiStream_constructor(niStream);

							RE::BSResourceNiBinaryStream binaryStream(filePath);
							if (!binaryStream.good()) {
								logger::error("Somehow NPC facegeometry was not found.");
								NiStream_deconstructor(niStream);
							} else {
								niStream->Load1(&binaryStream);
								if (niStream->topObjects[0]) {
									auto rootFadeNode = niStream->topObjects[0]->AsFadeNode();
									if (rootFadeNode) {
										logger::debug("NPC facegeometry root fadeNode found.");
										logBrokenNifOnce(filePath, rootFadeNode);
										// VR: NiSkinInstance::LinkObject fails to resolve internal bone refs,
										// storing the bone name as a raw char* instead of a resolved NiNode*.
										// Bone NiNodes are self-contained in the face geometry NIF, so resolve
										// them now by name lookup against the loaded tree.
										// Must run before NiStream_deconstructor while the tree is live.
										if (REL::Module::IsVR()) {
											auto& ch = rootFadeNode->GetChildren();
											for (std::uint32_t ci = 0; ci < ch.size(); ++ci) {
												auto faceChild = ch[ci].get();
												if (!faceChild || !isValidNiObject(faceChild))
													continue;
												auto faceGeo = faceChild->AsGeometry();
												if (!faceGeo)
													continue;
												const auto& grd = faceGeo->GetGeometryRuntimeData();
												if (!grd.skinInstance || !grd.skinInstance->skinData)
													continue;
												std::uint32_t vrResolved = 0, vrFailed = 0;
												for (std::uint32_t bi = 0; bi < grd.skinInstance->skinData->bones; ++bi) {
													auto bone = grd.skinInstance->bones[bi];
													if (!bone || isValidNiObject(bone))
														continue;
													// char* case: bone pointer is canonical but its bytes are not a valid vtable.
													// Guard against truly non-canonical addresses before reading as a string.
													if (reinterpret_cast<uintptr_t>(bone) > kCanonicalUserSpaceMax)
														continue;
													const char* name = reinterpret_cast<const char*>(bone);
													auto result = findNode(rootFadeNode, RE::BSFixedString(name));
													grd.skinInstance->bones[bi] = result;
													if (result)
														++vrResolved;
													else {
														++vrFailed;
														logger::warn("VR bone fix '{}': bone[{}] '{}' not found in NIF tree.", faceGeo->name.c_str(), bi, name);
													}
												}
												if (vrResolved || vrFailed)
													logger::info("VR bone fix '{}': resolved {}/{} unresolved bone refs in '{}'.", faceGeo->name.c_str(), vrResolved, vrResolved + vrFailed, filePath);
											}
										}
										// Detect remaining unresolvable bone refs (non-null, non-canonical pointers).
										bool brokenBoneRefs = false;
										auto& faceCh = rootFadeNode->GetChildren();
										for (std::uint32_t ci = 0; ci < faceCh.size() && !brokenBoneRefs; ++ci) {
											auto faceChild = faceCh[ci].get();
											if (!faceChild)
												continue;
											if (!isValidNiObject(faceChild)) {
												brokenBoneRefs = true;
												break;
											}
											auto faceGeo = faceChild->AsGeometry();
											if (!faceGeo)
												continue;
											const auto& fgrd = faceGeo->GetGeometryRuntimeData();
											if (!fgrd.skinInstance || !fgrd.skinInstance->skinData)
												continue;
											for (std::uint32_t bi = 0; bi < fgrd.skinInstance->skinData->bones && !brokenBoneRefs; ++bi) {
												auto fBone = fgrd.skinInstance->bones[bi];
												if (fBone && !isValidNiObject(fBone))
													brokenBoneRefs = true;
											}
										}
										if (brokenBoneRefs) {
											logger::warn(
												"processGeometry: NPC facegeometry '{}' has remaining unresolvable "
												"bone refs after VR fix pass. Skipping facegeometry-based bone lookup to avoid crashes.",
												filePath);
											head.npcFaceGeomNodeBroken = true;
										} else
											head.npcFaceGeomNode = hdt::make_nismart(rootFadeNode);
									} else
										logger::debug("NPC facegeometry root wasn't fadeNode as expected.");
								}
								NiStream_deconstructor(niStream);
							}
						}
					}
				}
			} else
				logger::debug("Using cached facegeometry.");
			if (head.npcFaceGeomNode) {
				head.headParts.back().physicsFile = DefaultBBP::instance()->scanBBP(head.npcFaceGeomNode.get());
				auto obj = findObject(head.npcFaceGeomNode.get(), geometry->name);
				if (obj) {
					auto ob = obj->AsGeometry();
					if (ob) {
						origGeom = ob;
					} else {
						auto on = obj->AsNiGeometry();
						if (on) {
							origNiGeom = on;
						}
					}
				}
			}
		}

		bool hasMerged = false;
		bool hasRenames = false;

		for (uint32_t boneIdx = 0; boneIdx < geometry->GetGeometryRuntimeData().skinInstance->skinData->bones; boneIdx++) {
			RE::BSFixedString boneName("");

			// skin the way the game does via FMD
			if (boneIdx <= 7) {
				if (fmd)
					boneName = fmd->bones[boneIdx];
			}

			if (!*boneName.c_str()) {
				if (origGeom) {
					const auto& rd = origGeom->GetGeometryRuntimeData();
					if (rd.skinInstance && rd.skinInstance->skinData && boneIdx < rd.skinInstance->skinData->bones) {
						auto bone = rd.skinInstance->bones[boneIdx];
						if (isValidNiObject(bone))
							boneName = bone->name;
						else if (bone)
							logger::warn("processGeometry: origGeom '{}' bone[{}] at {:p} is not a valid NiObject (VR NiStream unresolved bone ref)", geometry->name.c_str(), boneIdx, static_cast<void*>(bone));
					}
				} else if (origNiGeom) {
					const auto& spSkin = origNiGeom->GetRuntimeData().spSkinInstance;
					if (spSkin && spSkin->skinData && boneIdx < spSkin->skinData->bones) {
						auto bone = spSkin->bones[boneIdx];
						if (isValidNiObject(bone))
							boneName = bone->name;
						else if (bone)
							logger::warn("processGeometry: origNiGeom bone[{}] at {:p} is not a valid NiObject (VR NiStream unresolved bone ref)", boneIdx, static_cast<void*>(bone));
					}
				}
			}

			auto renameIt = this->head.renameMap.find(boneName.c_str());
			if (renameIt != this->head.renameMap.end()) {
				logger::debug("Found renamed bone {} -> {}.", boneName, renameIt->second->cstr());
				boneName = renameIt->second->cstr();
				hasRenames = true;
			}

			auto boneNode = findNode(this->npc.get(), boneName);

			if (!boneNode && !hasMerged) {
				logger::debug("Bone not found on skeleton, trying skeleton merge.");
				if (this->head.headParts.back().origPartRootNode) {
					doSkeletonMerge(npc.get(), head.headParts.back().origPartRootNode.get(), head.prefix.get(), head.renameMap);
				} else if (this->head.npcFaceGeomNode) {
					// Facegen data doesn't have any tree structure to the skeleton. We need to make any new
					// nodes children of the head node, so that they move properly when there's no physics.
					// This case never happens to a lurker skeleton, thus we don't need to test.
					RE::NiNode* npcHeadNode = findNode(head.npcFaceGeomNode.get(), "NPC Head [Head]");
					if (npcHeadNode) {
						RE::NiTransform invTransform = npcHeadNode->local.Invert();
						auto& children = head.npcFaceGeomNode->GetChildren();

						for (int32_t i = static_cast<int32_t>(children.size()) - 1; i >= 0; --i) {
							auto child = castNiNode(children[i].get());

							if (child && !findNode(npc.get(), child->name)) {
								// hold a reference so DetachChildAt2 doesn't destroy the node
								RE::NiPointer<RE::NiNode> ref(child);
								child->local = invTransform * child->local;
								head.npcFaceGeomNode->DetachChildAt2(i);
								npcHeadNode->AttachChild(child, false);
							}
						}
					}
					doSkeletonMerge(npc.get(), this->head.npcFaceGeomNode.get(), head.prefix.get(), head.renameMap);
				}

				hasMerged = true;

				auto postMergeRenameIt = this->head.renameMap.find(boneName.c_str());

				if (postMergeRenameIt != this->head.renameMap.end()) {
					logger::debug("Found renamed bone {} -> {}.", boneName, postMergeRenameIt->second->cstr());
					boneName = postMergeRenameIt->second->cstr();
					hasRenames = true;
				}

				boneNode = findNode(this->npc.get(), boneName);
			}

			if (!boneNode) {
				logger::error("Bone {} not found after skeleton merge, geometry cannot be fully skinned.", boneName);
				continue;
			}

			geometry->GetGeometryRuntimeData().skinInstance->bones[boneIdx] = boneNode;
			geometry->GetGeometryRuntimeData().skinInstance->boneWorldTransforms[boneIdx] = &boneNode->world;
		}

		geometry->GetGeometryRuntimeData().skinInstance->rootParent = headNode;

		if (hasRenames) {
			for (auto& entry : head.renameMap) {
				if ((this->head.headParts.back().origPartRootNode && findObject(this->head.headParts.back().origPartRootNode.get(), entry.first->cstr())) || (this->head.npcFaceGeomNode && findObject(this->head.npcFaceGeomNode.get(), entry.first->cstr()))) {
					auto findNode = this->head.nodeUseCount.find(entry.first);
					if (findNode != this->head.nodeUseCount.end()) {
						findNode->second += 1;
						logger::debug("Incrementing use count by 1, it is now {}.", findNode->second);
					} else {
						this->head.nodeUseCount.insert(std::make_pair(entry.first, static_cast<uint8_t>(1)));
						logger::debug("First use of bone, count 1.");
					}
					head.headParts.back().renamedBonesInUse.insert(entry.first);
				}
			}
		}

		logger::debug("Done skinning part.");
	}
}
