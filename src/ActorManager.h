#pragma once

#include "NetImmerseUtils.h"

#include "DynamicHDT.h"
#include "Events.h"
#include "hdtSkyrimSystem.h"

namespace hdt
{
	class ActorManager :
		public RE::BSTEventSink<Events::ArmorAttachEvent>,
		public RE::BSTEventSink<Events::ArmorDetachEvent>,
		public RE::BSTEventSink<Events::SkinSingleHeadGeometryEvent>,
		public RE::BSTEventSink<Events::SkinAllHeadGeometryEvent>,
		public RE::BSTEventSink<Events::FrameEvent>,
		public RE::BSTEventSink<Events::ShutdownEvent>
	{
		using IDType = uint32_t;

	public:
		enum class ItemState
		{
			e_NoPhysics,
			e_Inactive,
			e_Active
		};

		// Overall skeleton state, purely for console debug info
		enum class SkeletonState
		{
			// Note order: inactive states must come before e_SkeletonActive, and active states after
			e_InactiveNotInScene,
			e_InactiveUnseenByPlayer,
			e_InactiveTooFar,
			e_SkeletonActive,
			e_ActiveNearPlayer,
			e_ActiveIsPlayer
		};
		int activeSkeletons = 0;

	private:
		int maxActiveSkeletons = 10;
		int frameCount = 0;
		float rollingAverage = 0;
		struct Skeleton;

		struct PhysicsItem
		{
			DefaultBBP::PhysicsFile_t physicsFile;

			void setPhysics(RE::BSTSmartPointer<SkyrimSystem>& system, bool active);
			void clearPhysics();
			bool hasPhysics() const { return m_physics.get(); }
			ActorManager::ItemState state() const;

			const std::vector<RE::BSTSmartPointer<SkinnedMeshBody>>& meshes() const;

			void updateActive(bool active);

			// Update windfactor for all armors attached to skeleton.
			// a_windFactor is a percentage [0,1] with 0 being no wind effect to 1 being full wind effect.
			void setWindFactor(float a_windFactor);

			RE::BSTSmartPointer<SkyrimSystem> m_physics;
			bool m_hasDynamicPhysics = false;
		};

		struct Head
		{
			struct HeadPart : public PhysicsItem
			{
				RE::NiPointer<RE::BSGeometry> headPart;
				RE::NiPointer<RE::NiNode> origPartRootNode;
				std::unordered_set<RE::BSFixedString> renamedBonesInUse;
			};

			IDType id;
			std::string prefix;
			RE::NiPointer<RE::BSFaceGenNiNode> headNode;
			RE::NiPointer<RE::BSFadeNode> npcFaceGeomNode;
			bool npcFaceGeomNodeBroken = false;  // true if isolated NiStream load produced broken VR bone refs
			std::vector<HeadPart> headParts;
			std::unordered_map<RE::BSFixedString, RE::BSFixedString> renameMap;
			std::unordered_map<RE::BSFixedString, uint8_t> nodeUseCount;
			bool isFullSkinning;
			bool isActive = true;  // false when hidden by a wig
		};

		struct Armor : public PhysicsItem
		{
			IDType id;
			std::string prefix;
			RE::NiPointer<RE::NiAVObject> armorWorn;
			std::unordered_map<RE::BSFixedString, RE::BSFixedString> renameMap;
			// @brief This bool is set to true when the first name for the NiAVObject armor is attributed by the Skyrim executable,
			// and set back to false the name map is fixed (see fixArmorNameMaps()),
			bool mustFixNameMap = false;
			// @brief The string is the first name attributed by the Skyrim executable, to be able to detect the change.
			std::string armorCurrentMeshName = "";
		};

		struct Skeleton
		{
			RE::NiPointer<RE::TESObjectREFR> skeletonOwner;
			RE::NiPointer<RE::NiNode> skeleton;
			RE::NiPointer<RE::NiNode> npc;
			Head head;
			SkeletonState state;
			bool mustFixOneArmorMap = false;

			std::string name();
			void addArmor(RE::NiNode* armorModel);
			void attachArmor(RE::NiNode* armorModel, RE::NiAVObject* attachedNode);

			void cleanArmor();
			void cleanHead(bool cleanAll = false);
			void clear();

			// @brief This calculates and sets the distance from skeleton to player, and a value that is the cosinus
			// between the camera orientation vector and the camera to skeleton vector, multiplied by the length
			// of the camera to skeleton vector; that value is very fast to compute as it is a dot product, and it
			// can be directly used for our needs later; the distance is provided squared for performance reasons.
			// @param sourcePosition the position of the camera
			// @param sourceOrientation the orientation of the camera
			void calculateDistanceAndOrientationDifferenceFromSource(RE::NiPoint3 sourcePosition, RE::NiPoint3 sourceOrientation);

			bool isPlayerCharacter() const;
			bool isInPlayerView();
			bool hasPhysics = false;
			std::optional<RE::NiPoint3> position() const;

			// @brief Update windfactor for skeleton
			// @param a_windFactor is a percentage [0,1] with 0 being no wind effect to 1 being full wind effect.
			void updateWindFactor(float a_windFactor);
			// @brief Get windfactor for skeleton
			float getWindFactor();

			// @brief Updates the states and activity of skeletons, their heads parts and armors.
			// @param playerCell The skeletons not in the player cell are automatically inactive.
			// @param deactivate If set to true, the concerned skeleton will be inactive, regardless of other elements.
			bool updateAttachedState(const RE::NiNode* playerCell, bool deactivate);

			// bool deactivate(); // FIXME useless?
			void reloadMeshes();

			void scanHead();
			void processGeometry(RE::BSFaceGenNiNode* head, RE::BSGeometry* geometry);

			static void doSkeletonMerge(RE::NiNode* dst, RE::NiNode* src, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map);
			static void doSkeletonClean(RE::NiNode* dst, std::string_view prefix);
			static RE::NiNode* cloneNodeTree(RE::NiNode* src, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map);
			static void renameTree(RE::NiNode* root, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map);

			std::vector<Armor>& getArmors() { return armors; }

			// @brief This is the squared distance between the skeleton and the camera.
			float m_distanceFromCamera2 = std::numeric_limits<float>::max();

			// @brief This is |camera2SkeletonVector|*cos(angle between that vector and the camera direction).
			float m_cosAngleFromCameraDirectionTimesSkeletonDistance = -1.;

		private:
			bool isActiveInScene() const;
			bool checkPhysics();
			static void doSkeletonMerge(RE::NiNode* dst, RE::NiNode* src, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map, RE::NiNode* dstRoot);

			bool isActive = false;
			float currentWindFactor = 0.f;
			std::vector<Armor> armors;
		};

		bool m_shutdown = false;
		std::recursive_mutex m_lock;
		std::vector<Skeleton> m_skeletons;

		Skeleton& getSkeletonData(RE::NiNode* skeleton);
		ActorManager::Skeleton* get3rdPersonSkeleton(RE::Actor* actor);
		static void setHeadActiveIfNoHairArmor(RE::Actor* actor, Skeleton* skeleton);

	public:
		ActorManager();
		~ActorManager();

		static ActorManager* instance();

		static std::string armorPrefix(IDType id);
		static std::string headPrefix(IDType id);

		/*
		fix: take into account the unexpected armors names changes done by the Skyrim executable.

		We add smp physics to armors on the ArmorAttachEvent.
		But when a smp reset happens, we can't go through the ArmorAttachEvent processing: no event is sent by the skyrim executable.
		So for each known skeleton, and each of its known armors meshes, we reapply the related xml file.

		Each Armor has in .physicsFile the applied xml file, the names of the meshes of the armor in the xml file / nif,
		and for each mesh name the name(s) of the NiAVObject attached through the ArmorAttachEvent hook processing.

		So by looking for the recorded NiAVObject names in the related skyrim models, we can find back the NiAVObject and reapply the related xml file to it.

		But! The skyrim executable changes later the name of the NiAVObject passed as attachedNode through the ArmorAttachEvent hook.
		So, when trying to find the recorded name in the existing objects, we don't find it anymore.

		This bug happens for armors, but not for headparts, which names aren't changed by Skyrim on the fly.
		https://github.com/DaymareOn/hdtSMP64/issues/84
		This bug has happened since the original HDT-SMP, for all versions of Skyrim (well, I haven't checked on the VR version).

		The implemented solution is to 1) when attaching an armor, record that the fix will need to be applied on this armor,
		2) save the original name,
		3) to be able to detect on following events when that the name has changed (ArmorAttachEvent, ItemUnequipEvent, FrameEvent, OpenMenuEvent)
		   (checking that the fix needs to be applied is quick, and introducing the fix in all events allows to have it fixed asap),
		4) and then add in.physicsfile the new name;
		5) finally remove the information that a fix must be applied for this armor.
		*/
		void fixArmorNameMaps();

		RE::BSEventNotifyControl ProcessEvent(const Events::ArmorAttachEvent* e, RE::BSTEventSource<Events::ArmorAttachEvent>*) override;
		RE::BSEventNotifyControl ProcessEvent(const Events::ArmorDetachEvent* e, RE::BSTEventSource<Events::ArmorDetachEvent>*) override;

		// @brief On this event, we decide which skeletons will be active for physics this frame.
		RE::BSEventNotifyControl ProcessEvent(const Events::FrameEvent* e, RE::BSTEventSource<Events::FrameEvent>*) override;

		RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent*, RE::BSTEventSource<RE::MenuOpenCloseEvent>*);
		RE::BSEventNotifyControl ProcessEvent(const Events::ShutdownEvent*, RE::BSTEventSource<Events::ShutdownEvent>*) override;
		RE::BSEventNotifyControl ProcessEvent(const Events::SkinSingleHeadGeometryEvent*, RE::BSTEventSource<Events::SkinSingleHeadGeometryEvent>*) override;
		RE::BSEventNotifyControl ProcessEvent(const Events::SkinAllHeadGeometryEvent*, RE::BSTEventSource<Events::SkinAllHeadGeometryEvent>*) override;

		bool skeletonNeedsParts(RE::NiNode* skeleton);
		std::vector<Skeleton>& getSkeletons();  //Altered by Dynamic HDT

		bool m_skinNPCFaceParts = true;
		bool m_disableSMPHairWhenWigEquipped = false;
		bool m_autoAdjustMaxSkeletons = true;  // Whether to dynamically change the maxActive skeletons to maintain min_fps
		int m_maxActiveSkeletons = 20;         // The maximum active skeletons; hard limit
		float m_minCullingDistance = 500;      // The distance from the camera under which we never cull the skeletons.

		// @brief Depending on this setting, we avoid to calculate the physics of the PC when it is in 1st person view.
		bool m_disable1stPersonViewPhysics = false;

	private:
		RE::NiPoint3 m_cameraPositionDuringFrame;
		static RE::NiNode* getCameraNode();

		void setSkeletonsActive(const bool updateMetrics = false);
	};
}
