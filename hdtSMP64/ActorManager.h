#pragma once

#include "skse64/PapyrusActor.h"
#include "../hdtSSEUtils/NetImmerseUtils.h"
#include "../hdtSSEUtils/FrameworkUtils.h"

#include "hdtSkyrimSystem.h"

#include "IEventListener.h"
#include "HookEvents.h"
#include "Offsets.h"

#include <mutex>
#include <optional>

#include "DynamicHDT.h"

namespace hdt
{
	class ActorManager
		: public IEventListener<ArmorAttachEvent>
		, public IEventListener<ArmorDetachEvent>
		, public IEventListener<SkinSingleHeadGeometryEvent>
		, public IEventListener<SkinAllHeadGeometryEvent>
		, public IEventListener<FrameEvent>
		, public IEventListener<ShutdownEvent>
	{
		using IDType = UInt32;

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
			DefaultBBP::PhysicsFile physicsFile;

			void setPhysics(Ref<SkyrimSystem>& system, bool active);
			void clearPhysics();
			bool hasPhysics() const { return m_physics; }
			ActorManager::ItemState state() const;

			const std::vector<Ref<SkinnedMeshBody>>& meshes() const;

			void updateActive(bool active);

			// Update windfactor for all armors attached to skeleton.
			// a_windFactor is a percentage [0,1] with 0 being no wind efect to 1 being full wind effect.
			void setWindFactor(float a_windFactor);

			Ref<SkyrimSystem> m_physics;
		};

		struct Head
		{
			struct HeadPart : public PhysicsItem
			{
				Ref<BSGeometry> headPart;
				Ref<NiNode> origPartRootNode;
				std::set<IDStr> renamedBonesInUse;
			};

			IDType id;
			Ref<IString> prefix;
			Ref<BSFaceGenNiNode> headNode;
			Ref<BSFadeNode> npcFaceGeomNode;
			std::vector<HeadPart> headParts;
			std::unordered_map<IDStr, IDStr> renameMap;
			std::unordered_map<IDStr, uint8_t> nodeUseCount;
			bool isFullSkinning;
			bool isActive = true; // false when hidden by a wig
		};

		struct Armor : public PhysicsItem
		{
			IDType id;
			Ref<IString> prefix;
			Ref<NiAVObject> armorWorn;
			std::unordered_map<IDStr, IDStr> renameMap;
			// @brief This bool is set to true when the first name for the NiAVObject armor is attributed by the Skyrim executable,
			// and set back to false the name map is fixed (see fixArmorNameMaps()),
			bool mustFixNameMap = false;
			// @brief The string is the first name attributed by the Skyrim executable, to be able to detect the change.
			std::string armorCurrentMeshName = "";
		};

		struct Skeleton
		{
			NiPointer<TESObjectREFR> skeletonOwner;
			Ref<NiNode> skeleton;
			Ref<NiNode> npc;
			Head head;
			SkeletonState state;
			bool mustFixOneArmorMap = false;

			std::string name();
			void addArmor(NiNode* armorModel);
			void attachArmor(NiNode* armorModel, NiAVObject* attachedNode);

			void cleanArmor();
			void cleanHead(bool cleanAll = false);
			void clear();

			// @brief This calculates and sets the distance from skeleton to player, and a value that is the cosinus
			// between the camera orientation vector and the camera to skeleton vector, multiplied by the length
			// of the camera to skeleton vector; that value is very fast to compute as it is a dot product, and it
			// can be directly used for our needs later; the distance is provided squared for performance reasons.
			// @param sourcePosition the position of the camera
			// @param sourceOrientation the orientation of the camera
			void calculateDistanceAndOrientationDifferenceFromSource(NiPoint3 sourcePosition, NiPoint3 sourceOrientation);

			bool isPlayerCharacter() const;
			bool isInPlayerView();
			bool hasPhysics = false;
			std::optional<NiPoint3> position() const;

			// @brief Update windfactor for skeleton
			// @param a_windFactor is a percentage [0,1] with 0 being no wind efect to 1 being full wind effect.
			void updateWindFactor(float a_windFactor);
			// @brief Get windfactor for skeleton
			float getWindFactor();

			// @brief Updates the states and activity of skeletons, their heads parts and armors.
			// @param playerCell The skeletons not in the player cell are automatically inactive.
			// @param deactivate If set to true, the concerned skeleton will be inactive, regardless of other elements.
			bool updateAttachedState(const NiNode* playerCell, bool deactivate);

			// bool deactivate(); // FIXME useless?
			void reloadMeshes();

			void scanHead();
			void processGeometry(BSFaceGenNiNode* head, BSGeometry* geometry);

			static void doSkeletonMerge(NiNode* dst, NiNode* src, IString* prefix,
				std::unordered_map<IDStr, IDStr>& map);
			static void doSkeletonClean(NiNode* dst, IString* prefix);
			static NiNode* cloneNodeTree(NiNode* src, IString* prefix, std::unordered_map<IDStr, IDStr>& map);
			static void renameTree(NiNode* root, IString* prefix, std::unordered_map<IDStr, IDStr>& map);

			std::vector<Armor>& getArmors() { return armors; }

			// @brief This is the squared distance between the skeleton and the camera.
			float m_distanceFromCamera2 = std::numeric_limits<float>::max();

			// @brief This is |camera2SkeletonVector|*cos(angle between that vector and the camera direction).
			float m_cosAngleFromCameraDirectionTimesSkeletonDistance = -1.;

		private:
			bool isActiveInScene() const;
			bool checkPhysics();

			bool isActive = false;
			float currentWindFactor = 0.f;
			std::vector<Armor> armors;
		};

		bool m_shutdown = false;
		std::recursive_mutex m_lock;
		std::vector<Skeleton> m_skeletons;

		Skeleton& getSkeletonData(NiNode* skeleton);
		ActorManager::Skeleton* get3rdPersonSkeleton(Actor* actor);
		void ActorManager::setHeadActiveIfNoHairArmor(Actor* actor, Skeleton* skeleton);

	public:
		ActorManager();
		~ActorManager();

		static ActorManager* instance();

		static IDStr armorPrefix(IDType id);
		static IDStr headPrefix(IDType id);

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

		void onEvent(const ArmorAttachEvent& e) override;
		void onEvent(const ArmorDetachEvent& e) override;

		// @brief On this event, we decide which skeletons will be active for physics this frame.
		void onEvent(const FrameEvent& e) override;

		void onEvent(const MenuOpenCloseEvent&);
		void onEvent(const ShutdownEvent&) override;
		void onEvent(const SkinSingleHeadGeometryEvent&) override;
		void onEvent(const SkinAllHeadGeometryEvent&) override;

		bool skeletonNeedsParts(NiNode* skeleton);
		std::vector<Skeleton>& getSkeletons();//Altered by Dynamic HDT

		bool m_skinNPCFaceParts = true;
		bool m_disableSMPHairWhenWigEquipped = false;
		bool m_autoAdjustMaxSkeletons = true; // Whether to dynamically change the maxActive skeletons to maintain min_fps
		int m_maxActiveSkeletons = 20; // The maximum active skeletons; hard limit
		float m_minCullingDistance = 500; // The distance from the camera under which we never cull the skeletons.

		// @brief Depending on this setting, we avoid to calculate the physics of the PC when it is in 1st person view.
		bool m_disable1stPersonViewPhysics = false;

	private:
		NiPoint3 m_cameraPositionDuringFrame;
		static NiNode* getCameraNode();

		void setSkeletonsActive(const bool updateMetrics = false);
	};
}
