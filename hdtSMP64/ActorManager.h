#pragma once

#include "../hdtSSEUtils/NetImmerseUtils.h"
#include "../hdtSSEUtils/FrameworkUtils.h"

#include "hdtSkyrimSystem.h"

#include "IEventListener.h"
#include "HookEvents.h"

#include <mutex>
#include <optional>

#include "DynamicHDT.h"

namespace hdt
{
	class ActorManager
		: public IEventListener<ArmorAttachEvent>
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

		bool m_shutdown = false;
		std::recursive_mutex m_lock;
		std::vector<Skeleton> m_skeletons;

		Skeleton& getSkeletonData(NiNode* skeleton);

		struct PhysicsItem
		{
			DefaultBBP::PhysicsFile physicsFile;

			void setPhysics(Ref<SkyrimSystem>& system, bool active);
			void clearPhysics();
			bool hasPhysics() const { return m_physics; }
			ActorManager::ItemState state() const;

			const std::vector<Ref<SkinnedMeshBody>>& meshes() const;

			void updateActive(bool active);

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
		};

		struct Armor : public PhysicsItem
		{
			IDType id;
			Ref<IString> prefix;
			Ref<NiAVObject> armorWorn;
			std::unordered_map<IDStr, IDStr> renameMap;
		};

		struct Skeleton
		{
			NiPointer<TESObjectREFR> skeletonOwner;
			Ref<NiNode> skeleton;
			Ref<NiNode> npc;
			Head head;
			SkeletonState state;

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

			// FIXME we expose through a public fonction the address to a vector, in a multi-thread environment.
			// If one thread modifies the vector, while another iterates on it, this might lead to a CTD.
			std::vector<Armor>& getArmors() { return armors; }

			// @brief This is the squared distance between the skeleton and the camera.
			float m_distanceFromCamera2 = std::numeric_limits<float>::max();

			// @brief This is |camera2SkeletonVector|*cos(angle between that vector and the camera direction).
			float m_cosAngleFromCameraDirectionTimesSkeletonDistance = -1.;

		private:
			bool isActiveInScene() const;
			bool checkPhysics();

			bool isActive = false;
			std::vector<Armor> armors;
		};

	public:
		ActorManager();
		~ActorManager();

		static ActorManager* instance();

		static IDStr armorPrefix(IDType id);
		static IDStr headPrefix(IDType id);

		// @brief We set a physics file on a specific actor, on a specific worn armor.
		std::pair<bool, std::string> reloadPhysicsFile(UInt32 on_actor_formID, UInt32 on_item_formID, std::string new_physics_file_path);

		// @brief We set a physics file on a specific actor, if the physics file isn't already set.
		bool swapPhysicsFile(UInt32 on_actor_formID, std::string old_physics_file_path, std::string new_physics_file_path);

		// @brief We set the new physics file on the armor, the physics system on the armor, and transfer the current poses to the new system.
		void setPhysicsOnArmor(Armor* armor, Skeleton* skeleton, std::string new_physics_file_path);

		void transferCurrentPosesBetweenSystems(hdt::SkyrimSystem* src, hdt::SkyrimSystem* dst);

		// @brief We loop through the skeletons to find those which owner is on_actor_formID;
		// for those, if on_item_formID is defined, then we look for the worn armor with that formID;
		// then if the old_physics_file_path is set, we check that the armor has this path;
		// else we simply return te worn armor.
		std::pair<ActorManager::Armor*, ActorManager::Skeleton*> findArmor(UInt32 on_actor_formID, UInt32 on_item_formID, std::string old_physics_file_path);

		// @brief We return the current physics file of the found armor.
		std::string queryCurrentPhysicsFile(UInt32 on_actor_formID, UInt32 on_item_formID);

		void SMPDebug_PrintDetailed(bool includeItems);
		void SMPDebug_Execute();

		void onEvent(const ArmorAttachEvent& e) override;

		// @brief On this event, we decide which skeletons will be active for physics this frame.
		void onEvent(const FrameEvent& e) override;

		void onEvent(const MenuOpenCloseEvent&);
		void onEvent(const ShutdownEvent&) override;
		void onEvent(const SkinSingleHeadGeometryEvent&) override;
		void onEvent(const SkinAllHeadGeometryEvent&) override;

#ifdef ANNIVERSARY_EDITION
		bool skeletonNeedsParts(NiNode* skeleton);
#endif

		bool m_skinNPCFaceParts = true;
		bool m_autoAdjustMaxSkeletons = true; // Whether to dynamically change the maxActive skeletons to maintain min_fps
		int m_maxActiveSkeletons = 20; // The maximum active skeletons; hard limit
		int m_sampleSize = 5; // how many samples (each sample taken every second) for determining average time per activeSkeleton.
		float m_maxDistance = 1e4f;
		float m_maxDistance2 = 1e8f; // The maxDistance value needs to be transformed to be useful, this is the useful value.
		float m_maxAngle = 45.0f;
		float m_cosMaxAngle2 = 0.5f; // The maxAngle value needs to be transformed to be useful, this is the useful value.

	private:
		void setSkeletonsActive();
		std::vector<Skeleton>& getSkeletons();
	};

	namespace util {
		inline static std::string _deprefix(std::string str_with_prefix) {
			return str_with_prefix.find("hdtSSEPhysics_AutoRename_") == 0
				? str_with_prefix.substr(str_with_prefix.find(' ') + 1)
				: str_with_prefix;
		}

		inline static bool _match_name(IDStr& a, IDStr& b) {
			if (!a || !b) return false;
			return _deprefix(a->cstr()) == _deprefix(b->cstr());
		}
	}
}
