#pragma once

#include "NetImmerseUtils.h"

#include "DynamicHDT.h"
#include "Events.h"
#include "hdtSkyrimSystem.h"

#include <chrono>
#include <mutex>
#include <vector>

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
			// Which of the 6 axis probe rays manageWorldCollisions casts this frame; advances by one each
			// frame so each actor casts only one world-collision ray per frame (the 6 axes cycle over 6 frames).
			uint8_t m_worldRayCursor = 0;

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

			// @brief Experimental: casts a ray along each of the 6 axes from this skeleton's position,
			// and for every nearby static world object hit, registers it as an SMP obstruction (via
			// ActorManager::World) so this actor's dynamic bones can collide with the world. Expired
			// obstructions are pruned each call. Gated behind ActorManager::m_enableWorldCollision.
			void manageWorldCollisions();

			// @brief Updates the states and activity of skeletons, their heads parts and armors.
			// @param playerCell The skeletons not in the player cell are automatically inactive.
			// @param deactivate If set to true, the concerned skeleton will be inactive, regardless of other elements.
			bool updateAttachedState(const RE::NiNode* playerCell, bool deactivate);

			// bool deactivate(); // FIXME useless?
			void reloadMeshes();
			void softReloadMeshes();

			void scanHead();
			void processGeometry(RE::BSFaceGenNiNode* head, RE::BSGeometry* geometry);

			static void doSkeletonMerge(RE::NiNode* dst, RE::NiNode* src, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map, bool renameSource = true);
			static void doSkeletonClean(RE::NiNode* dst, std::string_view prefix);
			static RE::NiNode* cloneNodeTree(RE::NiNode* src, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map, bool renameSource);
			static void renameTree(RE::NiNode* root, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map);

			std::vector<Armor>& getArmors() { return armors; }

			// @brief This is the squared distance between the skeleton and the camera.
			float m_distanceFromCamera2 = std::numeric_limits<float>::max();

			// @brief This is |camera2SkeletonVector|*cos(angle between that vector and the camera direction).
			float m_cosAngleFromCameraDirectionTimesSkeletonDistance = -1.;

		private:
			bool isActiveInScene() const;
			bool checkPhysics();
			static void doSkeletonMerge(RE::NiNode* dst, RE::NiNode* src, std::string_view prefix, std::unordered_map<RE::BSFixedString, RE::BSFixedString>& map, RE::NiNode* dstRoot, bool renameSource);

			bool isActive = false;
			float currentWindFactor = 0.f;
			std::vector<Armor> armors;
		};

		// @brief Experimental world-collision obstruction: one nearby static world object we currently
		// collide with. We build a kinematic physics system straight from the object's LIVE geometry
		// (no cloning) and hold the object via NiPointer so it stays alive while we reference it. It is
		// removed once its timeout counts down to zero.
		struct Obstruction : public PhysicsItem
		{
			RE::NiPointer<RE::NiAVObject> object;  // the live world object we collide with (kept alive by this ref)
			int timeout = 0;
			RE::NiPoint3 builtAt;                  // owner position the current cropped collider was built around
			ObstructionCache cache;                // raw geometry per trishape, so re-cropping needs no GPU re-read
			// The obstruction is cropped around ONE actor (the owner); only the owner's movement re-crops it, so
			// several actors near the same big mesh can't fight to re-crop it around themselves every frame.
			RE::NiPointer<RE::TESObjectREFR> owner;  // actor the crop is centered on (kept alive while it owns this)
			int ownerTimeout = 0;                    // frames until ownership may pass to another actor (owner refreshes it)
			std::vector<RE::NiPoint3> colliderTris;  // kept triangles in world space (3 points each) for the debug wireframe
		};

		// @brief Tracks the nearby static world objects we currently turn into SMP colliders. Experimental
		// and gated behind ActorManager::m_enableWorldCollision. It deliberately does NOT clone geometry:
		// building a collider from a live object's trishapes avoids the CreateClone/ProcessClone crash on
		// complex world objects (issue #394).
		class World
		{
			std::vector<Obstruction> m_obstructions;

		public:
			// @brief Number of obstruction (re)builds performed since ActorManager last reset it. ActorManager
			// zeroes this before each frame's world-collision work and reads it after, to show a per-frame
			// rebuild count in the overlay -- the key signal for whether the add/remove cost is many cheap
			// rebuilds (rate-limit fixes it) or one catastrophic build (attack the build itself).
			int m_buildsThisFrame = 0;

			static World* instance();

			// @brief Builds (or refreshes) a kinematic collider from object's live geometry, cropped to a
			// sphere around clipCenter (actor's position). `actor` is who probed it: an obstruction is owned
			// by one actor and only re-crops when that owner moves (see addObstruction/buildObstruction), so
			// several nearby actors can't thrash the shared collider.
			void addObstruction(RE::NiAVObject* object, RE::TESObjectREFR* actor, RE::NiPoint3 clipCenter);
			// @brief Ages every obstruction each frame; unregisters and drops those that expire.
			void prune();
			// @brief Immediately unregisters and drops ALL obstructions (used when the feature is turned
			// off, so its physics cost is released at once rather than lingering until each times out).
			void clear();
			// @brief Number of world objects currently colliding (overlay stat).
			size_t count() const { return m_obstructions.size(); }
			// @brief Total collider vertices across all obstructions -- a complexity proxy for the overlay.
			size_t totalVertices() const;
			// @brief Append every obstruction's captured collider triangles (world space, 3 points each) to
			// out, up to maxTris triangles total, for the debug wireframe. Returns how many were appended.
			size_t collectColliderTris(std::vector<RE::NiPoint3>& out, size_t maxTris) const;

		private:
			// @brief (Re)builds an obstruction's cropped collider around clipCenter and registers it, reusing
			// the obstruction's cached raw geometry so no GPU re-read is needed.
			void buildObstruction(Obstruction& obstruction, RE::NiPoint3 clipCenter, float radius);
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
		std::unique_lock<std::recursive_mutex> lockGuard() { return std::unique_lock(m_lock); }

		bool m_disableSMPHairWhenWigEquipped = false;
		bool m_autoAdjustMaxSkeletons = true;  // Whether to dynamically change the maxActive skeletons to maintain min_fps
		int m_maxActiveSkeletons = 20;         // The maximum active skeletons; hard limit
		float m_minCullingDistance = 500;      // The distance from the camera under which we never cull the skeletons.

		// @brief Depending on this setting, we avoid to calculate the physics of the PC when it is in 1st person view.
		bool m_disable1stPersonViewPhysics = false;

		// @brief When true, physics is skipped for dead non-player actors, to save performance.
		bool m_skipDeadActors = false;

		// @brief Experimental: when true, dynamic bones collide with nearby static world geometry
		// (see Skeleton::manageWorldCollisions and ActorManager::World). Off by default; toggled via the
		// "World collision" menu option / the <worldCollision> bool in the <smp> config section. The
		// raycast-clone-and-simulate mechanism is expensive and still a prototype.
		bool m_enableWorldCollision = false;

		// @brief When true, only the player character probes and collides with world geometry; every other
		// NPC is skipped. Much cheaper (one set of probes and colliders instead of one per active NPC). Off by
		// default. Config <worldCollisionPlayerOnly>.
		bool m_worldCollisionPlayerOnly = false;

		// @brief How near (Skyrim units) static world geometry must be to an actor to be turned into a
		// collider by manageWorldCollisions. Also the reach of the probe rays. Larger = more coverage
		// but more geometry dragged into the sim (more cost). Config <worldCollisionDistance>.
		float m_worldCollisionDistance = 158.f;

		// @brief How many times per second the collider re-crops to follow an actor walking at a normal
		// speed. The re-crop is really gated on distance moved, so a standing actor never re-crops and a
		// runner re-crops proportionally more; this value just sets the walk-speed reference (converted to a
		// move distance via a nominal walk speed). 0 = build once and never follow. Config
		// <worldCollisionRecropsPerSec>.
		float m_worldCollisionRecropsPerSec = 2.f;

		// @brief Debug: draw the world-collision probe rays on screen (green = hit became a collider, red =
		// miss/too far). Needs the overlay shown. Config <worldCollisionVisualizeRaycasts>.
		bool m_visualizeWorldRaycasts = false;

		// @brief One probe ray captured for on-screen debugging.
		struct WorldRayViz
		{
			RE::NiPoint3 origin;   // where the ray started (the actor)
			RE::NiPoint3 end;      // where it ended: the hit point, or the full reach if it hit nothing
			bool hit = false;      // true when it hit geometry near enough to become a collider
		};
		// Published set of rays the overlay draws, the pending set filled during the frame, and the camera to
		// project them with. m_rayVizLock guards the published set + camera across the main and render threads.
		std::vector<WorldRayViz> m_rayViz;
		std::vector<WorldRayViz> m_rayVizPending;
		std::mutex m_rayVizLock;
		RE::NiPointer<RE::NiCamera> m_debugCamera;  // held so the render thread can project rays safely
		// Published collider triangles (world space, 3 points each) for the wireframe overlay; guarded by
		// m_rayVizLock alongside the rays and camera. Filled only while the raycast visualization is on.
		std::vector<RE::NiPoint3> m_colliderTris;

		// @brief Per-frame CPU cost (ms) of ADDING/REMOVING world colliders (raycast + build + register +
		// prune/clear) -- the on-thread work that causes micro-freezes. m_avg is EMA-smoothed over
		// SkyrimPhysicsWorld::m_sampleSize frames; m_peak is a slowly-decaying max so a one-frame build
		// spike stays visible in the overlay. This is NOT the physics-simulation collision cost.
		float m_avgWorldCollisionMs = 0.f;
		float m_peakWorldCollisionMs = 0.f;
		// @brief Snapshot of how many world objects we currently collide with and the total collider
		// vertices across them -- to see if the physics cost is driven by too many objects or too-complex
		// ones. Shown in the overlay.
		int m_obstructionCount = 0;
		int m_obstructionVertices = 0;
		// @brief Total obstruction (re)crops per second, averaged over a 1-second window (overlay stat). A
		// high value with a high peak means the cost is re-crop frequency; near-zero with a high peak means a
		// single build is expensive (then the next lever is the build itself, not the rate limit).
		float m_recropsPerSec = 0.f;
		// @brief How many probe rays were cast last frame (6 per probing actor). Overlay stat, shows how much
		// probing is going on -- e.g. it drops to 6 when "player only" is on.
		int m_raycastCount = 0;

		// @brief Min percent of screen height a non-player skeleton must occupy to stay active; 0 = disabled. [0,100]
		float m_minScreenSizePercent = 0.f;

	private:
		RE::NiPoint3 m_cameraPositionDuringFrame;
		float m_screenSizeThresholdScale = 0.f;  // precomputed per frame: (minScreenSizePercent/100)^2 * tan(fov/2)^2
		static RE::NiNode* getCameraNode();

		// Running accumulators that turn the per-frame re-crop count into m_recropsPerSec: sum re-crops and
		// wall-clock over a ~1-second window, then divide and reset. m_lastFrameStamp measures each frame's dt.
		int m_recropAccum = 0;
		float m_recropWindow = 0.f;
		std::chrono::steady_clock::time_point m_lastFrameStamp{};
		bool m_haveFrameStamp = false;
		int m_raycastAccum = 0;  // probe rays cast so far this frame; published into m_raycastCount at frame end

		void setSkeletonsActive(const bool updateMetrics = false);
	};
}
