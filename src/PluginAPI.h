#pragma once

#include "PluginOwnerAPI.h"

/*Plugins should register for messages from hdtSMP64 via SKSE during SKSE's PostLoad event.
* When ready, hdtSMP64 will send a message of type MSG_STARTUP containing a PluginInterface* as data.
*
* Plugin authors need both this header and PluginOwnerAPI.h.
*
* A plugin MUST verify compatibility with the interface version before calling any other functions.
* The interface version is semantic, meaning that
*	- any incompatible API change increments the MAJOR version
*	- any backwards-compatible new feature increments the MINOR version
*	- a PATCH update does not change anything in this API
*
* A plugin MUST verify compatibility with the Bullet version if it intends to interact with any Bullet object.
* Refer to Bullet documentation for information about their versioning scheme.
*
* If compatible, the plugin may call other functions on the PluginInterface at any time.
* The PluginInterface pointer shall remain valid until shutdown.
*/

template <typename T>
class btAlignedObjectArray;
class btCollisionObject;

namespace hdt
{
	//Sent right before the physics simulation begins updating.
	//Only forces and torques may be applied during this event.
	//The collision objects must in every other regard be treated as read-only.
	struct PreStepEvent
	{
		const btAlignedObjectArray<btCollisionObject*>& objects;
		float timeStep{ 0.0f };
	};

	//Sent right after the physics simulation has finished updating.
	//The collision objects must in every regard be treated as read-only.
	struct PostStepEvent
	{
		const btAlignedObjectArray<btCollisionObject*>& objects;
		float timeStep{ 0.0f };
	};

	using IPreStepListener = RE::BSTEventSink<PreStepEvent>;
	using IPostStepListener = RE::BSTEventSink<PostStepEvent>;

	// Sent on the main thread after hdtSMP64's own frame handler, outside its simulation lock.
	struct PluginFrameEvent
	{
		// True when hdtSMP64 suspends its simulation: engine pause, a pausing menu, or loading.
		bool gamePaused{ false };
		// The interval hdtSMP64 simulated this frame; 0 when it did not step.
		float timeStep{ 0.0f };
	};

	// Sent on the main thread after hdtSMP64 has joined its simulation and written its bone transforms, before skinning.
	struct PluginFrameSyncEvent
	{
	};

	// Sent once on the main thread at game quit, before mesh claims are released. Frame events stop after it.
	struct PluginShutdownEvent
	{
	};

	using IFrameListener = RE::BSTEventSink<PluginFrameEvent>;
	using IFrameSyncListener = RE::BSTEventSink<PluginFrameSyncEvent>;
	using IShutdownListener = RE::BSTEventSink<PluginShutdownEvent>;

	class PluginInterface
	{
	public:
		enum MessageType : unsigned long
		{
			MSG_STARTUP,
		};

		struct Version
		{
			int major;
			int minor;
			int patch;
		};

		struct VersionInfo
		{
			Version interfaceVersion;
			Version bulletVersion;
		};

	public:
		constexpr static Version INTERFACE_VERSION{ 2, 2, 0 };
		constexpr static Version BULLET_VERSION{ 3, 24, 0 };

	public:
		virtual ~PluginInterface() = default;

		virtual const VersionInfo& getVersionInfo() const = 0;

		virtual void addListener(IPreStepListener*) = 0;
		virtual void removeListener(IPreStepListener*) = 0;

		virtual void addListener(IPostStepListener*) = 0;
		virtual void removeListener(IPostStepListener*) = 0;

		// @brief Registers a mesh owner. Ignores null and already registered owners.
		virtual void addMeshOwner(IMeshOwner*, const MeshOwnerInfo&) = 0;

		// @brief Unregisters an owner. Call it from the game main thread, not from inside claim or release. It returns
		// after any running claim or release has finished; hdtSMP64 then never calls the owner or its claims again, so the
		// owner frees its remaining per-mesh state itself. The affected actors are rebuilt on the main thread.
		virtual void removeMeshOwner(IMeshOwner*) = 0;

		// @brief Rebuilds an actor's meshes: each releases its claim and asks the owners again. This is how an owner
		// gives a mesh back (decline in claim, then call this). Main thread only, not from inside claim or release; the
		// rebuild is queued, so release runs later.
		virtual void reloadActorPhysics(RE::Actor*) = 0;

		// @brief Registers a listener for PluginFrameEvent. Listeners run in registration order, on the main thread,
		// after hdtSMP64's own handlers. A listener may add or remove listeners; additions take effect from the next
		// event. A listener must stay valid until its removal returns. Adding or removing from another thread blocks
		// until the current event ends. Do not call removeMeshOwner from a listener.
		virtual void addFrameListener(IFrameListener*) = 0;
		virtual void removeFrameListener(IFrameListener*) = 0;

		// @brief Registers a listener for PluginFrameSyncEvent. Same contract as addFrameListener.
		virtual void addFrameSyncListener(IFrameSyncListener*) = 0;
		virtual void removeFrameSyncListener(IFrameSyncListener*) = 0;

		// @brief Registers a listener for PluginShutdownEvent. Same contract as addFrameListener.
		virtual void addShutdownListener(IShutdownListener*) = 0;
		virtual void removeShutdownListener(IShutdownListener*) = 0;
	};
}
