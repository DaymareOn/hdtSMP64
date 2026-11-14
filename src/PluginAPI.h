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
		constexpr static Version INTERFACE_VERSION{ 2, 1, 0 };
		constexpr static Version BULLET_VERSION{ 3, 24, 0 };

	public:
		virtual ~PluginInterface() = default;

		virtual const VersionInfo& getVersionInfo() const = 0;

		virtual void addListener(IPreStepListener*) = 0;
		virtual void removeListener(IPreStepListener*) = 0;

		virtual void addListener(IPostStepListener*) = 0;
		virtual void removeListener(IPostStepListener*) = 0;

		// @brief Registers an owner that is asked to take over simulation of specific meshes, ordered by
		// MeshOwnerInfo::priority and then by registration order. Ignores null and already registered owners.
		virtual void addMeshOwner(IMeshOwner*, const MeshOwnerInfo&) = 0;

		// @brief Unregisters an owner. Call it from the game main thread, never concurrently with an owner callback.
		// Its claims end without release callbacks and hdtSMP64 simulates those meshes itself from the next rebuild.
		virtual void removeMeshOwner(IMeshOwner*) = 0;

		// @brief Hands a claimed mesh back, for example after the owner lost its GPU device. hdtSMP64 rebuilds the
		// claim's actor on the main thread and simulates the mesh itself, and does not ask this owner about that
		// mesh and actor again during that rebuild. Later rebuilds ask it again. Safe to call from any thread;
		// ignored if the token is not a live claim of this owner. The owner gets no release call for the token.
		virtual void relinquishMesh(IMeshOwner*, std::uint64_t claimToken) = 0;

		// @brief Re-runs the soft reload for an actor, so every one of its meshes asks the mesh owners again.
		virtual void reloadActorPhysics(RE::Actor*) = 0;
	};
}
