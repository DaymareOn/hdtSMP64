#pragma once

#include <cstdint>

namespace RE
{
	class Actor;
	class NiNode;
	class NiAVObject;
}

namespace hdt
{
	enum class MeshKind : std::uint8_t
	{
		Armor,
		HeadPart
	};

	// @brief The mesh hdtSMP64 is about to simulate, passed to IMeshOwner::claim. The context and every pointer
	// obtained from it are valid only during that call: copy strings and take your own reference to nodes.
	class IMeshClaimContext
	{
	public:
		virtual ~IMeshClaimContext() = default;

		virtual MeshKind getKind() const = 0;

		// @brief SMP XML path of the mesh.
		virtual const char* getPhysicsFile() const = 0;

		// @brief May be null.
		virtual RE::Actor* getActor() const = 0;

		// @brief The NPC node the bones were merged into.
		virtual RE::NiNode* getSkeleton() const = 0;

		virtual RE::NiAVObject* getModel() const = 0;

		// @brief Nullptr if the bone was not renamed.
		virtual const char* getMergedBoneName(const char* originalName) const = 0;
	};

	// @brief Identity and ordering of a mesh owner. hdtSMP64 copies the name.
	struct MeshOwnerInfo
	{
		// @brief Shown in hdtSMP64's log; null or empty is logged as "unnamed".
		const char* name = nullptr;

		// @brief Higher priority is asked first; ties are asked in registration order.
		std::int32_t priority = 0;
	};

	// @brief One mesh an owner has taken over, allocated by the owner in IMeshOwner::claim.
	class IMeshClaim
	{
	public:
		// @brief hdtSMP64 is done with the mesh. Called exactly once and never again for this claim, so the owner frees
		// its per-mesh state here. Not called after PluginInterface::removeMeshOwner. Must not throw.
		virtual void release() = 0;

	protected:
		~IMeshClaim() = default;
	};

	// @brief Implemented by a plugin that takes over simulation of specific meshes.
	// @details For each mesh hdtSMP64 is about to simulate it asks the owners in priority order; the first claim that
	// returns a claim object wins, and if none does hdtSMP64 simulates the mesh itself. A claim holds until release.
	//
	// To give a mesh back, return null from claim from then on and call PluginInterface::reloadActorPhysics for the
	// actor: the rebuild releases the old claim and asks again.
	//
	// claim and release run on the game main thread or the Papyrus VM thread, one at a time, never from inside a plugin
	// callback. They must not throw (a throw from claim counts as a decline) and must not call any hdtSMP64 function.
	// release must not take a lock that one of the owner's frame or step callbacks holds while it calls into hdtSMP64.
	//
	// Claimed meshes skip hdtSMP64's activation rules (distance, off-screen, skeleton budget, dead actors): the owner
	// decides whether to simulate a claimed mesh in a given frame.
	class IMeshOwner
	{
	public:
		virtual ~IMeshOwner() = default;

		// @brief Returns a claim to take the mesh, so hdtSMP64 creates no system for it, or null to decline.
		virtual IMeshClaim* claim(const IMeshClaimContext& context) = 0;
	};
}
