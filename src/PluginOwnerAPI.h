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
	// @brief Which kind of mesh an IMeshClaimContext describes.
	enum class MeshKind : std::uint8_t
	{
		Armor,
		HeadPart
	};

	// @brief Read-only description of one mesh hdtSMP64 is about to simulate, handed to IMeshOwner::claim.
	// @details The context and every pointer obtained from it are valid only for the duration of the claim call.
	// Copy any string, and take your own reference to any node, that you need to keep.
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

	// @brief Identity and ordering of one mesh owner, given to PluginInterface::addMeshOwner.
	// @details hdtSMP64 copies the name, so it only has to stay valid for the call.
	struct MeshOwnerInfo
	{
		// @brief Stable identifier shown in hdtSMP64's log. A null or empty name is logged as "unnamed".
		const char* name = nullptr;

		// @brief Owners with a higher priority are asked first. Owners with equal priority are asked in
		// registration order, which follows SKSE plugin load order.
		std::int32_t priority = 0;
	};

	// @brief Implemented by a plugin that takes over simulation of specific hdtSMP64 meshes.
	// @details Several owners may be registered. For each mesh hdtSMP64 is about to simulate it asks them in
	// priority order and the first one whose claim returns true wins; if none does, hdtSMP64 simulates the mesh
	// itself. A claim is sticky: it holds until release, an actor reload, or PluginInterface::relinquishMesh, so
	// a later or higher-priority owner never takes a mesh away mid-session. It is asked again the next time the
	// mesh is created.
	// claim and release run on the game main thread or the Papyrus VM thread, serialized by hdtSMP64's
	// actor manager lock. Implementations must not throw and must not call back into hdtSMP64 except to register
	// or unregister owners, or to relinquish a mesh.
	class IMeshOwner
	{
	public:
		virtual ~IMeshOwner() = default;

		// @brief Returning true takes ownership of the described mesh; hdtSMP64 then creates no system for it.
		// @param claimToken Identifies this claim and is passed to the matching release.
		virtual bool claim(const IMeshClaimContext& context, std::uint64_t claimToken) = 0;

		// @brief hdtSMP64 is done with the claim (the mesh was rebuilt, its actor unloaded, or physics reset).
		// Not called for a claim the owner gave back with relinquishMesh, nor after the owner was removed.
		virtual void release(std::uint64_t claimToken) = 0;
	};
}
