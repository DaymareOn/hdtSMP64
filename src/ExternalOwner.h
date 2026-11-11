#pragma once

#include "PluginOwnerAPI.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace hdt
{
	class ExternalOwnerRegistry;

	// @brief Move-only RAII handle for one mesh claim; releases its owner when destroyed, reset or reassigned.
	class ExternalClaim
	{
	public:
		ExternalClaim() = default;
		ExternalClaim(ExternalOwnerRegistry* registry, IMeshOwner* owner, std::uint64_t token, std::string ownerName = {});
		ExternalClaim(ExternalClaim&& a_rhs) noexcept;

		// Releases the claim this object currently holds before taking over the other's.
		ExternalClaim& operator=(ExternalClaim&& a_rhs) noexcept;

		ExternalClaim(const ExternalClaim&) = delete;
		ExternalClaim& operator=(const ExternalClaim&) = delete;

		~ExternalClaim();

		bool active() const { return m_owner != nullptr; }
		std::uint64_t token() const { return m_token; }

		// The name the owner registered with, for logging.
		const std::string& ownerName() const { return m_ownerName; }

		// Releases if active, then becomes inactive.
		void reset();

	private:
		ExternalOwnerRegistry* m_registry = nullptr;
		IMeshOwner* m_owner = nullptr;
		std::uint64_t m_token = 0;
		std::string m_ownerName;
	};

	// @brief Process-wide list of mesh owners, asked in priority order whenever hdtSMP64 is about to create a
	// system for an armor or head part.
	class ExternalOwnerRegistry
	{
	public:
		static ExternalOwnerRegistry& instance();

		// Ignores null and already registered owners. Owners with a higher priority are asked first; equal
		// priorities keep registration order.
		void addOwner(IMeshOwner* owner, const MeshOwnerInfo& info = {});

		// Ends the owner's live claims without release callbacks, and forgets its pending skips.
		void removeOwner(IMeshOwner* owner);

		// The first owner in priority order that returns true from claim wins; inactive when none does. An owner
		// is not asked about a mesh it relinquished while that actor's relinquished reload runs (see
		// beginRelinquishedReload). An owner that unregisters inside claim and then accepts does not win.
		// @param actorFormId Form ID of the actor the mesh belongs to, 0 when there is none.
		ExternalClaim tryClaim(const IMeshClaimContext& context, std::uint32_t actorFormId = 0);

		// Calls owner->release only while the token is a live claim of that owner.
		void release(IMeshOwner* owner, std::uint64_t token);

		struct Relinquished
		{
			std::uint32_t actorFormId = 0;
		};

		// Ends a live claim without a release call and remembers that this owner must not get the mesh back in the
		// reload that follows. Safe from any thread. Empty when the token is not a live claim of the owner.
		std::optional<Relinquished> relinquish(IMeshOwner* owner, std::uint64_t token);

		// Brackets the actor reload that a relinquish queued: only claims made in between honour that actor's skips,
		// so an unrelated reload that happens first cannot use them up. endRelinquishedReload drops them.
		void beginRelinquishedReload(std::uint32_t actorFormId);
		void endRelinquishedReload(std::uint32_t actorFormId);

		// Registered owner names in the order they are asked, for diagnostics and tests.
		std::vector<std::string> ownerNames() const;

	private:
		struct Entry
		{
			IMeshOwner* owner = nullptr;
			std::string name;
			std::int32_t priority = 0;
		};

		struct Live
		{
			IMeshOwner* owner = nullptr;
			std::uint32_t actorFormId = 0;
			const void* model = nullptr;
		};

		// A mesh is told apart by its model node, because one actor can wear two meshes with the same physics file.
		struct Skip
		{
			IMeshOwner* owner = nullptr;
			std::uint32_t actorFormId = 0;
			const void* model = nullptr;
		};

		bool isRegistered(IMeshOwner* owner) const;
		bool isSkipped(IMeshOwner* owner, std::uint32_t actorFormId, const void* model) const;

		// Owner callbacks run outside m_mutex so an owner may register or unregister owners from inside claim.
		mutable std::mutex m_mutex;
		std::vector<Entry> m_owners;
		std::vector<std::pair<std::uint64_t, Live>> m_live;
		std::vector<Skip> m_skips;
		std::vector<std::uint32_t> m_reloading;
		std::atomic<std::uint64_t> m_nextToken{ 1 };
	};
}
