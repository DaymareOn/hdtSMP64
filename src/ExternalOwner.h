#pragma once

#include "PluginOwnerAPI.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace hdt
{
	class ExternalOwnerRegistry;

	class ExternalClaim
	{
	public:
		~ExternalClaim();

		ExternalClaim(const ExternalClaim&) = delete;
		ExternalClaim& operator=(const ExternalClaim&) = delete;

		const std::string& ownerName() const;
		void release();

	private:
		friend class ExternalOwnerRegistry;
		struct OwnerState;

		ExternalClaim(ExternalOwnerRegistry* registry, std::shared_ptr<OwnerState> owner, IMeshClaim* claim);

		ExternalOwnerRegistry* m_registry;
		std::shared_ptr<OwnerState> m_owner;
		IMeshClaim* m_claim;
		std::atomic<bool> m_released{ false };
	};

	class ExternalOwnerRegistry
	{
	public:
		static ExternalOwnerRegistry& instance();

		void addOwner(IMeshOwner* owner, const MeshOwnerInfo& info = {});

		std::vector<std::uint32_t> removeOwner(IMeshOwner* owner);

		std::shared_ptr<ExternalClaim> tryClaim(const IMeshClaimContext& context, std::uint32_t actorFormId = 0);

		std::vector<std::string> ownerNames() const;

	private:
		friend class ExternalClaim;
		using OwnerState = ExternalClaim::OwnerState;

		void releaseClaim(const std::shared_ptr<OwnerState>& owner, IMeshClaim* claim);

		// Always taken before m_mutex: the reverse order deadlocks removeOwner against a running callback.
		std::mutex m_callbackMutex;
		mutable std::mutex m_mutex;
		std::vector<std::shared_ptr<OwnerState>> m_owners;
	};
}
