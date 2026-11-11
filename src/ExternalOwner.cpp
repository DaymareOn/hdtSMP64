#include "ExternalOwner.h"

#include <algorithm>

namespace hdt
{
	ExternalClaim::ExternalClaim(ExternalOwnerRegistry* registry, IMeshOwner* owner, std::uint64_t token, std::string ownerName) :
		m_registry(registry),
		m_owner(owner),
		m_token(token),
		m_ownerName(std::move(ownerName))
	{}

	ExternalClaim::ExternalClaim(ExternalClaim&& a_rhs) noexcept :
		m_registry(a_rhs.m_registry),
		m_owner(a_rhs.m_owner),
		m_token(a_rhs.m_token),
		m_ownerName(std::move(a_rhs.m_ownerName))
	{
		a_rhs.m_registry = nullptr;
		a_rhs.m_owner = nullptr;
		a_rhs.m_token = 0;
	}

	ExternalClaim& ExternalClaim::operator=(ExternalClaim&& a_rhs) noexcept
	{
		if (this != &a_rhs) {
			reset();

			m_registry = a_rhs.m_registry;
			m_owner = a_rhs.m_owner;
			m_token = a_rhs.m_token;
			m_ownerName = std::move(a_rhs.m_ownerName);

			a_rhs.m_registry = nullptr;
			a_rhs.m_owner = nullptr;
			a_rhs.m_token = 0;
		}

		return *this;
	}

	ExternalClaim::~ExternalClaim()
	{
		reset();
	}

	void ExternalClaim::reset()
	{
		ExternalOwnerRegistry* const registry = m_registry;
		IMeshOwner* const owner = m_owner;
		const std::uint64_t token = m_token;

		m_registry = nullptr;
		m_owner = nullptr;
		m_token = 0;

		if (owner && registry) {
			registry->release(owner, token);
		}
	}

	ExternalOwnerRegistry& ExternalOwnerRegistry::instance()
	{
		static ExternalOwnerRegistry s_registry;
		return s_registry;
	}

	void ExternalOwnerRegistry::addOwner(IMeshOwner* owner, const MeshOwnerInfo& info)
	{
		if (!owner) {
			return;
		}

		std::lock_guard<std::mutex> lock(m_mutex);
		const auto known = std::find_if(m_owners.begin(), m_owners.end(), [owner](const Entry& e) { return e.owner == owner; });
		if (known != m_owners.end()) {
			return;
		}

		Entry entry{ owner, info.name && *info.name ? info.name : "unnamed", info.priority };
		const auto position = std::find_if(m_owners.begin(), m_owners.end(), [&entry](const Entry& e) { return e.priority < entry.priority; });
		m_owners.insert(position, std::move(entry));
	}

	void ExternalOwnerRegistry::removeOwner(IMeshOwner* owner)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_owners.erase(std::remove_if(m_owners.begin(), m_owners.end(), [owner](const Entry& e) { return e.owner == owner; }), m_owners.end());
		m_live.erase(std::remove_if(m_live.begin(), m_live.end(), [owner](const auto& l) { return l.second.owner == owner; }), m_live.end());
		m_skips.erase(std::remove_if(m_skips.begin(), m_skips.end(), [owner](const Skip& s) { return s.owner == owner; }), m_skips.end());
	}

	bool ExternalOwnerRegistry::isRegistered(IMeshOwner* owner) const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return std::any_of(m_owners.begin(), m_owners.end(), [owner](const Entry& e) { return e.owner == owner; });
	}

	bool ExternalOwnerRegistry::isSkipped(IMeshOwner* owner, std::uint32_t actorFormId, const void* model) const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (std::find(m_reloading.begin(), m_reloading.end(), actorFormId) == m_reloading.end()) {
			return false;
		}

		return std::any_of(m_skips.begin(), m_skips.end(), [&](const Skip& s) {
			return s.owner == owner && s.actorFormId == actorFormId && s.model == model;
		});
	}

	void ExternalOwnerRegistry::beginRelinquishedReload(std::uint32_t actorFormId)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_reloading.push_back(actorFormId);
	}

	void ExternalOwnerRegistry::endRelinquishedReload(std::uint32_t actorFormId)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto reloading = std::find(m_reloading.begin(), m_reloading.end(), actorFormId);
		if (reloading != m_reloading.end()) {
			m_reloading.erase(reloading);
		}

		m_skips.erase(std::remove_if(m_skips.begin(), m_skips.end(), [actorFormId](const Skip& s) { return s.actorFormId == actorFormId; }), m_skips.end());
	}

	ExternalClaim ExternalOwnerRegistry::tryClaim(const IMeshClaimContext& context, std::uint32_t actorFormId)
	{
		std::vector<Entry> owners;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			owners = m_owners;
		}

		const void* model = context.getModel();
		const std::uint64_t token = m_nextToken.fetch_add(1, std::memory_order_relaxed);
		for (const Entry& entry : owners) {
			// An earlier owner may have unregistered this one from inside its claim.
			if (!isRegistered(entry.owner) || isSkipped(entry.owner, actorFormId, model)) {
				continue;
			}

			if (!entry.owner->claim(context, token)) {
				continue;
			}

			{
				std::lock_guard<std::mutex> lock(m_mutex);
				const bool stillRegistered = std::any_of(m_owners.begin(), m_owners.end(), [&](const Entry& e) { return e.owner == entry.owner; });
				if (!stillRegistered) {
					continue;
				}

				m_live.emplace_back(token, Live{ entry.owner, actorFormId, model });
			}

			return ExternalClaim(this, entry.owner, token, entry.name);
		}

		return ExternalClaim();
	}

	void ExternalOwnerRegistry::release(IMeshOwner* owner, std::uint64_t token)
	{
		if (!owner) {
			return;
		}

		bool live = false;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			const auto found = std::find_if(m_live.begin(), m_live.end(), [&](const auto& l) { return l.first == token && l.second.owner == owner; });
			if (found != m_live.end()) {
				m_live.erase(found);
				live = true;
			}
		}

		if (live) {
			owner->release(token);
		}
	}

	std::optional<ExternalOwnerRegistry::Relinquished> ExternalOwnerRegistry::relinquish(IMeshOwner* owner, std::uint64_t token)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const auto found = std::find_if(m_live.begin(), m_live.end(), [&](const auto& l) { return l.first == token && l.second.owner == owner; });
		if (found == m_live.end()) {
			return std::nullopt;
		}

		Live live = std::move(found->second);
		m_live.erase(found);
		const Relinquished result{ live.actorFormId };
		m_skips.push_back(Skip{ live.owner, live.actorFormId, live.model });
		return result;
	}

	std::vector<std::string> ExternalOwnerRegistry::ownerNames() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<std::string> names;
		names.reserve(m_owners.size());
		for (const Entry& entry : m_owners) {
			names.push_back(entry.name);
		}

		return names;
	}
}
