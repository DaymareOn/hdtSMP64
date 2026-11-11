#include "ExternalOwner.h"

#include <algorithm>

namespace hdt
{
	struct ExternalClaim::OwnerState
	{
		struct Live
		{
			IMeshClaim* claim = nullptr;
			std::uint32_t actorFormId = 0;
		};

		IMeshOwner* owner = nullptr;
		std::string name;
		std::int32_t priority = 0;
		bool removed = false;
		std::vector<Live> live;
	};

	ExternalClaim::ExternalClaim(ExternalOwnerRegistry* registry, std::shared_ptr<OwnerState> owner, IMeshClaim* claim) :
		m_registry(registry),
		m_owner(std::move(owner)),
		m_claim(claim)
	{}

	ExternalClaim::~ExternalClaim()
	{
		release();
	}

	const std::string& ExternalClaim::ownerName() const
	{
		return m_owner->name;
	}

	void ExternalClaim::release()
	{
		if (!m_released.exchange(true)) {
			m_registry->releaseClaim(m_owner, m_claim);
		}
	}

	ExternalOwnerRegistry& ExternalOwnerRegistry::instance()
	{
		// Leaked on purpose: ExternalClaim destructors run after a function-local static would already be destroyed.
		static ExternalOwnerRegistry* const s_registry = new ExternalOwnerRegistry();
		return *s_registry;
	}

	void ExternalOwnerRegistry::addOwner(IMeshOwner* owner, const MeshOwnerInfo& info)
	{
		if (!owner) {
			return;
		}

		std::lock_guard<std::mutex> lock(m_mutex);
		const auto known = std::find_if(m_owners.begin(), m_owners.end(), [owner](const auto& e) { return e->owner == owner; });
		if (known != m_owners.end()) {
			return;
		}

		auto state = std::make_shared<OwnerState>();
		state->owner = owner;
		state->name = info.name && *info.name ? info.name : "unnamed";
		state->priority = info.priority;
		const auto position = std::find_if(m_owners.begin(), m_owners.end(), [&state](const auto& e) { return e->priority < state->priority; });
		m_owners.insert(position, std::move(state));
	}

	std::vector<std::uint32_t> ExternalOwnerRegistry::removeOwner(IMeshOwner* owner)
	{
		std::lock_guard<std::mutex> callbackLock(m_callbackMutex);
		std::lock_guard<std::mutex> lock(m_mutex);

		std::vector<std::uint32_t> actors;
		const auto found = std::find_if(m_owners.begin(), m_owners.end(), [owner](const auto& e) { return e->owner == owner; });
		if (found == m_owners.end()) {
			return actors;
		}

		const std::shared_ptr<OwnerState> state = *found;
		for (const auto& live : state->live) {
			if (live.actorFormId != 0 && std::find(actors.begin(), actors.end(), live.actorFormId) == actors.end()) {
				actors.push_back(live.actorFormId);
			}
		}

		state->removed = true;
		state->live.clear();
		m_owners.erase(found);
		return actors;
	}

	std::shared_ptr<ExternalClaim> ExternalOwnerRegistry::tryClaim(const IMeshClaimContext& context, std::uint32_t actorFormId)
	{
		std::vector<std::shared_ptr<OwnerState>> owners;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			owners = m_owners;
		}

		for (const auto& state : owners) {
			std::lock_guard<std::mutex> callbackLock(m_callbackMutex);

			if (state->removed) {
				continue;
			}

			IMeshClaim* claim = nullptr;
			try {
				claim = state->owner->claim(context);
			} catch (...) {
			}

			if (!claim) {
				continue;
			}

			{
				std::lock_guard<std::mutex> lock(m_mutex);
				state->live.push_back({ claim, actorFormId });
			}

			return std::shared_ptr<ExternalClaim>(new ExternalClaim(this, state, claim));
		}

		return nullptr;
	}

	void ExternalOwnerRegistry::releaseClaim(const std::shared_ptr<OwnerState>& owner, IMeshClaim* claim)
	{
		std::lock_guard<std::mutex> callbackLock(m_callbackMutex);

		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (owner->removed) {
				return;
			}

			const auto found = std::find_if(owner->live.begin(), owner->live.end(), [claim](const auto& l) { return l.claim == claim; });
			if (found != owner->live.end()) {
				owner->live.erase(found);
			}
		}

		try {
			claim->release();
		} catch (...) {
		}
	}

	std::vector<std::string> ExternalOwnerRegistry::ownerNames() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<std::string> names;
		names.reserve(m_owners.size());
		for (const auto& entry : m_owners) {
			names.push_back(entry->name);
		}

		return names;
	}
}
