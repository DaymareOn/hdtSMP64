#include <doctest/doctest.h>

#include "ExternalOwner.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace
{
	class FakeContext : public hdt::IMeshClaimContext
	{
	public:
		FakeContext() = default;
		explicit FakeContext(const int* a_model) :
			model(a_model)
		{}

		hdt::MeshKind getKind() const override { return hdt::MeshKind::Armor; }
		const char* getPhysicsFile() const override { return "test.xml"; }
		RE::Actor* getActor() const override { return nullptr; }
		RE::NiNode* getSkeleton() const override { return nullptr; }
		RE::NiAVObject* getModel() const override { return reinterpret_cast<RE::NiAVObject*>(const_cast<int*>(model)); }
		const char* getMergedBoneName(const char*) const override { return nullptr; }

		const int* model = nullptr;
	};

	class FakeOwner : public hdt::IMeshOwner
	{
	public:
		explicit FakeOwner(bool a_claims) :
			claims(a_claims)
		{}

		bool claim(const hdt::IMeshClaimContext&, std::uint64_t a_token) override
		{
			++claimCalls;
			lastClaimToken = a_token;

			if (claims && onClaim) {
				onClaim();
			}

			return claims;
		}

		void release(std::uint64_t a_token) override
		{
			++releaseCalls;
			lastReleaseToken = a_token;
		}

		bool claims = false;
		int claimCalls = 0;
		int releaseCalls = 0;
		std::uint64_t lastClaimToken = 0;
		std::uint64_t lastReleaseToken = 0;
		std::function<void()> onClaim;
	};

	// Unregisters on scope exit so a failing CHECK cannot leak a registration into the next case.
	class OwnerScope
	{
	public:
		~OwnerScope()
		{
			auto& registry = hdt::ExternalOwnerRegistry::instance();
			for (const auto& owner : m_owners) {
				registry.removeOwner(owner.get());
			}
		}

		FakeOwner& create(bool a_claims)
		{
			m_owners.push_back(std::make_unique<FakeOwner>(a_claims));
			return *m_owners.back();
		}

		FakeOwner& add(bool a_claims, hdt::MeshOwnerInfo a_info = {})
		{
			FakeOwner& owner = create(a_claims);
			hdt::ExternalOwnerRegistry::instance().addOwner(&owner, a_info);
			return owner;
		}

	private:
		std::vector<std::unique_ptr<FakeOwner>> m_owners;
	};
}

TEST_CASE("no owners registered gives an inactive claim")
{
	OwnerScope scope;

	const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
	CHECK_FALSE(claim.active());
	CHECK(claim.token() == 0);
}

TEST_CASE("add ignores null and duplicate owners")
{
	OwnerScope scope;
	auto& registry = hdt::ExternalOwnerRegistry::instance();

	registry.addOwner(nullptr);
	CHECK_FALSE(registry.tryClaim(FakeContext()).active());

	FakeOwner& owner = scope.add(true);
	registry.addOwner(&owner);
	registry.addOwner(&owner);

	const hdt::ExternalClaim claim = registry.tryClaim(FakeContext());
	CHECK(claim.active());
	CHECK(owner.claimCalls == 1);
}

TEST_CASE("the first owner in registration order that claims wins")
{
	OwnerScope scope;
	FakeOwner& first = scope.add(true);
	FakeOwner& second = scope.add(true);

	const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
	CHECK(claim.active());
	CHECK(first.claimCalls == 1);
	CHECK(second.claimCalls == 0);
}

TEST_CASE("an owner that declines is skipped and the next one is asked")
{
	OwnerScope scope;
	FakeOwner& declining = scope.add(false);
	FakeOwner& accepting = scope.add(true);

	const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
	CHECK(claim.active());
	CHECK(declining.claimCalls == 1);
	CHECK(accepting.claimCalls == 1);
}

TEST_CASE("a claim releases its owner exactly once")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	SUBCASE("on destruction")
	{
		{
			const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
			CHECK(claim.active());
			CHECK(owner.releaseCalls == 0);
		}

		CHECK(owner.releaseCalls == 1);
	}

	SUBCASE("on reset")
	{
		{
			hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
			REQUIRE(claim.active());

			claim.reset();
			CHECK_FALSE(claim.active());
			CHECK(owner.releaseCalls == 1);

			claim.reset();
			CHECK(owner.releaseCalls == 1);
		}

		CHECK(owner.releaseCalls == 1);
	}

	SUBCASE("on move-assignment over an active claim")
	{
		{
			hdt::ExternalClaim first = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
			hdt::ExternalClaim second = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
			REQUIRE(first.active());
			REQUIRE(second.active());

			first = std::move(second);
			CHECK(owner.releaseCalls == 1);
			CHECK(first.active());
			CHECK_FALSE(second.active());
		}

		CHECK(owner.releaseCalls == 2);
	}
}

TEST_CASE("a moved-from claim does not release")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	{
		hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
		REQUIRE(claim.active());

		hdt::ExternalClaim moved = std::move(claim);
		CHECK_FALSE(claim.active());
		CHECK(claim.token() == 0);
		CHECK(moved.active());
	}

	CHECK(owner.releaseCalls == 1);
}

TEST_CASE("release is not delivered to an owner that was removed after claiming")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	{
		hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
		REQUIRE(claim.active());

		hdt::ExternalOwnerRegistry::instance().removeOwner(&owner);
	}

	CHECK(owner.releaseCalls == 0);
}

TEST_CASE("an owner may register an owner from inside claim")
{
	OwnerScope scope;
	FakeOwner& added = scope.create(true);
	FakeOwner& registering = scope.add(true);
	registering.onClaim = [&] { hdt::ExternalOwnerRegistry::instance().addOwner(&added); };

	auto& registry = hdt::ExternalOwnerRegistry::instance();

	{
		const hdt::ExternalClaim claim = registry.tryClaim(FakeContext());
		CHECK(claim.active());
		CHECK(registering.claimCalls == 1);
		CHECK(added.claimCalls == 0);
	}

	registry.removeOwner(&registering);

	{
		const hdt::ExternalClaim claim = registry.tryClaim(FakeContext());
		CHECK(claim.active());
		CHECK(added.claimCalls == 1);
	}
}

TEST_CASE("an owner may unregister an owner from inside claim")
{
	OwnerScope scope;
	FakeOwner& removing = scope.add(true);
	FakeOwner& victim = scope.add(true);
	removing.onClaim = [&] { hdt::ExternalOwnerRegistry::instance().removeOwner(&victim); };

	auto& registry = hdt::ExternalOwnerRegistry::instance();

	{
		const hdt::ExternalClaim claim = registry.tryClaim(FakeContext());
		CHECK(claim.active());
		CHECK(removing.claimCalls == 1);
		CHECK(victim.claimCalls == 0);
	}

	{
		const hdt::ExternalClaim claim = registry.tryClaim(FakeContext());
		CHECK(claim.active());
		CHECK(victim.claimCalls == 0);
	}
}

TEST_CASE("an owner removed by an earlier owner that declines is not asked")
{
	OwnerScope scope;
	FakeOwner& victim = scope.create(true);
	auto& registry = hdt::ExternalOwnerRegistry::instance();

	struct Remover : hdt::IMeshOwner
	{
		bool claim(const hdt::IMeshClaimContext&, std::uint64_t) override
		{
			hdt::ExternalOwnerRegistry::instance().removeOwner(victim);
			return false;
		}

		void release(std::uint64_t) override {}
		hdt::IMeshOwner* victim = nullptr;
	} remover;
	remover.victim = &victim;
	registry.addOwner(&remover);
	registry.addOwner(&victim);

	CHECK_FALSE(registry.tryClaim(FakeContext()).active());
	CHECK(victim.claimCalls == 0);
	registry.removeOwner(&remover);
}

TEST_CASE("an owner that unregisters itself inside claim and accepts does not win")
{
	OwnerScope scope;
	FakeOwner& leaving = scope.add(true);
	FakeOwner& next = scope.add(true);
	leaving.onClaim = [&] { hdt::ExternalOwnerRegistry::instance().removeOwner(&leaving); };

	auto& registry = hdt::ExternalOwnerRegistry::instance();
	const hdt::ExternalClaim claim = registry.tryClaim(FakeContext());
	CHECK(claim.active());
	CHECK(next.claimCalls == 1);
	CHECK(leaving.releaseCalls == 0);
}

TEST_CASE("tokens are unique and nonzero across claims")
{
	OwnerScope scope;
	scope.add(true);

	std::vector<std::uint64_t> tokens;
	for (int i = 0; i < 8; ++i) {
		const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
		REQUIRE(claim.active());
		CHECK(claim.token() != 0);
		tokens.push_back(claim.token());
	}

	std::sort(tokens.begin(), tokens.end());
	CHECK(std::adjacent_find(tokens.begin(), tokens.end()) == tokens.end());
}

TEST_CASE("the token passed to claim equals the token passed to the matching release")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	{
		const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
		REQUIRE(claim.active());
		CHECK(owner.lastClaimToken == claim.token());
	}

	CHECK(owner.releaseCalls == 1);
	CHECK(owner.lastReleaseToken == owner.lastClaimToken);
}

TEST_CASE("a higher priority owner is asked before earlier registered ones")
{
	OwnerScope scope;
	FakeOwner& low = scope.add(true, { "low", 0 });
	FakeOwner& high = scope.add(true, { "high", 10 });

	const hdt::ExternalClaim claim = hdt::ExternalOwnerRegistry::instance().tryClaim(FakeContext());
	CHECK(claim.active());
	CHECK(claim.ownerName() == "high");
	CHECK(high.claimCalls == 1);
	CHECK(low.claimCalls == 0);
}

TEST_CASE("equal priorities keep registration order and an empty name becomes unnamed")
{
	OwnerScope scope;
	scope.add(false, { "a", 5 });
	scope.add(false, { "b", 5 });
	scope.add(false, { nullptr, 5 });

	const auto names = hdt::ExternalOwnerRegistry::instance().ownerNames();
	REQUIRE(names.size() == 3);
	CHECK(names[0] == "a");
	CHECK(names[1] == "b");
	CHECK(names[2] == "unnamed");
}

TEST_CASE("relinquish ends the claim without a release call and skips the owner during the queued reload")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true, { "gpu", 0 });
	auto& registry = hdt::ExternalOwnerRegistry::instance();

	{
		hdt::ExternalClaim claim = registry.tryClaim(FakeContext(), 0x14);
		REQUIRE(claim.active());

		const auto relinquished = registry.relinquish(&owner, claim.token());
		REQUIRE(relinquished.has_value());
		CHECK(relinquished->actorFormId == 0x14);
		CHECK_FALSE(registry.relinquish(&owner, claim.token()).has_value());
	}

	CHECK(owner.releaseCalls == 0);

	registry.beginRelinquishedReload(0x14);
	const int before = owner.claimCalls;
	CHECK_FALSE(registry.tryClaim(FakeContext(), 0x14).active());
	CHECK(owner.claimCalls == before);
	registry.endRelinquishedReload(0x14);

	CHECK(registry.tryClaim(FakeContext(), 0x14).active());
}

TEST_CASE("an unrelated reload before the queued one does not use up the skip")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true, { "gpu", 0 });
	auto& registry = hdt::ExternalOwnerRegistry::instance();

	hdt::ExternalClaim first = registry.tryClaim(FakeContext(), 0x14);
	REQUIRE(first.active());
	REQUIRE(registry.relinquish(&owner, first.token()).has_value());

	const hdt::ExternalClaim early = registry.tryClaim(FakeContext(), 0x14);
	CHECK(early.active());

	registry.beginRelinquishedReload(0x14);
	CHECK_FALSE(registry.tryClaim(FakeContext(), 0x14).active());
	registry.endRelinquishedReload(0x14);
}

TEST_CASE("the skip covers only the relinquished mesh when an actor wears two with one physics file")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true, { "gpu", 0 });
	auto& registry = hdt::ExternalOwnerRegistry::instance();
	const int bodyModel = 0;
	const int cloakModel = 0;

	hdt::ExternalClaim body = registry.tryClaim(FakeContext(&bodyModel), 0x14);
	hdt::ExternalClaim cloak = registry.tryClaim(FakeContext(&cloakModel), 0x14);
	REQUIRE(body.active());
	REQUIRE(cloak.active());
	REQUIRE(registry.relinquish(&owner, cloak.token()).has_value());

	registry.beginRelinquishedReload(0x14);
	CHECK(registry.tryClaim(FakeContext(&bodyModel), 0x14).active());
	CHECK_FALSE(registry.tryClaim(FakeContext(&cloakModel), 0x14).active());
	registry.endRelinquishedReload(0x14);
}

TEST_CASE("the skip after a relinquish covers only that actor and the lower priority owner still claims")
{
	OwnerScope scope;
	FakeOwner& high = scope.add(true, { "high", 10 });
	FakeOwner& low = scope.add(true, { "low", 0 });
	auto& registry = hdt::ExternalOwnerRegistry::instance();

	hdt::ExternalClaim first = registry.tryClaim(FakeContext(), 0x14);
	REQUIRE(first.active());
	REQUIRE(registry.relinquish(&high, first.token()).has_value());

	registry.beginRelinquishedReload(0x14);
	const hdt::ExternalClaim next = registry.tryClaim(FakeContext(), 0x14);
	CHECK(next.ownerName() == "low");
	registry.endRelinquishedReload(0x14);
	const hdt::ExternalClaim other = registry.tryClaim(FakeContext(), 0x15);
	CHECK(other.ownerName() == "high");
	CHECK(low.claimCalls == 1);
}

TEST_CASE("relinquish is refused for the wrong owner or an unknown token")
{
	OwnerScope scope;
	FakeOwner& a = scope.add(true);
	FakeOwner& b = scope.add(false);
	auto& registry = hdt::ExternalOwnerRegistry::instance();

	const hdt::ExternalClaim claim = registry.tryClaim(FakeContext(), 0x14);
	REQUIRE(claim.active());
	CHECK_FALSE(registry.relinquish(&b, claim.token()).has_value());
	CHECK_FALSE(registry.relinquish(&a, claim.token() + 1000).has_value());
}

TEST_CASE("removing an owner ends its live claims and pending skips")
{
	OwnerScope scope;
	FakeOwner& owner = scope.create(true);
	auto& registry = hdt::ExternalOwnerRegistry::instance();
	registry.addOwner(&owner, { "gpu", 0 });

	{
		hdt::ExternalClaim claim = registry.tryClaim(FakeContext(), 0x14);
		REQUIRE(claim.active());
		registry.removeOwner(&owner);
		claim.reset();
	}

	CHECK(owner.releaseCalls == 0);
	CHECK(registry.ownerNames().empty());

	registry.addOwner(&owner, { "gpu", 0 });
	CHECK(registry.tryClaim(FakeContext(), 0x14).active());
}
