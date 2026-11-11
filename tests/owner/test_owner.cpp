#include <doctest/doctest.h>

#include "ExternalOwner.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
	class FakeContext : public hdt::IMeshClaimContext
	{
	public:
		hdt::MeshKind getKind() const override { return hdt::MeshKind::Armor; }
		const char* getPhysicsFile() const override { return physicsFile; }
		RE::Actor* getActor() const override { return nullptr; }
		RE::NiNode* getSkeleton() const override { return nullptr; }
		RE::NiAVObject* getModel() const override { return nullptr; }
		const char* getMergedBoneName(const char*) const override { return nullptr; }

		const char* physicsFile = "test.xml";
	};

	class FakeClaim : public hdt::IMeshClaim
	{
	public:
		explicit FakeClaim(int& a_releases) :
			releases(a_releases)
		{}

		void release() override
		{
			++releases;
			if (onRelease) {
				onRelease();
			}
		}

		int& releases;
		std::function<void()> onRelease;
	};

	class FakeOwner : public hdt::IMeshOwner
	{
	public:
		explicit FakeOwner(bool a_claims) :
			claims(a_claims)
		{}

		hdt::IMeshClaim* claim(const hdt::IMeshClaimContext&) override
		{
			++claimCalls;

			if (throwOnClaim) {
				throw 1;
			}

			if (onClaim) {
				onClaim();
			}

			if (!claims) {
				return nullptr;
			}

			made.push_back(std::make_unique<FakeClaim>(releaseCalls));
			return made.back().get();
		}

		bool claims = false;
		bool throwOnClaim = false;
		int claimCalls = 0;
		int releaseCalls = 0;
		std::function<void()> onClaim;
		std::vector<std::unique_ptr<FakeClaim>> made;
	};

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

		FakeOwner& add(bool a_claims, hdt::MeshOwnerInfo a_info = {})
		{
			m_owners.push_back(std::make_unique<FakeOwner>(a_claims));
			hdt::ExternalOwnerRegistry::instance().addOwner(m_owners.back().get(), a_info);
			return *m_owners.back();
		}

	private:
		std::vector<std::unique_ptr<FakeOwner>> m_owners;
	};

	auto& registry()
	{
		return hdt::ExternalOwnerRegistry::instance();
	}
}

TEST_CASE("no owners registered gives no claim")
{
	OwnerScope scope;
	CHECK(registry().tryClaim(FakeContext()) == nullptr);
}

TEST_CASE("add ignores null and duplicate owners")
{
	OwnerScope scope;

	registry().addOwner(nullptr);
	CHECK(registry().tryClaim(FakeContext()) == nullptr);

	FakeOwner& owner = scope.add(true);
	registry().addOwner(&owner);
	registry().addOwner(&owner);

	CHECK(registry().tryClaim(FakeContext()) != nullptr);
	CHECK(owner.claimCalls == 1);
}

TEST_CASE("the first owner in registration order that claims wins")
{
	OwnerScope scope;
	FakeOwner& first = scope.add(true);
	FakeOwner& second = scope.add(true);

	const auto claim = registry().tryClaim(FakeContext());
	CHECK(claim != nullptr);
	CHECK(first.claimCalls == 1);
	CHECK(second.claimCalls == 0);
}

TEST_CASE("an owner that declines is skipped and the next one is asked")
{
	OwnerScope scope;
	FakeOwner& declining = scope.add(false);
	FakeOwner& accepting = scope.add(true);

	const auto claim = registry().tryClaim(FakeContext());
	CHECK(claim != nullptr);
	CHECK(declining.claimCalls == 1);
	CHECK(accepting.claimCalls == 1);
}

TEST_CASE("a higher priority owner is asked before earlier registered ones")
{
	OwnerScope scope;
	FakeOwner& low = scope.add(true, { "low", 0 });
	FakeOwner& high = scope.add(true, { "high", 10 });

	const auto claim = registry().tryClaim(FakeContext());
	REQUIRE(claim != nullptr);
	CHECK(claim->ownerName() == "high");
	CHECK(high.claimCalls == 1);
	CHECK(low.claimCalls == 0);
}

TEST_CASE("equal priorities keep registration order and an empty name becomes unnamed")
{
	OwnerScope scope;
	scope.add(true, { "a", 5 });
	scope.add(true, { nullptr, 5 });
	scope.add(true, { "c", 5 });

	const std::vector<std::string> expected{ "a", "unnamed", "c" };
	CHECK(registry().ownerNames() == expected);
}

TEST_CASE("a claim releases its owner exactly once")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	SUBCASE("on destruction")
	{
		{
			const auto claim = registry().tryClaim(FakeContext());
			REQUIRE(claim != nullptr);
			CHECK(owner.releaseCalls == 0);
		}

		CHECK(owner.releaseCalls == 1);
	}

	SUBCASE("on release, and not again on destruction")
	{
		{
			const auto claim = registry().tryClaim(FakeContext());
			REQUIRE(claim != nullptr);

			claim->release();
			CHECK(owner.releaseCalls == 1);

			claim->release();
			CHECK(owner.releaseCalls == 1);
		}

		CHECK(owner.releaseCalls == 1);
	}
}

TEST_CASE("a copy that outlives a rebuild neither delays nor repeats the release")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	auto item = registry().tryClaim(FakeContext());
	REQUIRE(item != nullptr);
	const auto snapshot = item;

	item->release();
	item.reset();
	CHECK(owner.releaseCalls == 1);

	const auto rebuilt = registry().tryClaim(FakeContext());
	REQUIRE(rebuilt != nullptr);
	CHECK(owner.claimCalls == 2);
	CHECK(owner.releaseCalls == 1);
}

TEST_CASE("a snapshot destroyed later does not release a second time")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	{
		auto item = registry().tryClaim(FakeContext());
		const auto snapshot = item;
		item->release();
		item.reset();
		CHECK(owner.releaseCalls == 1);
	}

	CHECK(owner.releaseCalls == 1);
}

TEST_CASE("an owner that throws from claim is treated as declining")
{
	OwnerScope scope;
	FakeOwner& thrower = scope.add(true);
	thrower.throwOnClaim = true;
	FakeOwner& next = scope.add(true);

	const auto claim = registry().tryClaim(FakeContext());
	CHECK(claim != nullptr);
	CHECK(thrower.claimCalls == 1);
	CHECK(next.claimCalls == 1);
}

TEST_CASE("an owner that throws from release does not escape")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	{
		const auto claim = registry().tryClaim(FakeContext());
		REQUIRE(claim != nullptr);
		owner.made.back()->onRelease = [] { throw 1; };
	}

	CHECK(owner.releaseCalls == 1);
}

TEST_CASE("removing an owner reports the actors it had claims on and never releases them")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);
	FakeOwner& other = scope.add(true, { "other", -1 });

	const auto a = registry().tryClaim(FakeContext(), 0x14);
	const auto b = registry().tryClaim(FakeContext(), 0x14);
	const auto c = registry().tryClaim(FakeContext(), 0x1000);
	const auto d = registry().tryClaim(FakeContext(), 0);
	REQUIRE((a && b && c && d));

	auto actors = registry().removeOwner(&owner);
	std::sort(actors.begin(), actors.end());
	const std::vector<std::uint32_t> expected{ 0x14, 0x1000 };
	CHECK(actors == expected);

	CHECK(registry().removeOwner(&owner).empty());
	CHECK(owner.releaseCalls == 0);
	CHECK(other.claimCalls == 0);
}

TEST_CASE("release is not delivered to an owner that was removed after claiming")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	{
		const auto claim = registry().tryClaim(FakeContext());
		REQUIRE(claim != nullptr);

		registry().removeOwner(&owner);
		claim->release();
	}

	CHECK(owner.releaseCalls == 0);
}

TEST_CASE("a removed owner is no longer asked")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	registry().removeOwner(&owner);
	CHECK(registry().tryClaim(FakeContext()) == nullptr);
	CHECK(owner.claimCalls == 0);
}

TEST_CASE("an owner may register another owner from inside claim")
{
	OwnerScope scope;
	FakeOwner& late = scope.add(true, { "late", -5 });
	registry().removeOwner(&late);

	FakeOwner& first = scope.add(false);
	first.onClaim = [&] { registry().addOwner(&late, { "late", -5 }); };

	CHECK(registry().tryClaim(FakeContext()) == nullptr);
	CHECK(late.claimCalls == 0);
	CHECK(registry().tryClaim(FakeContext()) != nullptr);
	CHECK(late.claimCalls == 1);
}

TEST_CASE("removeOwner waits for a claim in flight on another thread")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	std::atomic<bool> inClaim{ false };
	std::atomic<bool> proceed{ false };
	std::atomic<bool> claimFinished{ false };
	owner.onClaim = [&] {
		inClaim = true;
		while (!proceed) {
			std::this_thread::yield();
		}
		claimFinished = true;
	};

	std::shared_ptr<hdt::ExternalClaim> claim;
	std::thread vm([&] { claim = registry().tryClaim(FakeContext(), 0x14); });
	while (!inClaim) {
		std::this_thread::yield();
	}

	std::atomic<bool> removed{ false };
	std::vector<std::uint32_t> actors;
	std::thread remover([&] {
		actors = registry().removeOwner(&owner);
		removed = true;
	});

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	CHECK_FALSE(removed);

	proceed = true;
	vm.join();
	remover.join();

	CHECK(claimFinished);
	CHECK(removed);
	REQUIRE(claim != nullptr);
	const std::vector<std::uint32_t> expected{ 0x14 };
	CHECK(actors == expected);

	claim->release();
	CHECK(owner.releaseCalls == 0);
}

TEST_CASE("removeOwner waits for a release in flight on another thread")
{
	OwnerScope scope;
	FakeOwner& owner = scope.add(true);

	auto claim = registry().tryClaim(FakeContext());
	REQUIRE(claim != nullptr);

	std::atomic<bool> inRelease{ false };
	std::atomic<bool> proceed{ false };
	owner.made.back()->onRelease = [&] {
		inRelease = true;
		while (!proceed) {
			std::this_thread::yield();
		}
	};

	std::thread console([&] { claim->release(); });
	while (!inRelease) {
		std::this_thread::yield();
	}

	std::atomic<bool> removed{ false };
	std::thread remover([&] {
		registry().removeOwner(&owner);
		removed = true;
	});

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	CHECK_FALSE(removed);

	proceed = true;
	console.join();
	remover.join();
	CHECK(removed);
}
