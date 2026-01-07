#include "../include/catch.hpp"
#include <cstdint>
#include <vector>
#include <algorithm>

// Standalone collision filtering tests
// These validate the logic without requiring Bullet Physics

TEST_CASE("Collision group filtering", "[collision]") {
    // Collision groups use bitmasks for filtering
    // Group 0 = no collision, higher bits = collision groups

    SECTION("Same group collides with itself") {
        uint32_t groupA = 0x0001;
        uint32_t maskA = 0x0001;
        uint32_t groupB = 0x0001;
        uint32_t maskB = 0x0001;

        bool collides = (groupA & maskB) && (groupB & maskA);
        REQUIRE(collides == true);
    }

    SECTION("Different groups with matching masks collide") {
        uint32_t groupA = 0x0001;
        uint32_t maskA = 0x0003;  // Collides with groups 1 and 2
        uint32_t groupB = 0x0002;
        uint32_t maskB = 0x0003;  // Collides with groups 1 and 2

        bool collides = (groupA & maskB) && (groupB & maskA);
        REQUIRE(collides == true);
    }

    SECTION("Different groups with non-matching masks don't collide") {
        uint32_t groupA = 0x0001;
        uint32_t maskA = 0x0001;  // Only collides with group 1
        uint32_t groupB = 0x0002;
        uint32_t maskB = 0x0002;  // Only collides with group 2

        bool collides = (groupA & maskB) && (groupB & maskA);
        REQUIRE(collides == false);
    }

    SECTION("Zero mask means no collision") {
        uint32_t groupA = 0x0001;
        uint32_t maskA = 0x0000;  // Collides with nothing
        uint32_t groupB = 0x0001;
        uint32_t maskB = 0xFFFF;  // Collides with everything

        bool collides = (groupA & maskB) && (groupB & maskA);
        REQUIRE(collides == false);
    }
}

TEST_CASE("Per-vertex collision detection", "[collision]") {
    // Simplified AABB overlap test (similar to hdtAabb logic)
    struct AABB {
        float minX, minY, minZ;
        float maxX, maxY, maxZ;

        bool overlaps(const AABB& other) const {
            return (minX <= other.maxX && maxX >= other.minX) &&
                   (minY <= other.maxY && maxY >= other.minY) &&
                   (minZ <= other.maxZ && maxZ >= other.minZ);
        }
    };

    SECTION("Overlapping AABBs") {
        AABB a = {0, 0, 0, 10, 10, 10};
        AABB b = {5, 5, 5, 15, 15, 15};
        REQUIRE(a.overlaps(b) == true);
        REQUIRE(b.overlaps(a) == true);
    }

    SECTION("Non-overlapping AABBs") {
        AABB a = {0, 0, 0, 10, 10, 10};
        AABB b = {20, 20, 20, 30, 30, 30};
        REQUIRE(a.overlaps(b) == false);
    }

    SECTION("Touching AABBs (edge case)") {
        AABB a = {0, 0, 0, 10, 10, 10};
        AABB b = {10, 0, 0, 20, 10, 10};  // Touches on X edge
        REQUIRE(a.overlaps(b) == true);  // Touching counts as overlap
    }

    SECTION("Contained AABB") {
        AABB outer = {0, 0, 0, 100, 100, 100};
        AABB inner = {25, 25, 25, 75, 75, 75};
        REQUIRE(outer.overlaps(inner) == true);
        REQUIRE(inner.overlaps(outer) == true);
    }
}

TEST_CASE("Collision pair deduplication", "[collision]") {
    // Test the logic for avoiding duplicate collision checks
    using CollisionPair = std::pair<int, int>;

    auto makeOrderedPair = [](int a, int b) -> CollisionPair {
        return a < b ? CollisionPair{a, b} : CollisionPair{b, a};
    };

    SECTION("Pairs are consistently ordered") {
        REQUIRE(makeOrderedPair(1, 2) == makeOrderedPair(2, 1));
        REQUIRE(makeOrderedPair(5, 3) == makeOrderedPair(3, 5));
    }

    SECTION("Duplicate pairs are detected") {
        std::vector<CollisionPair> pairs;
        auto addPair = [&](int a, int b) {
            auto pair = makeOrderedPair(a, b);
            if (std::find(pairs.begin(), pairs.end(), pair) == pairs.end()) {
                pairs.push_back(pair);
            }
        };

        addPair(1, 2);
        addPair(2, 1);  // Duplicate
        addPair(1, 3);
        addPair(3, 1);  // Duplicate

        REQUIRE(pairs.size() == 2);
    }
}

TEST_CASE("Skeleton distance calculations", "[actor]") {
    // Test actor prioritization by distance (ActorManager logic)
    struct SkeletonInfo {
        int id;
        float distance;
        bool isPlayer;
    };

    SECTION("Sort by distance, player first") {
        std::vector<SkeletonInfo> skeletons = {
            {1, 100.0f, false},
            {2, 50.0f, false},
            {3, 200.0f, true},   // Player, far away
            {4, 25.0f, false},
        };

        std::sort(skeletons.begin(), skeletons.end(),
            [](const SkeletonInfo& a, const SkeletonInfo& b) {
                // Player always first
                if (a.isPlayer != b.isPlayer) return a.isPlayer;
                // Then by distance
                return a.distance < b.distance;
            });

        REQUIRE(skeletons[0].id == 3);  // Player first
        REQUIRE(skeletons[1].id == 4);  // Closest NPC
        REQUIRE(skeletons[2].id == 2);
        REQUIRE(skeletons[3].id == 1);  // Furthest NPC
    }

    SECTION("Active skeleton limit") {
        std::vector<SkeletonInfo> skeletons;
        for (int i = 0; i < 20; i++) {
            skeletons.push_back({i, static_cast<float>(i * 10), false});
        }

        int maxActive = 5;
        int activeCount = std::min(static_cast<int>(skeletons.size()), maxActive);

        REQUIRE(activeCount == 5);
    }
}
