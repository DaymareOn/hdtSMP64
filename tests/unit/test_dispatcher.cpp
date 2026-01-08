#include "../include/catch.hpp"
#include <cstdint>
#include <vector>
#include <memory>

// Standalone tests for collision dispatcher logic
// These validate the core dispatcher algorithms without requiring Bullet Physics

// BUG-002 regression test: Copy-paste error where body0->getUserPointer() was used
// instead of body1->getUserPointer() for the second bone lookup.
// This test validates that the dispatcher correctly retrieves user pointers from
// BOTH collision objects, not just the first one.

namespace {
    // Mock collision object to test getUserPointer logic
    struct MockCollisionObject {
        void* userPointer = nullptr;

        void setUserPointer(void* ptr) { userPointer = ptr; }
        void* getUserPointer() const { return userPointer; }
    };

    // Mock bone structure (simplified from SkinnedMeshBone)
    struct MockBone {
        int boneId;
        uint32_t collisionGroup;
        uint32_t collisionMask;

        bool canCollideWith(const MockBone* other) const {
            if (!other) return false;
            return (collisionGroup & other->collisionMask) &&
                   (other->collisionGroup & collisionMask);
        }
    };

    // Simulates the needsCollision logic from hdtDispatcher.cpp lines 58-64
    // The BUG was that rb1 was assigned from body0->getUserPointer() instead of body1
    bool needsCollisionCorrect(MockCollisionObject* body0, MockCollisionObject* body1) {
        auto rb0 = static_cast<MockBone*>(body0->getUserPointer());
        auto rb1 = static_cast<MockBone*>(body1->getUserPointer());  // CORRECT: body1

        if (!rb0 || !rb1) return false;
        return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
    }

    // The BUGGY version that was in the code before fix
    bool needsCollisionBuggy(MockCollisionObject* body0, MockCollisionObject* body1) {
        auto rb0 = static_cast<MockBone*>(body0->getUserPointer());
        auto rb1 = static_cast<MockBone*>(body0->getUserPointer());  // BUG: body0 instead of body1

        if (!rb0 || !rb1) return false;
        return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
    }
}

TEST_CASE("BUG-002: Dispatcher getUserPointer retrieval", "[dispatcher][regression]") {

    SECTION("Correct implementation uses both body pointers") {
        MockBone bone0 = {0, 0x0001, 0x0002};  // Group 1, collides with group 2
        MockBone bone1 = {1, 0x0002, 0x0001};  // Group 2, collides with group 1

        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&bone0);
        obj1.setUserPointer(&bone1);

        // Correct implementation should detect collision
        REQUIRE(needsCollisionCorrect(&obj0, &obj1) == true);
    }

    SECTION("Buggy implementation fails to detect valid collision") {
        MockBone bone0 = {0, 0x0001, 0x0002};  // Group 1, collides with group 2
        MockBone bone1 = {1, 0x0002, 0x0001};  // Group 2, collides with group 1

        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&bone0);
        obj1.setUserPointer(&bone1);

        // Buggy implementation: Uses bone0 twice, so it checks bone0 vs bone0
        // bone0 (group 1) cannot collide with itself (mask 2 doesn't include group 1)
        REQUIRE(needsCollisionBuggy(&obj0, &obj1) == false);
    }

    SECTION("Distinction matters when bones have different collision settings") {
        // This is the key test - different bones with complementary collision groups
        MockBone clothBone = {0, 0x0004, 0x0008};   // Cloth group, collides with body
        MockBone bodyBone = {1, 0x0008, 0x0004};    // Body group, collides with cloth

        MockCollisionObject clothObj, bodyObj;
        clothObj.setUserPointer(&clothBone);
        bodyObj.setUserPointer(&bodyBone);

        // CORRECT: cloth can collide with body
        REQUIRE(needsCollisionCorrect(&clothObj, &bodyObj) == true);

        // BUGGY: Would check cloth vs cloth (false) because it reads body0 twice
        REQUIRE(needsCollisionBuggy(&clothObj, &bodyObj) == false);
    }

    SECTION("Self-collision bones work correctly") {
        // A bone that CAN collide with itself
        MockBone selfCollideBone = {0, 0x0001, 0x0001};

        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&selfCollideBone);
        obj1.setUserPointer(&selfCollideBone);

        // Both implementations give same result for self-collision
        // (This wouldn't catch the bug)
        REQUIRE(needsCollisionCorrect(&obj0, &obj1) == needsCollisionBuggy(&obj0, &obj1));
    }

    SECTION("Null pointer handling") {
        MockBone bone = {0, 0x0001, 0x0001};
        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&bone);
        obj1.setUserPointer(nullptr);

        // Should return false for null pointers
        REQUIRE(needsCollisionCorrect(&obj0, &obj1) == false);
    }
}

TEST_CASE("Collision pair symmetry", "[dispatcher]") {
    // Verify collision detection is symmetric - order shouldn't matter

    MockBone boneA = {0, 0x0001, 0x0002};
    MockBone boneB = {1, 0x0002, 0x0001};

    MockCollisionObject objA, objB;
    objA.setUserPointer(&boneA);
    objB.setUserPointer(&boneB);

    SECTION("Order A,B equals order B,A") {
        bool resultAB = needsCollisionCorrect(&objA, &objB);
        bool resultBA = needsCollisionCorrect(&objB, &objA);

        REQUIRE(resultAB == resultBA);
    }
}

TEST_CASE("Collision group edge cases", "[dispatcher]") {

    SECTION("Zero mask means no collision") {
        MockBone bone0 = {0, 0x0001, 0x0000};  // Group 1, collides with nothing
        MockBone bone1 = {1, 0x0002, 0xFFFF};  // Group 2, collides with everything

        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&bone0);
        obj1.setUserPointer(&bone1);

        // bone0's mask is 0, so it can't collide with anything
        REQUIRE(needsCollisionCorrect(&obj0, &obj1) == false);
    }

    SECTION("Maximum mask allows all collisions") {
        MockBone bone0 = {0, 0x0001, 0xFFFF};  // Group 1, collides with all
        MockBone bone1 = {1, 0x0002, 0xFFFF};  // Group 2, collides with all

        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&bone0);
        obj1.setUserPointer(&bone1);

        REQUIRE(needsCollisionCorrect(&obj0, &obj1) == true);
    }

    SECTION("One-way collision mask") {
        MockBone bone0 = {0, 0x0001, 0x0002};  // Group 1, collides with group 2
        MockBone bone1 = {1, 0x0002, 0x0000};  // Group 2, collides with nothing

        MockCollisionObject obj0, obj1;
        obj0.setUserPointer(&bone0);
        obj1.setUserPointer(&bone1);

        // Both bones must agree for collision to occur
        // bone0 wants to collide with bone1, but bone1 doesn't want to collide with anyone
        REQUIRE(needsCollisionCorrect(&obj0, &obj1) == false);
    }
}
