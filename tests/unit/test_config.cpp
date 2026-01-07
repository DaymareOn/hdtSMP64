#include "../include/catch.hpp"
#include <string>
#include <vector>

// Mock/standalone tests for config parsing logic
// These tests validate XML parsing and config value handling

TEST_CASE("Config value parsing", "[config]") {
    SECTION("Parse integer values") {
        // Test integer parsing similar to config.cpp readUInt/readInt
        auto parseUInt = [](const char* str, unsigned int def) -> unsigned int {
            if (!str) return def;
            try {
                return std::stoul(str);
            } catch (...) {
                return def;
            }
        };

        REQUIRE(parseUInt("100", 0) == 100);
        REQUIRE(parseUInt("0", 50) == 0);
        REQUIRE(parseUInt(nullptr, 42) == 42);
        REQUIRE(parseUInt("invalid", 99) == 99);
        REQUIRE(parseUInt("", 10) == 10);
    }

    SECTION("Parse float values") {
        auto parseFloat = [](const char* str, float def) -> float {
            if (!str) return def;
            try {
                return std::stof(str);
            } catch (...) {
                return def;
            }
        };

        REQUIRE(parseFloat("1.5", 0.0f) == Approx(1.5f));
        REQUIRE(parseFloat("0.0", 1.0f) == Approx(0.0f));
        REQUIRE(parseFloat(nullptr, 3.14f) == Approx(3.14f));
        REQUIRE(parseFloat("invalid", 2.0f) == Approx(2.0f));
    }

    SECTION("Parse boolean values") {
        auto parseBool = [](const char* str, bool def) -> bool {
            if (!str) return def;
            std::string s(str);
            if (s == "true" || s == "1") return true;
            if (s == "false" || s == "0") return false;
            return def;
        };

        REQUIRE(parseBool("true", false) == true);
        REQUIRE(parseBool("false", true) == false);
        REQUIRE(parseBool("1", false) == true);
        REQUIRE(parseBool("0", true) == false);
        REQUIRE(parseBool(nullptr, true) == true);
        REQUIRE(parseBool("invalid", false) == false);
    }
}

TEST_CASE("Frame rate calculations", "[physics]") {
    SECTION("Time tick from FPS") {
        // Verify fps-to-tick conversion matches config.cpp logic
        auto fpsToTick = [](int fps) -> float {
            return 1.0f / static_cast<float>(fps);
        };

        REQUIRE(fpsToTick(60) == Approx(1.0f / 60.0f));
        REQUIRE(fpsToTick(120) == Approx(1.0f / 120.0f));
        REQUIRE(fpsToTick(30) == Approx(1.0f / 30.0f));
    }

    SECTION("Substep limiting") {
        // Test max substep clamping logic from SkyrimPhysicsWorld
        auto clampSubsteps = [](float accumulated, float tick, int maxSubsteps) -> float {
            return std::min(accumulated, tick * maxSubsteps);
        };

        float tick = 1.0f / 60.0f;  // 60 FPS
        int maxSubsteps = 4;

        // Normal case - small accumulation
        REQUIRE(clampSubsteps(tick * 2, tick, maxSubsteps) == Approx(tick * 2));

        // Clamped case - large accumulation
        REQUIRE(clampSubsteps(tick * 10, tick, maxSubsteps) == Approx(tick * 4));
    }
}

TEST_CASE("Exponential moving average", "[physics]") {
    SECTION("EMA converges to stable value") {
        // Test the averaging logic from SkyrimPhysicsWorld::doUpdate
        float average = 1.0f / 60.0f;  // Start at 60 FPS
        float alpha = 0.125f;

        // Feed constant 30 FPS intervals
        float targetInterval = 1.0f / 30.0f;
        for (int i = 0; i < 100; i++) {
            average += (targetInterval - average) * alpha;
        }

        // Should converge close to target
        REQUIRE(average == Approx(targetInterval).epsilon(0.01));
    }

    SECTION("EMA responds to sudden changes") {
        float average = 1.0f / 60.0f;
        float alpha = 0.125f;

        // Sudden spike
        float spike = 1.0f / 15.0f;  // 15 FPS
        average += (spike - average) * alpha;

        // Should partially move toward spike
        REQUIRE(average > 1.0f / 60.0f);
        REQUIRE(average < spike);
    }
}
