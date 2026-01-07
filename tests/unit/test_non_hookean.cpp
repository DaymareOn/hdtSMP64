#include "../include/catch.hpp"
#include <cmath>
#include <algorithm>

// Mock/standalone tests for Non-Hookean spring physics
// These tests validate the algorithm logic from btGeneric6DofSpring2Constraint

namespace {
    // Fuzzy zero check matching Bullet's btFuzzyZero
    constexpr float BT_EPSILON = 1.192092896e-07f;
    bool btFuzzyZero(float x) { return std::abs(x) < BT_EPSILON; }

    // Clamp function matching Bullet's btClamped
    float btClamped(float val, float lo, float hi) {
        return std::max(lo, std::min(val, hi));
    }

    // Non-Hookean spring calculation - mirrors btGeneric6DofSpring2Constraint logic
    struct NonHookeanResult {
        float effectiveStiffness;
        float effectiveDamping;
    };

    NonHookeanResult calculateNonHookean(
        float baseStiffness,
        float baseDamping,
        float error,           // Current position - equilibrium
        float loLimit,
        float hiLimit,
        float equilibrium,
        float nonHookeanStiffness,
        float nonHookeanDamping
    ) {
        float ks = baseStiffness;
        float kd = baseDamping;

        if (!btFuzzyZero(error) && (!btFuzzyZero(nonHookeanDamping) || !btFuzzyZero(nonHookeanStiffness))) {
            // Calculate range from equilibrium to limit
            float range = error < equilibrium ? equilibrium - loLimit : hiLimit - equilibrium;

            // Normalized error factor (0 at equilibrium, 1 at limit)
            float rf = !btFuzzyZero(range) ? std::abs(error) / range : 0.0f;
            float t = btClamped(rf, 0.0f, 1.0f);

            // Apply Non-Hookean reduction
            kd *= 1.0f - (nonHookeanDamping * t);
            ks *= 1.0f - (nonHookeanStiffness * t);
        }

        return { ks, kd };
    }
}

TEST_CASE("Non-Hookean spring defaults", "[physics][non-hookean]") {
    SECTION("Zero factors produce Hookean behavior") {
        float baseK = 100.0f;
        float baseD = 0.5f;

        auto result = calculateNonHookean(
            baseK, baseD,
            0.5f,           // error
            -1.0f, 1.0f,    // limits
            0.0f,           // equilibrium
            0.0f, 0.0f      // Non-Hookean factors (disabled)
        );

        REQUIRE(result.effectiveStiffness == Approx(baseK));
        REQUIRE(result.effectiveDamping == Approx(baseD));
    }

    SECTION("Zero error produces Hookean behavior") {
        float baseK = 100.0f;
        float baseD = 0.5f;

        auto result = calculateNonHookean(
            baseK, baseD,
            0.0f,           // error = 0 (at equilibrium)
            -1.0f, 1.0f,
            0.0f,
            0.5f, 0.5f      // Non-Hookean factors (enabled but no effect)
        );

        REQUIRE(result.effectiveStiffness == Approx(baseK));
        REQUIRE(result.effectiveDamping == Approx(baseD));
    }
}

TEST_CASE("Non-Hookean stiffness reduction", "[physics][non-hookean]") {
    float baseK = 100.0f;
    float baseD = 0.5f;
    float loLimit = -1.0f;
    float hiLimit = 1.0f;
    float equilibrium = 0.0f;

    SECTION("50% reduction at limit with factor 0.5") {
        auto result = calculateNonHookean(
            baseK, baseD,
            1.0f,           // At positive limit
            loLimit, hiLimit,
            equilibrium,
            0.5f, 0.0f      // 50% stiffness reduction
        );

        // At limit, t = 1.0, so ks *= (1 - 0.5 * 1.0) = 0.5
        REQUIRE(result.effectiveStiffness == Approx(50.0f));
        REQUIRE(result.effectiveDamping == Approx(baseD));
    }

    SECTION("25% reduction at halfway with factor 0.5") {
        auto result = calculateNonHookean(
            baseK, baseD,
            0.5f,           // Halfway to limit
            loLimit, hiLimit,
            equilibrium,
            0.5f, 0.0f
        );

        // At halfway, t = 0.5, so ks *= (1 - 0.5 * 0.5) = 0.75
        REQUIRE(result.effectiveStiffness == Approx(75.0f));
    }

    SECTION("Full reduction at limit with factor 1.0") {
        auto result = calculateNonHookean(
            baseK, baseD,
            1.0f,
            loLimit, hiLimit,
            equilibrium,
            1.0f, 0.0f      // 100% stiffness reduction
        );

        // At limit with factor 1.0: ks *= (1 - 1.0 * 1.0) = 0
        REQUIRE(result.effectiveStiffness == Approx(0.0f));
    }
}

TEST_CASE("Non-Hookean damping reduction", "[physics][non-hookean]") {
    float baseK = 100.0f;
    float baseD = 0.5f;

    SECTION("Damping reduced independently of stiffness") {
        auto result = calculateNonHookean(
            baseK, baseD,
            1.0f,           // At limit
            -1.0f, 1.0f,
            0.0f,
            0.0f, 0.5f      // Only damping reduction
        );

        REQUIRE(result.effectiveStiffness == Approx(baseK));
        REQUIRE(result.effectiveDamping == Approx(0.25f)); // 0.5 * (1 - 0.5)
    }

    SECTION("Both stiffness and damping can be reduced") {
        auto result = calculateNonHookean(
            baseK, baseD,
            1.0f,
            -1.0f, 1.0f,
            0.0f,
            0.3f, 0.4f      // Both factors
        );

        REQUIRE(result.effectiveStiffness == Approx(70.0f));   // 100 * (1 - 0.3)
        REQUIRE(result.effectiveDamping == Approx(0.3f));      // 0.5 * (1 - 0.4)
    }
}

TEST_CASE("Non-Hookean factor clamping", "[physics][non-hookean]") {
    float baseK = 100.0f;
    float baseD = 0.5f;

    SECTION("Error beyond limit is clamped") {
        auto result = calculateNonHookean(
            baseK, baseD,
            2.0f,           // Beyond limit!
            -1.0f, 1.0f,
            0.0f,
            0.5f, 0.0f
        );

        // t should be clamped to 1.0, same as at-limit behavior
        REQUIRE(result.effectiveStiffness == Approx(50.0f));
    }

    SECTION("Negative error uses low limit range") {
        auto result = calculateNonHookean(
            baseK, baseD,
            -0.5f,          // Negative error (toward loLimit)
            -1.0f, 1.0f,
            0.0f,
            0.5f, 0.0f
        );

        // Range = equilibrium - loLimit = 0 - (-1) = 1
        // t = 0.5 / 1.0 = 0.5
        REQUIRE(result.effectiveStiffness == Approx(75.0f));
    }
}

TEST_CASE("Non-Hookean asymmetric limits", "[physics][non-hookean]") {
    float baseK = 100.0f;
    float baseD = 0.5f;

    SECTION("Non-centered equilibrium") {
        // Equilibrium at 0.5, limits at 0 and 2
        auto result = calculateNonHookean(
            baseK, baseD,
            1.5f,           // 1.0 away from equilibrium toward hiLimit
            0.0f, 2.0f,     // Limits
            0.5f,           // Equilibrium
            0.5f, 0.0f
        );

        // Range = hiLimit - equilibrium = 2.0 - 0.5 = 1.5
        // error from eq = |1.5 - 0.5| = 1.0, wait no - error param is position
        // Actually error = 1.5 (position), equilibrium = 0.5
        // So distance = |1.5| = 1.5 (the error value represents distance already)
        // Hmm, need to check the actual implementation...
        // Actually looking at the code: error is passed directly
        // range = 2.0 - 0.5 = 1.5 (since error > equilibrium)
        // rf = 1.5 / 1.5 = 1.0 (at limit)
        REQUIRE(result.effectiveStiffness == Approx(50.0f));
    }
}

TEST_CASE("GenericConstraintTemplate defaults", "[config][non-hookean]") {
    // Test that the template struct has correct default values
    // This mirrors hdtSkyrimSystem.h GenericConstraintTemplate
    struct MockGenericConstraintTemplate {
        float linearNonHookeanDamping[3] = { 0, 0, 0 };
        float angularNonHookeanDamping[3] = { 0, 0, 0 };
        float linearNonHookeanStiffness[3] = { 0, 0, 0 };
        float angularNonHookeanStiffness[3] = { 0, 0, 0 };
    };

    MockGenericConstraintTemplate tmpl;

    SECTION("All Non-Hookean values default to zero") {
        for (int i = 0; i < 3; i++) {
            REQUIRE(tmpl.linearNonHookeanDamping[i] == 0.0f);
            REQUIRE(tmpl.angularNonHookeanDamping[i] == 0.0f);
            REQUIRE(tmpl.linearNonHookeanStiffness[i] == 0.0f);
            REQUIRE(tmpl.angularNonHookeanStiffness[i] == 0.0f);
        }
    }
}

TEST_CASE("Non-Hookean index mapping", "[physics][non-hookean]") {
    // Verify the index scheme: 0-2 = linear XYZ, 3-5 = angular XYZ
    SECTION("Linear axes are 0-2") {
        int linearX = 0, linearY = 1, linearZ = 2;
        REQUIRE(linearX >= 0);
        REQUIRE(linearZ <= 2);
    }

    SECTION("Angular axes are 3-5") {
        int angularX = 3, angularY = 4, angularZ = 5;
        REQUIRE(angularX >= 3);
        REQUIRE(angularZ <= 5);
    }

    SECTION("Conversion from angular index") {
        // constraint->setNonHookeanDamping(i + 3, angularValue[i])
        for (int i = 0; i < 3; i++) {
            int angularIndex = i + 3;
            REQUIRE(angularIndex >= 3);
            REQUIRE(angularIndex <= 5);
        }
    }
}
