#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <threepp/math/Vector3.hpp>
#include "catch2/catch_approx.hpp"

threepp::Vector3 getTankHalfExtents() {
    // based on body geometry
    return threepp::Vector3(3.0f/2.0f, 1.2f/2.0f + 1.0f, 5.0f/2.0f);
}

// test clamp function
float clampf(float v, float a, float b) {
    return std::max(a, std::min(b, v));
}

TEST_CASE("Clamp function works correctly", "[math]") {
    REQUIRE(clampf(5.0f, 0.0f, 10.0f) == 5.0f);
    REQUIRE(clampf(-5.0f, 0.0f, 10.0f) == 0.0f);
    REQUIRE(clampf(15.0f, 0.0f, 10.0f) == 10.0f);
}

TEST_CASE("Tank collision box is valid", "[collision]") {
    threepp::Vector3 extents = getTankHalfExtents();

    REQUIRE(extents.x > 0.0f);
    REQUIRE(extents.y > 0.0f);
    REQUIRE(extents.z > 0.0f);

    REQUIRE(extents.x == Catch::Approx(1.5f));
    REQUIRE(extents.z == Catch::Approx(2.5f));
}
