# Unit tests for hdt::DecimateCollisionMesh
#
# Build with:  cmake -DHDTSMP_BUILD_TESTS=ON …
# Run with:    ctest --test-dir <build>/tests
#
# Dependencies pulled in here (not the full plugin stack):
#   • Bullet physics  — btVector3 etc. used by hdtVertex / hdtMeshDecimator
#   • Catch2 v3       — test framework, downloaded via FetchContent (BSL-1.0 licence,
#                       compatible with the project's GPL-3.0)
#
# Skyrim/SKSE headers are NOT needed: hdtMeshDecimator.cpp only includes
# its own header → hdtVertex.h → hdtBone.h → hdtBulletHelper.h (Bullet only).

cmake_minimum_required(VERSION 3.22)

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")

# ── Catch2 ────────────────────────────────────────────────────────────────────
# FetchContent downloads Catch2 into the build directory at configure time;
# nothing is added to the source tree.  GIT_SHALLOW=TRUE avoids fetching the
# full git history, which keeps the initial configure fast.

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(Catch2)

# ── Bullet physics ────────────────────────────────────────────────────────────
# vcpkg's toolchain file (CMAKE_TOOLCHAIN_FILE) is already active from the
# parent scope, so find_package resolves from the vcpkg package cache.

find_package(Bullet CONFIG REQUIRED)

# ── Test executable ───────────────────────────────────────────────────────────
# Only the two translation units that the decimator actually depends on are
# compiled here — no SKSE, spdlog, TBB, detours, or CommonLibSSE.

add_executable(hdtSMP64_tests
    "${SOURCE_DIR}/hdtSkinnedMesh/hdtVertex.cpp"
    "${SOURCE_DIR}/Validator/Improvers/hdtMeshDecimator.cpp"
    "decimator/test_decimator.cpp"
)

target_compile_features(hdtSMP64_tests PRIVATE cxx_std_20)

target_compile_definitions(hdtSMP64_tests PRIVATE
    BT_USE_SSE_IN_API               # must match the main build (btVector3 ABI)
    NOMINMAX                        # prevent <windows.h> from defining min/max macros
    _SILENCE_AMP_DEPRECATION_WARNINGS
)

# Match the main plugin's key MSVC flags so both targets compile the shared
# source files under the same rules (standard conformance, UTF-8, etc.).
# /WX (warnings-as-errors) is intentionally omitted for the test target because
# Catch2 internal headers may produce warnings we do not control.
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_options(hdtSMP64_tests PRIVATE
        "/utf-8"
        "/permissive-"
        "/Zc:preprocessor"
        "/wd4100"   # unreferenced formal parameter — common in Catch2 fixtures
        "/wd4201"   # nameless struct/union — used in btVector3
    )
endif()

# hdt::Vertex's default constructor calls ZeroMemory, which requires <windows.h>.
# The main plugin gets this via PCH.h → <RE/Skyrim.h>; the test target uses a
# minimal precompiled header that provides only <windows.h> to avoid pulling in
# the full SKSE/CommonLibSSE dependency tree.
target_precompile_headers(hdtSMP64_tests PRIVATE
    "decimator/test_pch.h"
)

target_include_directories(hdtSMP64_tests PRIVATE
    "${SOURCE_DIR}"         # resolves "Validator/Improvers/…" and "hdtSkinnedMesh/…"
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(hdtSMP64_tests PRIVATE
    Catch2::Catch2WithMain  # provides main(); no separate test_main.cpp needed
    ${BULLET_LIBRARIES}
)

# ── CTest integration ─────────────────────────────────────────────────────────
# catch_discover_tests queries the test binary at build time and registers each
# TEST_CASE as a separate CTest test, enabling per-test filtering and reporting.

include(CTest)
list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
include(Catch)
catch_discover_tests(hdtSMP64_tests)
