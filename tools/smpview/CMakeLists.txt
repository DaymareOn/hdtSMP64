# smpview — standalone HDT-SMP physics viewer/tuner (Phase 0 foundation).
#
# This is a SELF-CONTAINED CMake project, deliberately NOT add_subdirectory()'d
# from the mod's root CMakeLists: smpcore reuses the unmodified hdtSkinnedMesh/*
# physics sources but links only Bullet + TBB (+ Windows SDK) — no CommonLibSSE,
# no SKSE. Keeping it separate means the tool builds without the mod's runtime
# dependency graph.
#
# Configure like the mod (vcpkg toolchain supplies Bullet + TBB), e.g.:
#   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
#   cmake --build build --config Release
cmake_minimum_required(VERSION 3.21)
project(smpview CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Bullet CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)

# The mod's physics core, reused verbatim. Relative to tools/smpview/.
set(CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src/hdtSkinnedMesh")
file(GLOB SMPCORE_SOURCES CONFIGURE_DEPENDS "${CORE_DIR}/*.cpp")

add_library(smpcore STATIC ${SMPCORE_SOURCES})

# Same Bullet SIMD-in-API define the mod sets (root CMakeLists.txt). Without it,
# btVector3 <-> __m128 assignments in hdtBulletHelper.h fail to compile.
target_compile_definitions(smpcore PUBLIC BT_USE_SSE_IN_API NOMINMAX)
target_compile_options(smpcore PUBLIC /arch:AVX2)

# Swap the mod's CommonLibSSE PCH for the standalone shim — this is the seam.
target_precompile_headers(smpcore PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/re_shim/smpcore_pch.h")

target_include_directories(smpcore PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}/re_shim"
	"${CMAKE_CURRENT_SOURCE_DIR}/../../src"
	"${CORE_DIR}"
	${BULLET_INCLUDE_DIRS})

target_link_libraries(smpcore PUBLIC ${BULLET_LIBRARIES} TBB::tbb)

add_executable(smpview_smoketest "${CMAKE_CURRENT_SOURCE_DIR}/src/smoketest.cpp")
target_link_libraries(smpview_smoketest PRIVATE smpcore)
