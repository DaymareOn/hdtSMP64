cmake_minimum_required(VERSION 3.22)

project(
	hdtsmp64
	VERSION 3.0.0
	LANGUAGES CXX)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pre-release build identifier (e.g. "dev-abc1234"). Empty for production builds.
set(PROJECT_BUILD_INFO
	""
	CACHE STRING "Pre-release build info injected by CI (e.g. dev-abc1234)")

if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
	message(FATAL_ERROR "in-source builds are not allowed")
endif()

macro(set_from_environment VARIABLE)
	if(NOT DEFINED ${VARIABLE} AND DEFINED ENV{${VARIABLE}})
		set(${VARIABLE} $ENV{${VARIABLE}})
	endif()
endmacro()
set_from_environment(CompiledPluginsPath)
if(NOT DEFINED CompiledPluginsPath)
	message(FATAL_ERROR "CompiledPluginsPath is not set")
endif()

option(COPY_OUTPUT "Copy the output of build operations to the game directory" ON)
option(ENABLE_SKYRIM_SE "Enable support for Skyrim SE in the dynamic runtime feature." ON)
option(ENABLE_SKYRIM_AE "Enable support for Skyrim AE in the dynamic runtime feature." ON)
option(ENABLE_SKYRIM_VR "Enable support for Skyrim VR in the dynamic runtime feature." ON)

#
set(BUILD_TESTS OFF)

# Standalone physics replay/benchmark tool (smp_replay) + its unit tests. ON by default: it builds
# alongside the mod and (with COPY_OUTPUT) is delivered into the same SKSE/Plugins folder as the DLL,
# so it ships with the mod, including the released package. Turn it OFF for a leaner DLL-only build.
option(BUILD_BENCHMARK "Build the standalone smp_replay benchmark/replay tool and its tests" ON)

# default definitions
add_definitions(-DBT_USE_SSE_IN_API)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

add_subdirectory(src)

if(BUILD_BENCHMARK)
	add_subdirectory(tests/benchmark)
	# Tie the tool to the mod target so building the mod always builds (and, via COPY_OUTPUT, delivers)
	# smp_replay too - even when someone builds only the DLL target.
	add_dependencies("${PROJECT_NAME}" smp_replay)
endif()

include(cmake/packaging.cmake)
