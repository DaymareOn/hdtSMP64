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

# Headless doctest suite for the XML reader contract and the validator's inert-<bone> warning (a dev/CI tool, never
# packaged; no CommonLibSSE), mirroring tests/patterns.
option(BUILD_VALIDATOR_TESTS "Build the validator/XML-reader doctest suite" OFF)

# CommonLibSSE consumes a variable of this exact name to gate its own Catch2-based tests; keep them off (Catch2 is not
# in our vcpkg manifest).
set(BUILD_TESTS OFF)

# Standalone doctest suite for the SMP XML pattern expander (PLAN smp-xml-patterns, D5). OFF by default so
# normal/packaged builds stay DLL-only. Unlike BUILD_VALIDATOR_TESTS, CI does NOT currently turn this ON --
# it is a documented exception in .github/workflows/validate-build-test-flags.yml pending a real
# Windows/MSVC/vcpkg CI run to confirm the suite still compiles (issue #466). Turn it ON locally to build
# and run the suite in the meantime.
option(BUILD_PATTERN_TESTS "Build the SMP XML pattern expander doctest suite" OFF)

# Standalone doctest suite for the GlobalConfig JSON (de)serialization. OFF by default and never packaged; it compiles
# src/GlobalConfig.cpp directly without the game's PCH and links only rapidjson + doctest (no CommonLibSSE/Bullet/TBB),
# mirroring tests/patterns. Also NOT currently turned on by CI -- see BUILD_PATTERN_TESTS's comment above
# and issue #466; the same documented exception applies here.
option(BUILD_CONFIG_TESTS "Build the GlobalConfig JSON doctest suite" OFF)

# default definitions
add_definitions(-DBT_USE_SSE_IN_API)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

add_subdirectory(src)

if(BUILD_CONFIG_TESTS OR BUILD_VALIDATOR_TESTS)
	enable_testing()
endif()
if(BUILD_CONFIG_TESTS)
	add_subdirectory(tests/config)
endif()
if(BUILD_VALIDATOR_TESTS)
	add_subdirectory(tests/validator)
endif()

if(BUILD_PATTERN_TESTS)
	add_subdirectory(tests/patterns) # tests/patterns/CMakeLists.txt owns enable_testing()
endif()

include(cmake/packaging.cmake)
