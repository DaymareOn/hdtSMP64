cmake_minimum_required(VERSION 3.22)

project(
	hdtsmp64
	VERSION 3.0.4
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
# Enable with:  cmake -DHDTSMP_BUILD_TESTS=ON …
# Requires an internet connection on first configure (Catch2 via FetchContent).
# Named HDTSMP_BUILD_TESTS (not BUILD_TESTS) to avoid clashing with CommonLibSSE's
# own BUILD_TESTS option, which expects Catch2 to be installed via vcpkg.
option(HDTSMP_BUILD_TESTS "Build the hdtSMP64_tests unit-test executable" OFF)

# default definitions
add_definitions(-DBT_USE_SSE_IN_API)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# CommonLibSSE's BUILD_TESTS option defaults to ON and expects Catch2 from vcpkg,
# which is not in this project's vcpkg manifest.  Force it off before the
# subdirectory is added so CommonLibSSE's option() call sees it already cached.
set(BUILD_TESTS OFF CACHE BOOL "CommonLibSSE test suite (disabled for hdtSMP64)" FORCE)

add_subdirectory(src)
include(cmake/packaging.cmake)

if(HDTSMP_BUILD_TESTS)
    add_subdirectory(tests)
endif()
