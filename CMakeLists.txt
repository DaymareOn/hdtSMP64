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

# default definitions
add_definitions(-DBT_USE_SSE_IN_API)

if(CMAKE_CONFIGURATION_TYPES AND NOT "Profile" IN_LIST CMAKE_CONFIGURATION_TYPES)
	list(APPEND CMAKE_CONFIGURATION_TYPES Profile)
	set(CMAKE_CONFIGURATION_TYPES
		"${CMAKE_CONFIGURATION_TYPES}"
		CACHE STRING "" FORCE)
endif()
foreach(_lang IN ITEMS C CXX)
	set(CMAKE_${_lang}_FLAGS_PROFILE
		"${CMAKE_${_lang}_FLAGS_RELEASE}"
		CACHE STRING "" FORCE)
endforeach()
foreach(_kind IN ITEMS EXE SHARED MODULE STATIC)
	set(CMAKE_${_kind}_LINKER_FLAGS_PROFILE
		"${CMAKE_${_kind}_LINKER_FLAGS_RELEASE}"
		CACHE STRING "" FORCE)
endforeach()
set(CMAKE_MAP_IMPORTED_CONFIG_PROFILE Release RelWithDebInfo MinSizeRel "")
add_compile_definitions("$<$<CONFIG:Profile>:BT_ENABLE_PROFILE>")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

add_subdirectory(src)
include(cmake/packaging.cmake)
