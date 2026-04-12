set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_compile_definitions(_SILENCE_AMP_DEPRECATION_WARNINGS)

set(SOURCE_DIR "${ROOT_DIR}/src")

set(SOURCE_FILES
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtAABB.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtBone.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtBoneScaleConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtBoneScaleConstraint.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtBulletHelper.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtCollider.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtCollider.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtCollisionAlgorithm.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtCollisionAlgorithm.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtConeTwistConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtConeTwistConstraint.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtConstraintGroup.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtDispatcher.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtDispatcher.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtGeneric6DofConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshBody.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshBody.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshBone.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshBone.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshShape.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshSystem.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshWorld.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshWorld.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtStiffSpringConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtStiffSpringConstraint.h"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtVertex.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtVertex.h"
	"${SOURCE_DIR}/Events.h"
	"${SOURCE_DIR}/Events.cpp"
	"${SOURCE_DIR}/HavokUtils.h"
	"${SOURCE_DIR}/Hooks.cpp"
	"${SOURCE_DIR}/Hooks.h"
	"${SOURCE_DIR}/dhdtPapyrusFunctions.cpp"
	"${SOURCE_DIR}/dhdtPapyrusFunctions.h"
	"${SOURCE_DIR}/EventDebugLogger.cpp"
	"${SOURCE_DIR}/EventDebugLogger.h"
	"${SOURCE_DIR}/Events.cpp"
	"${SOURCE_DIR}/Events.h"
	"${SOURCE_DIR}/dhdtOverrideManager.cpp"
	"${SOURCE_DIR}/dhdtOverrideManager.h"
	"${SOURCE_DIR}/hdtForceUpdateList.cpp"
	"${SOURCE_DIR}/hdtForceUpdateList.h"
	"${SOURCE_DIR}/config.cpp"
	"${SOURCE_DIR}/config.h"
	"${SOURCE_DIR}/hdtSerialization.cpp"
	"${SOURCE_DIR}/hdtSerialization.h"
	"${SOURCE_DIR}/PluginAPI.h"
	"${SOURCE_DIR}/PluginInterfaceImpl.cpp"
	"${SOURCE_DIR}/PluginInterfaceImpl.h"
	"${SOURCE_DIR}/DynamicHDT.cpp"
	"${SOURCE_DIR}/DynamicHDT.h"
	"${SOURCE_DIR}/WeatherManager.cpp"
	"${SOURCE_DIR}/WeatherManager.h"
	"${SOURCE_DIR}/ActorManager.cpp"
	"${SOURCE_DIR}/ActorManager.h"
	"${SOURCE_DIR}/hdtSkyrimPhysicsWorld.cpp"
	"${SOURCE_DIR}/hdtSkyrimPhysicsWorld.h"
	"${SOURCE_DIR}/hdtDefaultBBP.cpp"
	"${SOURCE_DIR}/hdtDefaultBBP.h"
	"${SOURCE_DIR}/hdtSkyrimBody.cpp"
	"${SOURCE_DIR}/hdtSkyrimBody.h"
	"${SOURCE_DIR}/hdtSkyrimBone.cpp"
	"${SOURCE_DIR}/hdtSkyrimBone.h"
	"${SOURCE_DIR}/hdtConvertNi.cpp"
	"${SOURCE_DIR}/hdtConvertNi.h"
	"${SOURCE_DIR}/hdtSkyrimSystem.cpp"
	"${SOURCE_DIR}/hdtSkyrimSystem.h"
	"${SOURCE_DIR}/XmlInspector/CharactersReader.hpp"
	"${SOURCE_DIR}/XmlInspector/CharactersWriter.hpp"
	"${SOURCE_DIR}/XmlInspector/XmlInspector.hpp"
	"${SOURCE_DIR}/XmlReader.h"
	"${SOURCE_DIR}/XmlReader.cpp"
	"${SOURCE_DIR}/NetImmerseUtils.h"
	"${SOURCE_DIR}/main.h"
	"${SOURCE_DIR}/main.cpp"
	"${SOURCE_DIR}/PCH.h")
source_group(TREE "${ROOT_DIR}" FILES ${SOURCE_FILES})

set(VERSION_HEADER "${CMAKE_CURRENT_BINARY_DIR}/src/Plugin.h")

configure_file("${ROOT_DIR}/cmake/Plugin.h.in" "${VERSION_HEADER}" @ONLY)

source_group("src" FILES "${VERSION_HEADER}")

configure_file("${ROOT_DIR}/cmake/version.rc.in" "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)

find_path(DETOURS_INCLUDE_DIRS "detours/detours.h")
# find_library resolves to whichever variant it finds first — on vcpkg it tends to pick debug/lib even for Release
# builds.  Look up both variants explicitly so we can pass them with the optimized/debug keywords below. Prefer the
# public VCPKG_INSTALLED_DIR variable; fall back to the internal _VCPKG_INSTALLED_DIR for older vcpkg, and error if
# neither is set.
if(DEFINED VCPKG_INSTALLED_DIR)
	set(_detours_vcpkg_installed "${VCPKG_INSTALLED_DIR}")
elseif(DEFINED _VCPKG_INSTALLED_DIR)
	set(_detours_vcpkg_installed "${_VCPKG_INSTALLED_DIR}")
else()
	message(FATAL_ERROR "detours: could not determine vcpkg installed directory (set VCPKG_INSTALLED_DIR)")
endif()
find_library(
	DETOURS_LIBRARY_RELEASE
	NAMES detours
	NO_DEFAULT_PATH
	PATHS "${_detours_vcpkg_installed}/${VCPKG_TARGET_TRIPLET}/lib")
find_library(
	DETOURS_LIBRARY_DEBUG
	NAMES detours
	NO_DEFAULT_PATH
	PATHS "${_detours_vcpkg_installed}/${VCPKG_TARGET_TRIPLET}/debug/lib")
if(NOT DETOURS_LIBRARY_RELEASE)
	message(FATAL_ERROR "detours: release library not found in vcpkg installed dir")
endif()
if(NOT DETOURS_LIBRARY_DEBUG)
	set(DETOURS_LIBRARY_DEBUG ${DETOURS_LIBRARY_RELEASE})
endif()

add_library("${PROJECT_NAME}" SHARED ${SOURCE_FILES} "${VERSION_HEADER}" "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
									 "${ROOT_DIR}/.clang-format" "${ROOT_DIR}/.editorconfig")

target_compile_features("${PROJECT_NAME}" PRIVATE cxx_std_20)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(
		"${PROJECT_NAME}"
		PRIVATE "/sdl" # Enable Additional Security Checks
				"/utf-8" # Set Source and Executable character sets to UTF-8
				"/Zi" # Debug Information Format
				"/permissive-" # Standards conformance
				"/Zc:preprocessor" # Enable preprocessor conformance mode
				"/bigobj" # Increase Number of Sections in .obj file
				"/WX" # Warnings as errors
				"$<$<CONFIG:DEBUG>:>"
				"$<$<CONFIG:RELEASE>:/Zc:inline;/JMC-;/Ob3>")

	target_link_options("${PROJECT_NAME}" PRIVATE "$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
						"$<$<CONFIG:RELEASE>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>")
endif()

target_include_directories("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/src" "${SOURCE_DIR}"
													 ${DETOURS_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS})

set(SKSE_SUPPORT_XBYAK ON)

# dependency macros
macro(find_dependency_path DEPENDENCY FILE)
	# searches extern for dependencies and if not checks the environment variable
	if(NOT ${DEPENDENCY} STREQUAL "")
		# Check extern
		message(STATUS "Searching for ${DEPENDENCY} using file ${FILE}")
		find_path(PATH ${FILE} PATHS "../extern/${DEPENDENCY}" "extern/${DEPENDENCY}" "../external/${DEPENDENCY}"
									 "external/${DEPENDENCY}")

		set("${DEPENDENCY}Path" "${PATH}")
		message("${PATH}")
		message("${DEPENDENCY}Path")
		message("${${DEPENDENCY}Path}")
		if("${${DEPENDENCY}Path}" STREQUAL "PATH-NOTFOUND")
			# Check path
			message(STATUS "Getting environment variable for ${DEPENDENCY}Path: $ENV{${DEPENDENCY}Path}")
			set("${DEPENDENCY}Path" "$ENV{${DEPENDENCY}Path}")
		endif()

		if(NOT "${${DEPENDENCY}Path}" STREQUAL "")
			message(STATUS "Found ${DEPENDENCY} in ${${DEPENDENCY}Path}; adding")
			add_subdirectory("${${DEPENDENCY}Path}" ${DEPENDENCY})
		endif()
	endif()
endmacro()

# dependencies
find_dependency_path(CommonLibSSE include/REL/Relocation.h)

if(("${CommonLibSSEPath}" STREQUAL "CommonLibSSEPath-NOTFOUND") OR "${CommonLibSSEPath}" STREQUAL "")
	# fallback to CommonLibSSEPath_NG from environment
	message(STATUS "Found CommonLibSSE from CommonLibSSEPath_NG environment variable")
	add_subdirectory("$ENV{CommonLibSSEPath_NG}" CommonLibSSE EXCLUDE_FROM_ALL)
endif()

find_package(Bullet CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(xbyak REQUIRED CONFIG)

include_directories(${BULLET_INCLUDE_DIRS})

target_link_libraries(
	"${PROJECT_NAME}"
	PRIVATE CommonLibSSE::CommonLibSSE
			xbyak::xbyak
			spdlog::spdlog
			optimized
			${DETOURS_LIBRARY_RELEASE}
			debug
			${DETOURS_LIBRARY_DEBUG}
			${BULLET_LIBRARIES})

target_precompile_headers("${PROJECT_NAME}" PRIVATE "${SOURCE_DIR}/PCH.h")

install(
	FILES "$<TARGET_FILE:${PROJECT_NAME}>"
	DESTINATION "SKSE/Plugins"
	COMPONENT "main")

install(
	FILES "$<TARGET_PDB_FILE:${PROJECT_NAME}>"
	DESTINATION "/"
	COMPONENT "pdbs")

if("${COPY_OUTPUT}")
	add_custom_command(
		TARGET "${PROJECT_NAME}"
		POST_BUILD
		COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:${PROJECT_NAME}>"
				"${CompiledPluginsPath}/SKSE/Plugins/"
		COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_PDB_FILE:${PROJECT_NAME}>"
				"${CompiledPluginsPath}/SKSE/Plugins/"
		VERBATIM)
endif()
