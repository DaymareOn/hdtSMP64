set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_compile_definitions(_SILENCE_AMP_DEPRECATION_WARNINGS NOMINMAX)

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
	"${SOURCE_DIR}/hdtPhysicsProfiler.cpp"
	"${SOURCE_DIR}/hdtPhysicsProfiler.h"
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
		PRIVATE "/utf-8" # Set Source and Executable character sets to UTF-8
				"/Zi" # Debug Information Format
				"/permissive-" # Standards conformance
				"/Zc:preprocessor" # Enable preprocessor conformance mode
				"/bigobj" # Increase Number of Sections in .obj file
				"/W4" # Elevated warnings: unreachable code (C4702), unused vars (C4189), etc.
				"/wd4100" # Suppress: unreferenced formal parameter (pervasive in SKSE callbacks)
				"/wd4201" # Suppress: nameless struct/union (used throughout CommonLibSSE)
				"/WX" # Warnings as errors
				"/external:anglebrackets" # Treat angle-bracket includes as external
				"/external:W0" # No warnings from external headers (vcpkg, system)
				# /sdl adds extra runtime security checks that cost measurably in the per-frame physics hot loops. Keep
				# it in Debug; drop it from the shipped Release/Profile builds. (/GS stack cookies are NOT dropped.)
				# Release/Profile speed flags:
				#   /Zc:inline strip unreferenced COMDATs  /JMC- disable Just-My-Code  /Ob3 most aggressive inlining
				#   /Gw       global-data COMDATs so /OPT:REF + LTCG can drop/place static data
				#   /fp:fast  let the optimizer emit FMA and reassociate FP math (large for FP-heavy physics)
				"$<$<CONFIG:DEBUG>:/analyze;/analyze:external->"  # Static analysis in Debug only; skip external headers
				"$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:/Zc:inline;/JMC-;/Ob3;/Gw;/fp:fast>")

	target_link_options("${PROJECT_NAME}" PRIVATE "$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
						"$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>")
endif()

# Whole-program optimization (LTCG/IPO) on the shipped DLL. The hot physics loops span several translation units
# (collision, body, world) plus statically-linked Bullet, so cross-TU / cross-library inlining is the single biggest
# "free" runtime win. This mirrors what CommonLibSSE already enables. Release-like configs only; adds /GL to compile and
# /LTCG to link, which coexist with the /OPT:REF /OPT:ICF above.
include(CheckIPOSupported)
check_ipo_supported(RESULT _fsmp_ipo_ok OUTPUT _fsmp_ipo_msg)
if(_fsmp_ipo_ok)
	set_property(TARGET "${PROJECT_NAME}" PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
	set_property(TARGET "${PROJECT_NAME}" PROPERTY INTERPROCEDURAL_OPTIMIZATION_PROFILE ON)
else()
	message(WARNING "FSMP: IPO/LTCG unsupported; shipping without whole-program optimization: ${_fsmp_ipo_msg}")
endif()

# Profile-Guided Optimization (opt-in, MSVC only) -- layered on top of LTCG, the largest realistic win for the hot
# paths. Because Bullet is compiled with /GL (see cmake/triplets/_perf_flags.cmake) and folded into this DLL's LTCG, a
# single PGO pass over hdtSMP64 also profiles the heavy Bullet math. Two-stage workflow, see docs/PERFORMANCE.md:
# -DFSMP_PGO=INSTRUMENT  build an instrumented DLL, then run the standard physics-heavy scene in-game to emit .pgc data
# -DFSMP_PGO=OPTIMIZE    rebuild using the merged .pgd profile
set(FSMP_PGO
	"OFF"
	CACHE STRING "Profile-Guided Optimization stage: OFF | INSTRUMENT | OPTIMIZE")
set_property(CACHE FSMP_PGO PROPERTY STRINGS OFF INSTRUMENT OPTIMIZE)
if(NOT FSMP_PGO STREQUAL "OFF")
	if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		message(FATAL_ERROR "FSMP_PGO is only supported with MSVC (got '${CMAKE_CXX_COMPILER_ID}')")
	endif()
	if(NOT _fsmp_ipo_ok)
		message(FATAL_ERROR "FSMP_PGO requires IPO/LTCG support, which this toolchain reports as unavailable")
	endif()
	# PGO requires /GL + /LTCG in whatever configuration is built, so force IPO on for the whole target.
	set_property(TARGET "${PROJECT_NAME}" PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
	if(FSMP_PGO STREQUAL "INSTRUMENT")
		target_link_options("${PROJECT_NAME}" PRIVATE "/GENPROFILE")
	elseif(FSMP_PGO STREQUAL "OPTIMIZE")
		target_link_options("${PROJECT_NAME}" PRIVATE "/USEPROFILE")
	else()
		message(FATAL_ERROR "FSMP_PGO must be OFF, INSTRUMENT, or OPTIMIZE (got '${FSMP_PGO}')")
	endif()
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
find_package(TBB CONFIG REQUIRED)

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
			${BULLET_LIBRARIES}
			TBB::tbb)

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
