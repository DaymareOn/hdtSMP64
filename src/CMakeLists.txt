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
	"${SOURCE_DIR}/GlobalConfig.cpp"
	"${SOURCE_DIR}/GlobalConfig.h"
	"${SOURCE_DIR}/SMPDebug.h"
	"${SOURCE_DIR}/UI/FSMPMenu.cpp"
	"${SOURCE_DIR}/UI/FSMPMenu.h"
	"${SOURCE_DIR}/UI/Localization.cpp"
	"${SOURCE_DIR}/UI/Localization.h"
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
	"${SOURCE_DIR}/Patterns/hdtXmlPatternExpander.h"
	"${SOURCE_DIR}/Patterns/hdtXmlPatternExpander.cpp"
	"${SOURCE_DIR}/Patterns/hdtPatternLibrary.h"
	"${SOURCE_DIR}/Patterns/hdtPatternLibrary.cpp"
	"${SOURCE_DIR}/hdtStringUtils.h"
	"${SOURCE_DIR}/NetImmerseUtils.h"
	"${SOURCE_DIR}/Validator/hdtAssetValidator.cpp"
	"${SOURCE_DIR}/Validator/hdtAssetValidator.h"
	"${SOURCE_DIR}/Validator/Config/hdtValidatorPaths.h"
	"${SOURCE_DIR}/Validator/Schema/hdtXSDSchemaModel.h"
	"${SOURCE_DIR}/Validator/Schema/hdtSCHSchemaModel.h"
	"${SOURCE_DIR}/Validator/Schema/hdtNifSchema.cpp"
	"${SOURCE_DIR}/Validator/Schema/hdtNifSchema.h"
	"${SOURCE_DIR}/Validator/Parser/hdtXSDSchemaParser.cpp"
	"${SOURCE_DIR}/Validator/Parser/hdtXSDSchemaParser.h"
	"${SOURCE_DIR}/Validator/Parser/hdtSCHSchemaParser.cpp"
	"${SOURCE_DIR}/Validator/Parser/hdtSCHSchemaParser.h"
	"${SOURCE_DIR}/Validator/Parser/hdtNIFBinaryParser.cpp"
	"${SOURCE_DIR}/Validator/Parser/hdtNIFBinaryParser.h"
	"${SOURCE_DIR}/Validator/Validators/hdtNIFValidator.h"
	"${SOURCE_DIR}/Validator/Validators/hdtNIFBoneRefValidator.cpp"
	"${SOURCE_DIR}/Validator/Validators/hdtNIFBoneRefValidator.h"
	"${SOURCE_DIR}/Validator/Validators/hdtXSDValidator.cpp"
	"${SOURCE_DIR}/Validator/Validators/hdtXSDValidator.h"
	"${SOURCE_DIR}/Validator/Validators/hdtSCHValidator.cpp"
	"${SOURCE_DIR}/Validator/Validators/hdtSCHValidator.h"
	"${SOURCE_DIR}/Validator/Validators/hdtNIFPhysicsXMLExtractor.cpp"
	"${SOURCE_DIR}/Validator/Validators/hdtNIFStructureValidator.cpp"
	"${SOURCE_DIR}/Validator/Improvers/hdtNIFBinaryIO.cpp"
	"${SOURCE_DIR}/Validator/Improvers/hdtNIFBinaryIO.h"
	"${SOURCE_DIR}/Validator/Improvers/hdtNIFOrphanedSkinImprover.cpp"
	"${SOURCE_DIR}/Validator/Improvers/hdtNIFOrphanedSkinImprover.h"
	"${SOURCE_DIR}/Validator/Improvers/hdtNIFSkinMeshValidator.cpp"
	"${SOURCE_DIR}/Validator/Improvers/hdtNIFSkinMeshValidator.h"
	"${SOURCE_DIR}/Validator/Utils/hdtConcurrencyUtils.h"
	"${SOURCE_DIR}/Validator/Utils/hdtPhysicsXmlSource.h"
	"${SOURCE_DIR}/Validator/Utils/hdtStringUtils.h"
	"${SOURCE_DIR}/Validator/Utils/hdtTemplateDefaults.cpp"
	"${SOURCE_DIR}/Validator/Utils/hdtTemplateDefaults.h"
	"${SOURCE_DIR}/Validator/Utils/hdtTimeUtils.h"
	"${SOURCE_DIR}/Validator/Utils/hdtXMLUtils.h"
	"${SOURCE_DIR}/Validator/Utils/hdtNIFBinaryUtils.h"
	"${SOURCE_DIR}/Validator/Utils/hdtNiflyShapeAudit.cpp"
	"${SOURCE_DIR}/Validator/Utils/hdtNiflyShapeAudit.h"
	"${SOURCE_DIR}/Validator/Utils/hdtValidatorFamily.h"
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
				"/W4" # Elevated warnings: unreachable code (C4702), unused vars (C4189), etc.
				"/wd4100" # Suppress: unreferenced formal parameter (pervasive in SKSE callbacks)
				"/wd4201" # Suppress: nameless struct/union (used throughout CommonLibSSE)
				"/WX" # Warnings as errors
				"/external:anglebrackets" # Treat angle-bracket includes as external
				"/external:W0" # No warnings from external headers (vcpkg, system)
				"$<$<CONFIG:DEBUG>:/analyze;/analyze:external->" # Static analysis in Debug only; skip external headers
				"$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:/Zc:inline;/JMC-;/Ob3>")

	target_link_options("${PROJECT_NAME}" PRIVATE "$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
						"$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>")
endif()

# The SKSE Menu Framework SDK header (extern/SKSEMenuFramework/include) is vendored pristine. hdtSMP64 builds without
# UNICODE, so its GetModuleHandle(L"...") resolves to GetModuleHandleA and rejects the wide literal. Patch it to the W
# variant at configure time --- like the bullet3 port's string(REPLACE) mods --- writing the patched copy into the build
# tree so the vendored file itself is never modified. FSMPMenu.cpp includes it.
set(SMF_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/SKSEMenuFramework")
set(SMF_HEADER_SRC "${ROOT_DIR}/extern/SKSEMenuFramework/include/SKSEMenuFramework.h")
# Re-run configure (regenerating the patched copy) whenever the vendored header changes, so a header update isn't
# silently ignored until someone manually reconfigures.
set_property(
	DIRECTORY
	APPEND
	PROPERTY CMAKE_CONFIGURE_DEPENDS "${SMF_HEADER_SRC}")
file(READ "${SMF_HEADER_SRC}" SMF_HEADER)
string(REPLACE "GetModuleHandle(L\"SKSEMenuFramework\")" "GetModuleHandleW(L\"SKSEMenuFramework\")" SMF_HEADER
			   "${SMF_HEADER}")
# Fail loudly if a future vendored-header update drops the string we patch, instead of silently shipping the unpatched
# wide-literal GetModuleHandle (which resolves to GetModuleHandleA in this non-UNICODE build and would make the
# framework's module lookup fail at runtime). Guards against silent drift.
string(FIND "${SMF_HEADER}" "GetModuleHandleW(L\"SKSEMenuFramework\")" SMF_PATCH_APPLIED)
if(SMF_PATCH_APPLIED EQUAL -1)
	message(
		FATAL_ERROR
			"SKSEMenuFramework.h: the GetModuleHandle(L\"SKSEMenuFramework\") string this build patches was not found "
			"(the vendored header changed). Update the string(REPLACE) patch in src/CMakeLists.txt.")
endif()
file(WRITE "${SMF_PATCHED_DIR}/SKSEMenuFramework.h" "${SMF_HEADER}")

target_include_directories(
	"${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/src" "${SOURCE_DIR}" ${DETOURS_INCLUDE_DIRS}
							  ${BULLET_INCLUDE_DIRS} ${RAPIDJSON_INCLUDE_DIRS} "${SMF_PATCHED_DIR}")

# The vendored SDK header trips two /W4 warnings of its own (C4099 struct/class tag mismatch on ImGuiTextFilter; C5054
# deprecated operator| between distinct ImGui enums). It is included only by FSMPMenu.cpp, so silence just those two
# there; the rest of our /W4 /WX build stays strict. /utf-8 makes MSVC read this file's narrow string literals (the
# native language names in the Language picker, e.g. "Français", "日本語") as UTF-8 both at compile and run time, matching
# what ImGui expects.
set_source_files_properties("${SOURCE_DIR}/UI/FSMPMenu.cpp" PROPERTIES COMPILE_OPTIONS "/wd4099;/wd5054;/utf-8")

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
find_package(pugixml CONFIG REQUIRED)
find_package(RapidJSON CONFIG REQUIRED) # configs.json (de)serialization in GlobalConfig.cpp
find_package(miniz CONFIG REQUIRED) # zip the bug-report bundle (log + report + crashlog) in the menu

# nifly (submodule: extern/nifly) — GPL-3.0 NIF library, Phase 0 of the nifly migration. It is built as its own separate
# static library, so our strict warning settings (/W4 /WX) do NOT apply to it — those are attached only to our own
# target. Linking against nifly (target_link_libraries below) automatically gives our code access to nifly's public
# headers (include/ + external/). nifly's own tests are skipped: it only adds them when it is the top-level CMake
# project, and here it is a subproject.
set(BUILD_TESTING OFF)
# nifly's include(CTest) declares an option named BUILD_TESTING. Under old CMake rules that declaration would ERASE the
# value we just set on the line above, and CMake prints a warning about it. Forcing the newer rule (policy CMP0077)
# makes the option respect the value we already set.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
add_subdirectory("${ROOT_DIR}/extern/nifly" "${CMAKE_CURRENT_BINARY_DIR}/extern/nifly" EXCLUDE_FROM_ALL)

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
			TBB::tbb
			pugixml::pugixml
			miniz::miniz
			nifly)

target_precompile_headers("${PROJECT_NAME}" PRIVATE "${SOURCE_DIR}/PCH.h")

install(
	FILES "$<TARGET_FILE:${PROJECT_NAME}>"
	DESTINATION "SKSE/Plugins"
	COMPONENT "main")

install(
	FILES "$<TARGET_PDB_FILE:${PROJECT_NAME}>"
	DESTINATION "/"
	COMPONENT "pdbs")

# Ship the default global config + in-game presets. These are deployed by `cmake --install` / CPack and by the CI
# package step (.github/workflows/build.yml). They are deliberately NOT added to the dev POST_BUILD copy below: a normal
# dev build only refreshes the DLL/PDB, so it never overwrites a configs.json the user has tuned in-game.
install(
	FILES "${ROOT_DIR}/configs/configs.json"
	DESTINATION "SKSE/Plugins/hdtSkinnedMeshConfigs"
	COMPONENT "main")
# The FSMP logo shown in the in-game menu's status header (loaded at runtime via SKSEMenuFramework::LoadTexture).
install(
	FILES "${ROOT_DIR}/configs/FSMP.dds"
	DESTINATION "SKSE/Plugins/hdtSkinnedMeshConfigs"
	COMPONENT "main")
install(
	DIRECTORY "${ROOT_DIR}/configs/configsPresets"
	DESTINATION "SKSE/Plugins/hdtSkinnedMeshConfigs"
	COMPONENT "main")
install(
	DIRECTORY "${ROOT_DIR}/configs/localization"
	DESTINATION "SKSE/Plugins/hdtSkinnedMeshConfigs"
	COMPONENT "main")
# The config menu's monospace font for its Output / log panels. It installs into the SKSE Menu Framework's own font
# folder (Data/SKSE/Plugins/Fonts) --- NOT hdtSkinnedMeshConfigs --- because the framework is what scans that folder and
# bakes the face; the menu then selects it by name via SKSEMenuFramework::PushFont.
install(
	FILES "${ROOT_DIR}/configs/fonts/FSMPMono.ttf" "${ROOT_DIR}/configs/fonts/FSMPMono.LICENSE.txt"
	DESTINATION "SKSE/Plugins/Fonts"
	COMPONENT "main")

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
