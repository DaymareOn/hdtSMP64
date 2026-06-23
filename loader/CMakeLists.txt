# Faster HDT-SMP loader stub.
#
# A tiny SKSE plugin compiled WITHOUT any /arch:AVX* flag, so it executes on every x64 CPU. It is
# the only DLL SKSE auto-loads; at runtime it detects the CPU and loads the best optimized variant
# from SKSE/Plugins/FSMP/. It is therefore only meaningful - and only safe - in the no-AVX configure:
# that configure carries no /arch in the global flags, which also guarantees the CommonLibSSE it
# links was built without AVX. In the AVX/AVX2/AVX512 configures we skip it entirely so we never emit
# an AVX-tainted "runs everywhere" loader.
if(NOT AVX_VARIANT STREQUAL "noavx")
	return()
endif()

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(LOADER_NAME hdtSMP64_loader)

# The loader reuses the same version template as the physics plugin so its SKSE version data (name,
# version, compatible runtimes) is guaranteed to match. AVX_VARIANT here is "noavx".
set(LOADER_VERSION_HEADER "${CMAKE_CURRENT_BINARY_DIR}/Plugin.h")
configure_file("${ROOT_DIR}/cmake/Plugin.h.in" "${LOADER_VERSION_HEADER}" @ONLY)
configure_file("${ROOT_DIR}/cmake/version.rc.in" "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)

set(LOADER_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/src/AvxVariant.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/CpuFeatures.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/CpuFeatures.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/CrashState.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/CrashState.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/VariantSelector.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/VariantSelector.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

add_library(
	"${LOADER_NAME}" SHARED
	${LOADER_SOURCES}
	"${LOADER_VERSION_HEADER}"
	"${CMAKE_CURRENT_BINARY_DIR}/version.rc")

# Output is hdtSMP64.dll - the exact name SKSE auto-loads, replacing the old single-variant DLL.
set_target_properties("${LOADER_NAME}" PROPERTIES OUTPUT_NAME "hdtSMP64")

target_compile_features("${LOADER_NAME}" PRIVATE cxx_std_20)
target_compile_definitions("${LOADER_NAME}" PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_include_directories("${LOADER_NAME}" PRIVATE "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src")

# CommonLibSSE is defined while processing src/ (added before this subdirectory); its alias target is
# global, so it is visible here. spdlog, however, is an IMPORTED target whose visibility is limited to
# the directory that called find_package (src/) and below - so we must find it again in this scope.
# The loader uses CommonLibSSE only for the SKSE plugin types/constants and logging; it does NOT call
# SKSE::Init (all real init happens inside the variant it forwards to). spdlog is linked explicitly
# because the loader uses the file sink directly, not only through CommonLibSSE's logger helpers.
find_package(spdlog CONFIG REQUIRED)
target_link_libraries("${LOADER_NAME}" PRIVATE CommonLibSSE::CommonLibSSE spdlog::spdlog)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(
		"${LOADER_NAME}"
		PRIVATE "/sdl"
				"/utf-8"
				"/Zi"
				"/permissive-"
				"/Zc:preprocessor"
				"/bigobj"
				"/W4"
				"/wd4100" # unreferenced formal parameter (pervasive in SKSE callbacks)
				"/WX"
				"/external:anglebrackets"
				"/external:W0"
				"$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:/Zc:inline>")
	target_link_options(
		"${LOADER_NAME}" PRIVATE
		"$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>")
endif()

install(
	FILES "$<TARGET_FILE:${LOADER_NAME}>"
	DESTINATION "SKSE/Plugins"
	COMPONENT "main")
install(
	FILES "${CMAKE_CURRENT_SOURCE_DIR}/hdtSMP64_loader.ini"
	DESTINATION "SKSE/Plugins"
	COMPONENT "main")
install(
	FILES "$<TARGET_PDB_FILE:${LOADER_NAME}>"
	DESTINATION "/"
	COMPONENT "pdbs")

# Local dev convenience: drop the loader DLL and the default INI next to the game's plugins so a
# from-source build is immediately runnable. The variant target performs the matching copy into the
# FSMP subfolder.
if("${COPY_OUTPUT}")
	add_custom_command(
		TARGET "${LOADER_NAME}"
		POST_BUILD
		COMMAND "${CMAKE_COMMAND}" -E make_directory "${CompiledPluginsPath}/SKSE/Plugins"
		COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:${LOADER_NAME}>"
				"${CompiledPluginsPath}/SKSE/Plugins/"
		COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/hdtSMP64_loader.ini"
				"${CompiledPluginsPath}/SKSE/Plugins/"
		VERBATIM)
endif()
