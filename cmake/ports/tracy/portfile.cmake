vcpkg_from_github(
	OUT_SOURCE_PATH
	SOURCE_PATH
	REPO
	wolfpld/tracy
	REF
	f2a4f2529e7e71844c9e434b877770a21ecf7587
	SHA512
	b1c93209f19bc4f6c98606ed5c69447b299917e548f8560526c069c4689b5562a88e68abaf9fcc9fde492c95a0c60a7909c544475ca7b78f2fff63c0dad13f26
	HEAD_REF
	master)

vcpkg_check_features(
	OUT_FEATURE_OPTIONS
	FEATURE_OPTIONS
	FEATURES
	on-demand
	TRACY_ON_DEMAND
	fibers
	TRACY_FIBERS
	verbose
	TRACY_VERBOSE
	INVERTED_FEATURES
	crash-handler
	TRACY_NO_CRASH_HANDLER)

set(TOOLS_OPTIONS "")
if("cli-tools" IN_LIST FEATURES OR "gui-tools" IN_LIST FEATURES)
	vcpkg_find_acquire_program(PKGCONFIG)
	list(APPEND TOOLS_OPTIONS "-DPKG_CONFIG_EXECUTABLE=${PKGCONFIG}")
endif()

vcpkg_cmake_configure(
	SOURCE_PATH
	${SOURCE_PATH}
	OPTIONS
	-DDOWNLOAD_CAPSTONE=OFF
	-DLEGACY=ON
	-DTRACY_ENABLE=ON
	${FEATURE_OPTIONS}
	OPTIONS_RELEASE
	${TOOLS_OPTIONS}
	MAYBE_UNUSED_VARIABLES
	DOWNLOAD_CAPSTONE
	LEGACY)
vcpkg_cmake_install()

vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(PACKAGE_NAME Tracy CONFIG_PATH lib/cmake/Tracy)

# Public headers reach ../common and ../client by relative path; forward via thin <Tracy/*.hpp> wrappers instead of
# copying so those relative includes keep resolving.
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/include/Tracy")
foreach(header IN ITEMS Tracy.hpp TracyC.h TracyD3D11.hpp)
	file(WRITE "${CURRENT_PACKAGES_DIR}/include/Tracy/${header}"
		 "#pragma once\n#include \"../tracy/tracy/${header}\"\n")
endforeach()

function(tracy_copy_tool tool_name tool_dir)
	vcpkg_copy_tools(TOOL_NAMES "${tool_name}" SEARCH_DIR "${CURRENT_BUILDTREES_DIR}/${TARGET_TRIPLET}-rel/${tool_dir}")
endfunction()

if("cli-tools" IN_LIST FEATURES)
	tracy_copy_tool(tracy-capture capture)
	tracy_copy_tool(tracy-csvexport csvexport)
	tracy_copy_tool(tracy-import-chrome import)
	tracy_copy_tool(tracy-import-fuchsia import)
	tracy_copy_tool(tracy-update update)
endif()
if("gui-tools" IN_LIST FEATURES)
	tracy_copy_tool(tracy-profiler profiler)
endif()

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
