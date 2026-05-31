include("${CMAKE_CURRENT_LIST_DIR}/_pin_toolset.cmake")

string(APPEND VCPKG_CXX_FLAGS " /arch:AVX512")
string(APPEND VCPKG_C_FLAGS " /arch:AVX512")
