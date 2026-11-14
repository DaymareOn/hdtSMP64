# owner_tests - doctest unit suite for the external mesh owner registry.
#
# Built only when BUILD_OWNER_TESTS is ON (a dev/CI tool, never packaged). ExternalOwner is pure STL plus the
# dependency-light PluginOwnerAPI.h, so this target links only doctest - no CommonLibSSE, Bullet, or TBB - and it
# compiles the engine source directly without the game's force-included src/PCH.h. Mirrors tests/config.

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SOURCE_DIR "${ROOT_DIR}/src")

find_package(doctest CONFIG REQUIRED)

add_executable(
	owner_tests "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/test_owner.cpp"
				"${SOURCE_DIR}/ExternalOwner.cpp" "${SOURCE_DIR}/ExternalOwner.h" "${SOURCE_DIR}/PluginOwnerAPI.h")

target_compile_features(owner_tests PRIVATE cxx_std_20)
target_include_directories(owner_tests PRIVATE "${SOURCE_DIR}")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(owner_tests PRIVATE "/EHsc" "/permissive-" "/bigobj" "/utf-8")
endif()

target_link_libraries(owner_tests PRIVATE doctest::doctest)

# CTest entry: running the exe with no args runs the whole doctest suite and exits non-zero on failure.
add_test(NAME owner_tests COMMAND owner_tests)
