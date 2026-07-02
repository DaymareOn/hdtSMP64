# config_tests - doctest unit suite for the GlobalConfig JSON (de)serialization.
#
# Built only when BUILD_CONFIG_TESTS is ON (a dev/CI tool, never packaged). GlobalConfig is pure rapidjson + STL, so
# this target links only rapidjson + doctest - no CommonLibSSE, Bullet, or TBB - and it compiles the engine source
# directly without the game's force-included src/PCH.h. Mirrors tests/patterns.

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SOURCE_DIR "${ROOT_DIR}/src")

find_package(RapidJSON CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)

add_executable(config_tests "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/test_globalconfig.cpp"
							"${SOURCE_DIR}/GlobalConfig.cpp" "${SOURCE_DIR}/GlobalConfig.h")

target_compile_features(config_tests PRIVATE cxx_std_20)
target_include_directories(config_tests PRIVATE "${SOURCE_DIR}" ${RAPIDJSON_INCLUDE_DIRS})

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(config_tests PRIVATE "/EHsc" "/permissive-" "/bigobj" "/utf-8")
endif()

target_link_libraries(config_tests PRIVATE doctest::doctest)

# CTest entry: running the exe with no args runs the whole doctest suite and exits non-zero on failure.
add_test(NAME config_tests COMMAND config_tests)
