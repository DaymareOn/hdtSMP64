# xml_pattern_tests - doctest unit suite for the SMP XML pattern expander (PLAN §10, decision D5).
#
# Built only when BUILD_PATTERN_TESTS is ON (a dev/CI tool, never packaged). The expander is pure
# pugixml + STL, so this target links only pugixml + doctest - no CommonLibSSE, Bullet, or TBB - and it
# compiles the engine source directly without the game's force-included src/PCH.h.

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SOURCE_DIR "${ROOT_DIR}/src")

find_package(pugixml CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)

add_executable(
	xml_pattern_tests
	"${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/test_expander.cpp"
	"${SOURCE_DIR}/hdtXmlPatternExpander.cpp"
	"${SOURCE_DIR}/hdtXmlPatternExpander.h")

target_compile_features(xml_pattern_tests PRIVATE cxx_std_20)
target_include_directories(xml_pattern_tests PRIVATE "${SOURCE_DIR}")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(xml_pattern_tests PRIVATE "/EHsc" "/permissive-" "/bigobj" "/utf-8")
endif()

target_link_libraries(xml_pattern_tests PRIVATE pugixml::pugixml doctest::doctest)

# CTest entry: running the exe with no args runs the whole doctest suite and exits non-zero on failure.
enable_testing()
add_test(NAME xml_pattern_tests COMMAND xml_pattern_tests)
