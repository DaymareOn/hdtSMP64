# validator_tests - doctest unit suite for the XML reader stream contract (#404) and the validator's inert-<bone>
# (#402) and skin-redundant-<bone> (#406) warnings.
#
# Built only when BUILD_VALIDATOR_TESTS is ON (a dev/CI tool, never packaged). The targets under test are
# game-independent: XmlReader.cpp needs only the XmlInspector headers and Bullet's header-inline math types; the
# XML-redundancy analysis (hdtTemplateDefaults.cpp) is pure pugixml + STL; and the #406 offline skin-bone extraction
# (hdtNIFBinaryParser.cpp + hdtNifSchema.cpp) is pure binary/schema parsing - no CommonLibSSE, no TBB - so all compile
# directly without the game's force-included src/PCH.h (tests supply test_pch.h instead). Mirrors tests/patterns.

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SOURCE_DIR "${ROOT_DIR}/src")

find_package(Bullet CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)

add_executable(
	validator_tests
	"${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/test_xmlreader.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/test_inert_bones.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/test_skin_redundant_bones.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/test_skin_bone_extraction.cpp" "${SOURCE_DIR}/XmlReader.cpp"
	"${SOURCE_DIR}/Validator/Utils/hdtTemplateDefaults.cpp"
	"${SOURCE_DIR}/Validator/Parser/hdtNIFBinaryParser.cpp"
	"${SOURCE_DIR}/Validator/Schema/hdtNifSchema.cpp")

target_compile_features(validator_tests PRIVATE cxx_std_20)
target_include_directories(validator_tests PRIVATE "${SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" ${BULLET_INCLUDE_DIRS})

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(validator_tests PRIVATE "/EHsc" "/permissive-" "/bigobj" "/utf-8")
endif()

# XmlReader.cpp relies on headers the plugin's PCH normally provides; give the standalone compile the same minimal
# platform/std surface.
target_precompile_headers(validator_tests PRIVATE test_pch.h)

target_link_libraries(validator_tests PRIVATE pugixml::pugixml doctest::doctest ${BULLET_LIBRARIES})

# CTest entry: running the exe with no args runs the whole doctest suite and exits non-zero on failure.
add_test(NAME validator_tests COMMAND validator_tests)
