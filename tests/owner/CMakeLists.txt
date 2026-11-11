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

add_test(NAME owner_tests COMMAND owner_tests)
