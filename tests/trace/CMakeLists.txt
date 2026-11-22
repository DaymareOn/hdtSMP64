set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SOURCE_DIR "${ROOT_DIR}/src")

find_package(Bullet CONFIG REQUIRED)
find_package(RapidJSON CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)

# Compiled from a private copy: from src/hdtSkinnedMesh its quoted includes would find the real constraint and bone
# headers (which need CommonLibSSE) before the doubles.
set(RECORDER_DIR "${CMAKE_CURRENT_BINARY_DIR}/recorder")
configure_file("${SOURCE_DIR}/hdtSkinnedMesh/hdtTraceRecorder.cpp" "${RECORDER_DIR}/hdtTraceRecorder.cpp" COPYONLY)
configure_file("${SOURCE_DIR}/hdtSkinnedMesh/hdtTraceRecorder.h" "${RECORDER_DIR}/hdtTraceRecorder.h" COPYONLY)

add_executable(trace_tests "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/test_trace.cpp"
							"${RECORDER_DIR}/hdtTraceRecorder.cpp")

target_compile_features(trace_tests PRIVATE cxx_std_20)
target_include_directories(trace_tests PRIVATE "${RECORDER_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/stubs"
											   "${CMAKE_CURRENT_SOURCE_DIR}" ${BULLET_INCLUDE_DIRS} ${RAPIDJSON_INCLUDE_DIRS})

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(trace_tests PRIVATE "/EHsc" "/permissive-" "/bigobj" "/utf-8" "/FI${CMAKE_CURRENT_SOURCE_DIR}/stubs/test_pch.h")
endif()

target_link_libraries(trace_tests PRIVATE doctest::doctest ${BULLET_LIBRARIES})

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cwd")
add_test(NAME trace_tests COMMAND trace_tests WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cwd")
