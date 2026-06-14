# smp_replay - standalone SMP physics replay & benchmark harness + its doctest unit suite.
#
# Built only when BUILD_BENCHMARK is ON (a dev/CI tool, never packaged - PLAN §17). It compiles the
# shared physics core (src/hdtSkinnedMesh/*) against the RE shim instead of CommonLibSSE, so it links
# Bullet + TBB only and never touches the game's CommonLibSSE - no ODR clash (D2/§10).

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SOURCE_DIR "${ROOT_DIR}/src")

find_package(Bullet CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)

# The shared physics core - compiled here against the shim, unchanged.
set(CORE_SOURCES
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtBoneScaleConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtCollider.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtCollisionAlgorithm.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtConeTwistConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtDispatcher.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtGeneric6DofConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshAlgorithm.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshBody.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshBone.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshShape.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshSystem.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtSkinnedMeshWorld.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtStiffSpringConstraint.cpp"
	"${SOURCE_DIR}/hdtSkinnedMesh/hdtVertex.cpp")

set(HARNESS_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/replayRunner.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/replayRunner.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/hdtReplaySystem.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/hdtReplaySystem.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/replayProfiler.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/replayProfiler.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/shim/ShimPCH.h"
	"${SOURCE_DIR}/Replay/hdtReplayFormat.h"
	"${SOURCE_DIR}/Replay/hdtReplayConvert.h"
	# The capture-side snapshot builder operates purely on core types, so it compiles against the
	# shim too; including it lets the harness self-test the capture<->replay round-trip.
	"${SOURCE_DIR}/Replay/hdtReplayCapture.cpp"
	"${SOURCE_DIR}/Replay/hdtReplayCapture.h")

set(TEST_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/tests/test_format.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/tests/test_apply_kinematic.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/tests/test_replay_pipeline.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/tests/test_capture_snapshot.cpp")

add_executable(smp_replay ${HARNESS_SOURCES} ${TEST_SOURCES} ${CORE_SOURCES})

target_compile_features(smp_replay PRIVATE cxx_std_20)

target_include_directories(smp_replay PRIVATE
	"${SOURCE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	${BULLET_INCLUDE_DIRS})

target_compile_definitions(smp_replay PRIVATE BT_USE_SSE_IN_API BT_ENABLE_PROFILE NOMINMAX)

# Force-include the shim in front of every TU (the core .cpp files do not #include a PCH; the game
# build force-includes src/PCH.h, the harness uses the shim instead).
target_precompile_headers(smp_replay PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/shim/ShimPCH.h")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_options(smp_replay PRIVATE "/EHsc" "/permissive-" "/bigobj" "/utf-8")
endif()

target_link_libraries(smp_replay PRIVATE ${BULLET_LIBRARIES} TBB::tbb doctest::doctest)

# CTest entry: the harness self-test (format round-trip + trust-boundary, the applyKinematicTarget
# seam, and the end-to-end replay pipeline). CI runs this.
enable_testing()
add_test(NAME smp_replay_selftest COMMAND smp_replay --selftest)

# Seam-parity gate (D8): only runs when a committed fixture exists (the binary capture must be
# produced in-game first - see fixtures/README.md). The harness exits non-zero on a golden mismatch.
set(PARITY_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/parity.bin")
if(EXISTS "${PARITY_FIXTURE}")
	add_test(NAME smp_replay_parity COMMAND smp_replay "${PARITY_FIXTURE}" --threads 1 --check-golden)
endif()

# Deliver the tool next to hdtSMP64.dll, in the same SKSE/Plugins folder, on every build - so a normal
# mod build (and the release package built from that folder) carries smp_replay.exe with the DLL.
# COPY_OUTPUT and CompiledPluginsPath are inherited from the top-level CMakeLists. make_directory keeps
# this safe even if the tool happens to build before the DLL has created the folder.
if(COPY_OUTPUT)
	add_custom_command(
		TARGET smp_replay
		POST_BUILD
		COMMAND "${CMAKE_COMMAND}" -E make_directory "${CompiledPluginsPath}/SKSE/Plugins"
		COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:smp_replay>"
				"${CompiledPluginsPath}/SKSE/Plugins/"
		VERBATIM)
endif()
