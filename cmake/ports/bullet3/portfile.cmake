if(VCPKG_TARGET_IS_WINDOWS)
	vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

vcpkg_from_github(
	OUT_SOURCE_PATH
	SOURCE_PATH
	REPO
	bulletphysics/bullet3
	REF
	"${VERSION}"
	SHA512
	7086e5fcf69635801bb311261173cb8d173b712ca1bd78be03df48fad884674e85512861190e45a1a62d5627aaad65cde08c175c44a3be9afa410d3dfd5358d4
	HEAD_REF
	master
	PATCHES
	cmake-version.diff
	cmake-config-export.diff
	opencl.diff
	tinyxml2.diff)

# Apply non-Hookean spring modifications
file(READ "${SOURCE_PATH}/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h" HEADER_CONTENT)
string(
	REPLACE
		"btScalar m_currentPosition;\n\tint m_currentLimit;"
		"btScalar m_currentPosition;\n\tint m_currentLimit;\n\n\t// BM: Definitely a better way to do this, but yay dirty hacks for Skyrims.\n\tbtScalar m_nonHookeanDamping;\n\tbtScalar m_nonHookeanStiffness;"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
string(
	REPLACE
		"m_currentPosition = 0;\n\t\tm_currentLimit = 0;"
		"m_currentPosition = 0;\n\t\tm_currentLimit = 0;\n\n\t\tm_nonHookeanDamping = 0.f;\n\t\tm_nonHookeanStiffness = 0.f;"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
string(
	REPLACE
		"m_currentPosition = limot.m_currentPosition;\n\t\tm_currentLimit = limot.m_currentLimit;"
		"m_currentPosition = limot.m_currentPosition;\n\t\tm_currentLimit = limot.m_currentLimit;\n\n\t\tm_nonHookeanDamping = limot.m_nonHookeanDamping;\n\t\tm_nonHookeanStiffness = limot.m_nonHookeanStiffness;"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
string(
	REPLACE
		"btVector3 m_currentLinearDiff;\n\tint m_currentLimit[3];"
		"btVector3 m_currentLinearDiff;\n\tint m_currentLimit[3];\n\n\t// BM: Definitely a better way to do this, but yay dirty hacks for Skyrims.\n\tbtVector3 m_nonHookeanDamping;\n\tbtVector3 m_nonHookeanStiffness;"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
string(
	REPLACE
		"m_currentLimit[i] = 0;"
		"m_currentLimit[i] = 0;\n\n\t\t\tm_nonHookeanDamping[i] = btScalar(0.f);\n\t\t\tm_nonHookeanStiffness[i] = btScalar(0.f);"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
string(
	REPLACE
		"m_currentLimit[i] = other.m_currentLimit[i];"
		"m_currentLimit[i] = other.m_currentLimit[i];\n\n\t\t\tm_nonHookeanDamping[i] = other.m_nonHookeanDamping[i];\n\t\t\tm_nonHookeanStiffness[i] = other.m_nonHookeanStiffness[i];"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
string(
	REPLACE
		"void setStiffness(int index, btScalar stiffness, bool limitIfNeeded = true);  // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely\n\tvoid setDamping"
		"void setStiffness(int index, btScalar stiffness, bool limitIfNeeded = true);  // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely\n\tvoid setNonHookeanDamping(int index, btScalar factor);                        // Increases spring damping by factor (using a power of 2 curve), the closer the current position is to equilibrium.\n\tvoid setNonHookeanStiffness(int index, btScalar factor);                      // Increases spring stiffness the further position is from equilibrium, by this factor (power curve).\n\tvoid setDamping"
		HEADER_CONTENT
		"${HEADER_CONTENT}")
file(WRITE "${SOURCE_PATH}/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h" "${HEADER_CONTENT}")

file(READ "${SOURCE_PATH}/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp" CPP_CONTENT)
string(
	REPLACE
		"limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;\n\n\t\t\t//rotAllowed"
		"limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;\n\n\t\t\tlimot.m_nonHookeanDamping = m_linearLimits.m_nonHookeanDamping[i];\n\t\t\tlimot.m_nonHookeanStiffness = m_linearLimits.m_nonHookeanStiffness[i];\n\n\t\t\t//rotAllowed"
		CPP_CONTENT
		"${CPP_CONTENT}")
string(
	REPLACE
		"vel = (linVelA + tanVelA).dot(ax1) - (linVelB + tanVelB).dot(ax1);\n\t\t}\n\n\t\tbtScalar cfm = BT_ZERO;"
		"vel = (linVelA + tanVelA).dot(ax1) - (linVelB + tanVelB).dot(ax1);\n\t\t}\n\n\t\tbtScalar dampingFactor = limot->m_nonHookeanDamping;\n\t\tbtScalar stiffnessFactor = limot->m_nonHookeanStiffness;\n\t\tif (!btFuzzyZero(error) && (!btFuzzyZero(dampingFactor) || !btFuzzyZero(stiffnessFactor)))\n\t\t{\n\t\t\tbtScalar range = pos < ep ? ep - limot->m_loLimit : limot->m_hiLimit - ep;\n\t\t\tbtScalar rf = !btFuzzyZero(range) ? btFabs(error) / range : BT_ZERO;\n\t\t\t// Avoid blowing shit up.\n\t\t\tbtScalar t = btClamped(rf, BT_ZERO, BT_ONE);\n\n\t\t\tkd *= BT_ONE - (dampingFactor * t);\n\t\t\tks *= BT_ONE - (stiffnessFactor * t);\n\t\t}\n\n\t\tbtScalar cfm = BT_ZERO;"
		CPP_CONTENT
		"${CPP_CONTENT}")
string(
	REPLACE
		"}\n}\n\nvoid btGeneric6DofSpring2Constraint::setDamping(int index, btScalar damping, bool limitIfNeeded)"
		"}\n}\n\n// Elastic, uneven damping used to fake non-Hookean springs for a specific axis.\n// Default: 0 (disabled).  Other values below 1 will likely blow your spring up.. individual speed and hilarity may vary.\nvoid btGeneric6DofSpring2Constraint::setNonHookeanDamping(int index, btScalar damping)\n{\n\tbtAssert((index >= 0) && (index < 6));\n\tif (index < 3)\n\t{\n\t\tm_linearLimits.m_nonHookeanDamping[index] = damping;\n\t}\n\telse\n\t{\n\t\tm_angularLimits[index - 3].m_nonHookeanDamping = damping;\n\t}\n}\n\nvoid btGeneric6DofSpring2Constraint::setNonHookeanStiffness(int index, btScalar damping)\n{\n\tbtAssert((index >= 0) && (index < 6));\n\tif (index < 3)\n\t{\n\t\tm_linearLimits.m_nonHookeanStiffness[index] = damping;\n\t}\n\telse\n\t{\n\t\tm_angularLimits[index - 3].m_nonHookeanStiffness = damping;\n\t}\n}\n\nvoid btGeneric6DofSpring2Constraint::setDamping(int index, btScalar damping, bool limitIfNeeded)"
		CPP_CONTENT
		"${CPP_CONTENT}")
file(WRITE "${SOURCE_PATH}/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp" "${CPP_CONTENT}")

# oneTBB (TBB >= 2021) removed task_scheduler_init.  Port btThreads.cpp to the oneTBB API: use tbb::global_control for
# thread-count limiting and tbb::info::default_concurrency() instead of the removed default_num_threads().
file(READ "${SOURCE_PATH}/src/LinearMath/btThreads.cpp" THREADS_CPP)
string(REPLACE "#include <tbb/task_scheduler_init.h>\n" "#include <tbb/global_control.h>\n#include <tbb/info.h>\n"
			   THREADS_CPP "${THREADS_CPP}")
string(REPLACE "tbb::task_scheduler_init* m_tbbSchedulerInit;" "tbb::global_control* m_tbbSchedulerInit;" THREADS_CPP
			   "${THREADS_CPP}")
string(REPLACE "return tbb::task_scheduler_init::default_num_threads();" "return tbb::info::default_concurrency();"
			   THREADS_CPP "${THREADS_CPP}")
string(
	REPLACE "m_tbbSchedulerInit = new tbb::task_scheduler_init(m_numThreads);"
			"m_tbbSchedulerInit = new tbb::global_control(tbb::global_control::max_allowed_parallelism, m_numThreads);"
			THREADS_CPP "${THREADS_CPP}")
file(WRITE "${SOURCE_PATH}/src/LinearMath/btThreads.cpp" "${THREADS_CPP}")

# Runtime profiler gate. Bullet still compiles the profiler support, but disabled BT_PROFILE scopes become a cheap
# inline bool check instead of two indirect calls into empty profile callbacks.
file(READ "${SOURCE_PATH}/src/LinearMath/btQuickprof.h" QUICKPROF_HEADER)
string(
	REPLACE
		"void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc);\n\n#ifndef BT_ENABLE_PROFILE"
		"void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc);\n\nvoid btEnterProfileZone(const char* name);\nvoid btLeaveProfileZone();\n\nextern bool gBtProfileEnabled;\nvoid btSetProfileEnabled(bool enabled);\nbool btGetProfileEnabled();\nSIMD_FORCE_INLINE bool btIsProfileEnabled()\n{\n\treturn gBtProfileEnabled;\n}\n\n#ifndef BT_ENABLE_PROFILE"
		QUICKPROF_HEADER
		"${QUICKPROF_HEADER}")
string(
	REPLACE
		"void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc);\n\nextern bool gBtProfileEnabled;"
		"void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc);\n\nvoid btEnterProfileZone(const char* name);\nvoid btLeaveProfileZone();\n\nextern bool gBtProfileEnabled;"
		QUICKPROF_HEADER
		"${QUICKPROF_HEADER}")
string(
	REPLACE
		"class CProfileSample\n{\npublic:\n\tCProfileSample(const char* name);\n\n\t~CProfileSample(void);\n};"
		"class CProfileSample\n{\npublic:\n\tCProfileSample(const char* name) :\n\t\tm_enabled(btIsProfileEnabled())\n\t{\n\t\tif (m_enabled)\n\t\t{\n\t\t\tbtEnterProfileZone(name);\n\t\t}\n\t}\n\n\t~CProfileSample(void)\n\t{\n\t\tif (m_enabled)\n\t\t{\n\t\t\tbtLeaveProfileZone();\n\t\t}\n\t}\n\nprivate:\n\tbool m_enabled;\n};"
		QUICKPROF_HEADER
		"${QUICKPROF_HEADER}")
file(WRITE "${SOURCE_PATH}/src/LinearMath/btQuickprof.h" "${QUICKPROF_HEADER}")

file(READ "${SOURCE_PATH}/src/LinearMath/btQuickprof.cpp" QUICKPROF_CPP)
string(
	REPLACE
		"static btEnterProfileZoneFunc* bts_enterFunc = btEnterProfileZoneDefault;\nstatic btLeaveProfileZoneFunc* bts_leaveFunc = btLeaveProfileZoneDefault;"
		"bool gBtProfileEnabled = false;\n\nstatic btEnterProfileZoneFunc* bts_enterFunc = btEnterProfileZoneDefault;\nstatic btLeaveProfileZoneFunc* bts_leaveFunc = btLeaveProfileZoneDefault;"
		QUICKPROF_CPP
		"${QUICKPROF_CPP}")
string(
	REPLACE
		"void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc)\n{\n\tbts_leaveFunc = leaveFunc;\n}\n\nCProfileSample::CProfileSample(const char* name)\n{\n\tbtEnterProfileZone(name);\n}\n\nCProfileSample::~CProfileSample(void)\n{\n\tbtLeaveProfileZone();\n}\n"
		"void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc)\n{\n\tbts_leaveFunc = leaveFunc;\n}\n\nvoid btSetProfileEnabled(bool enabled)\n{\n\tgBtProfileEnabled = enabled;\n}\n\nbool btGetProfileEnabled()\n{\n\treturn gBtProfileEnabled;\n}\n"
		QUICKPROF_CPP
		"${QUICKPROF_CPP}")
file(WRITE "${SOURCE_PATH}/src/LinearMath/btQuickprof.cpp" "${QUICKPROF_CPP}")

file(REMOVE_RECURSE "${SOURCE_PATH}/examples/ThirdPartyLibs")

vcpkg_check_features(
	OUT_FEATURE_OPTIONS
	FEATURE_OPTIONS
	FEATURES
	multithreading
	BULLET2_MULTITHREADING
	double-precision
	USE_DOUBLE_PRECISION
	extras
	BUILD_EXTRAS
	opencl
	BUILD_OPENCL
	INVERTED_FEATURES
	rtti
	USE_MSVC_DISABLE_RTTI)

set(_bullet_tbb_options_release "")
set(_bullet_tbb_options_debug "")
if("multithreading" IN_LIST FEATURES)
	# Use TBB (from vcpkg) instead of PPL so Bullet's internal task scheduler shares the same threading backend as the
	# hdtSMP64 plugin. vcpkg's oneTBB ships as tbb12(_debug).lib; bullet's CMake does a fixed find_library(TBB_LIBRARY
	# tbb ...), so we pre-seed the cache vars to short-circuit the find and point at the right file per config.
	list(APPEND FEATURE_OPTIONS -DBULLET2_USE_TBB_MULTITHREADING=ON
		 "-DBULLET2_TBB_INCLUDE_DIR=${CURRENT_INSTALLED_DIR}/include")
	list(APPEND _bullet_tbb_options_release "-DBULLET2_TBB_LIB_DIR=${CURRENT_INSTALLED_DIR}/lib"
		 "-DTBB_LIBRARY=${CURRENT_INSTALLED_DIR}/lib/tbb12.lib"
		 "-DTBBMALLOC_LIBRARY=${CURRENT_INSTALLED_DIR}/lib/tbbmalloc.lib")
	list(APPEND _bullet_tbb_options_debug "-DBULLET2_TBB_LIB_DIR=${CURRENT_INSTALLED_DIR}/debug/lib"
		 "-DTBB_LIBRARY=${CURRENT_INSTALLED_DIR}/debug/lib/tbb12_debug.lib"
		 "-DTBBMALLOC_LIBRARY=${CURRENT_INSTALLED_DIR}/debug/lib/tbbmalloc_debug.lib")
endif()

string(COMPARE EQUAL "${VCPKG_CRT_LINKAGE}" "dynamic" USE_MSVC_RUNTIME_LIBRARY_DLL)

# Match the project's BT_USE_SSE_IN_API define so btVector3 layout is consistent
string(APPEND VCPKG_CXX_FLAGS " /DBT_USE_SSE_IN_API")
string(APPEND VCPKG_C_FLAGS " /DBT_USE_SSE_IN_API")

# Build Bullet's profiler symbols into every configuration. Runtime profiling stays disabled by default through the
# btQuickprof gate above, so normal builds only pay the inline bool check in BT_PROFILE scopes.
string(APPEND VCPKG_CXX_FLAGS " /DBT_ENABLE_PROFILE")
string(APPEND VCPKG_C_FLAGS " /DBT_ENABLE_PROFILE")

vcpkg_cmake_configure(
	SOURCE_PATH
	"${SOURCE_PATH}"
	OPTIONS
	-DUSE_MSVC_RUNTIME_LIBRARY_DLL=${USE_MSVC_RUNTIME_LIBRARY_DLL}
	-DBUILD_CPU_DEMOS=OFF
	-DBUILD_BULLET2_DEMOS=OFF
	-DBUILD_OPENGL3_DEMOS=OFF
	-DBUILD_BULLET3=ON
	-DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF
	-DBUILD_BULLET_ROBOTICS_EXTRA=OFF
	-DBUILD_GIMPACTUTILS_EXTRA=OFF
	-DBUILD_HACD_EXTRA=OFF
	-DBUILD_OBJ2SDF_EXTRA=OFF
	-DBUILD_UNIT_TESTS=OFF
	-DINSTALL_LIBS=ON
	${FEATURE_OPTIONS}
	OPTIONS_RELEASE
	${_bullet_tbb_options_release}
	OPTIONS_DEBUG
	${_bullet_tbb_options_debug}
	MAYBE_UNUSED_VARIABLES
	BUILD_BULLET_ROBOTICS_EXTRA
	BUILD_BULLET_ROBOTICS_GUI_EXTRA
	BUILD_GIMPACTUTILS_EXTRA
	BUILD_HACD_EXTRA
	BUILD_OBJ2SDF_EXTRA
	USE_MSVC_DISABLE_RTTI
	BULLET2_TBB_LIB_DIR)

vcpkg_cmake_install()
vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/bullet)
vcpkg_fixup_pkgconfig()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/include/bullet/BulletInverseDynamics/details") # empty

file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE.txt")
