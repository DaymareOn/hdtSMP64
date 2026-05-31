set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

# Pin the MSVC toolset that vcpkg uses to build dependencies.
#
# vcpkg selects its own compiler for dependencies independently of the CMake preset toolset, and by default picks the
# NEWEST installed Visual Studio. On a machine with VS2026 (MSVC 14.50) alongside VS2022 (14.44), that means the
# dependencies (Bullet, TBB, fmt, ...) get built with 14.50 while the plugin and the CI build use 14.44 -- a real
# codegen mismatch that destabilised the physics. Pinning to 14.44 keeps the dependency compiler aligned with the plugin
# and with CI on every machine.
#
# Included by every overlay triplet (noavx/avx/avx2/avx512) so all four variants build their dependencies with the same
# compiler.
set(VCPKG_PLATFORM_TOOLSET v143)
set(VCPKG_PLATFORM_TOOLSET_VERSION "14.44")
