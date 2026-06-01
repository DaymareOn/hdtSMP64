# Performance flags carried into every vcpkg dependency (Bullet, TBB, fmt, detours, ...), Release config only.
#
# Bullet is statically linked into hdtSMP64.dll and runs the heavy collision/constraint math every frame, so optimising
# it matters as much as optimising our own code -- but the only way to reach it is through the triplet. These mirror the
# flags the plugin sets on itself in src/CMakeLists.txt:
#   /Ob3     most aggressive inline expansion
#   /Gw      put each global/static in its own COMDAT so /OPT:REF and LTCG can strip/place them
#   /fp:fast let the compiler emit FMA and reassociate FP math (large for FP-heavy physics; behavioural -- validated)
#   /GL      whole-program optimization; produces IL object files that the plugin's /LTCG link folds in, extending
#            cross-module optimization into Bullet itself
#
# Release only: Debug dependency builds stay un-LTCG'd and fully debuggable. /GL + /LTCG require the dependency compiler
# and the plugin compiler to be the *same* MSVC toolset version -- that alignment is what _pin_toolset.cmake guarantees.
string(APPEND VCPKG_CXX_FLAGS_RELEASE " /Ob3 /Gw /fp:fast /GL")
string(APPEND VCPKG_C_FLAGS_RELEASE " /Ob3 /Gw /fp:fast /GL")

# /GL (IL) object files require /LTCG at link time for any dependency that produces a DLL/EXE. Static libraries store the
# IL and are code-generated when the plugin links them with /LTCG, so this mainly future-proofs DLL-producing deps.
string(APPEND VCPKG_LINKER_FLAGS_RELEASE " /LTCG")
