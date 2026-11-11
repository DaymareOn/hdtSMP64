// Test runner for the external mesh owner registry (src/ExternalOwner.cpp). Running the executable with no
// arguments runs the whole doctest suite and exits non-zero on any failure (so CTest / CI can gate on it).
// ExternalOwner is pure STL plus the dependency-light PluginOwnerAPI.h, so this target links neither
// CommonLibSSE, Bullet, nor TBB.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
