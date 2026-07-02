// Test runner for the GlobalConfig JSON (de)serialization. Running the executable with no arguments runs
// the whole doctest suite and exits non-zero on any failure (so CTest / CI can gate on it). GlobalConfig
// is pure rapidjson + STL, so this target links neither CommonLibSSE, Bullet, nor TBB.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
