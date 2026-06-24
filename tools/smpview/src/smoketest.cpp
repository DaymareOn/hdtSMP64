// smpview Phase 0 smoke test.
//
// Proves the standalone smpcore static lib links and runs: it constructs the
// real hdt::SkinnedMeshWorld (Bullet MT world + custom dispatcher/solver pool,
// all from FSMP's unmodified physics core) and steps it a few frames with no
// systems attached. No CommonLibSSE, no Skyrim runtime. If this exits 0, the
// physics core is usable outside the game — the foundation every later smpview
// phase (nifly load, render, tuning, XML export) builds on.
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"

#include <cstdio>

int main()
{
	hdt::SkinnedMeshWorld world;

	// Exercise a member hoisted into the base during decoupling (was on the
	// Skyrim-only SkyrimPhysicsWorld): toggling it must compile and link here.
	world.m_enableWind = false;

	constexpr int kFrames = 8;
	for (int i = 0; i < kFrames; ++i)
		world.stepSimulation(1.0f / 60.0f);

	std::printf("smpcore OK: stepped empty SkinnedMeshWorld %d frames standalone\n", kFrames);
	return 0;
}
