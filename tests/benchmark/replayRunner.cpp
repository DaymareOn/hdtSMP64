#include "replayRunner.h"

#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <pmmintrin.h>
#include <xmmintrin.h>

namespace hdt::replay
{
	std::vector<uint8_t> readFile(const std::string& path)
	{
		std::ifstream in(path, std::ios::binary | std::ios::ate);
		if (!in)
			throw ReplayFormatError("cannot open capture file: " + path);
		const std::streamsize size = in.tellg();
		if (size <= 0)
			throw ReplayFormatError("capture file is empty: " + path);
		in.seekg(0);
		std::vector<uint8_t> buf(static_cast<std::size_t>(size));
		if (!in.read(reinterpret_cast<char*>(buf.data()), size))
			throw ReplayFormatError("failed reading capture file: " + path);
		return buf;
	}

	void setFlushToZero()
	{
		_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
		_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
	}

	namespace
	{
		/// Rebuilds a system from a snapshot, adds it to the world, and registers it by systemId so the
		/// per-frame stream and a later remove event can find it.
		void addSystem(LoadedWorld& lw, const Snapshot& snap)
		{
			auto sys = ReplaySystem::build(snap);
			lw.world.addSkinnedMeshSystem(sys.get());
			lw.systems[snap.systemId] = sys;
		}
	}

	void initWorld(LoadedWorld& lw, const Document& doc)
	{
		lw.world.applySolver(doc.solver);
		for (const auto& snap : doc.initialSystems)
			addSystem(lw, snap);
	}

	void applySceneEvents(LoadedWorld& lw, const Document& doc, uint32_t frameIndex)
	{
		for (const auto& e : doc.sceneLog) {
			if (e.frame != frameIndex)
				continue;
			if (e.kind == SceneEventKind::AddSystem) {
				addSystem(lw, e.snapshot);
			} else {
				auto it = lw.systems.find(e.systemId);
				if (it != lw.systems.end()) {
					lw.world.removeSkinnedMeshSystem(it->second.get());
					lw.systems.erase(it);
				}
			}
		}
	}

	PhaseTimings runReplay(LoadedWorld& lw, const Document& doc, std::size_t maxFrames, int warmup)
	{
		PhaseTimings timings;
		if (doc.frames.empty())
			return timings;

		const std::size_t total = maxFrames == 0 ? doc.frames.size() : maxFrames;
		timings.read.reserve(total);
		timings.step.reserve(total);
		timings.write.reserve(total);

		using clock = std::chrono::steady_clock;
		using ms = std::chrono::duration<double, std::milli>;

		for (std::size_t i = 0; i < total; ++i) {
			const std::size_t fi = i % doc.frames.size();
			// scene-log events are addressed by absolute capture-frame index; only meaningful on the
			// first pass over the stream.
			if (i < doc.frames.size())
				applySceneEvents(lw, doc, static_cast<uint32_t>(fi));

			const Frame& f = doc.frames[fi];

			for (const auto& t : f.kinematicTargets) {
				auto it = lw.systems.find(t.systemId);
				if (it == lw.systems.end())
					continue;
				if (ReplayBone* bn = it->second->bone(t.boneIndex))
					bn->setFrameTarget(toBtQs(t.target), f.reset != 0);
			}

			lw.world.setWind(toBt(f.windSpeed));

			// Each phase is both wall-clock timed (chrono, feeds the headline) and wrapped in a
			// BT_PROFILE scope (feeds the pass-2 profiler tree). read/write here are the physics-core
			// halves only - the Skyrim NiNode I/O and updateTransformUpDown don't exist offline.
			const auto a = clock::now();
			{
				BT_PROFILE("replay_readTransform");
				lw.world.readTransform(f.remainingTimeStep);
			}
			const auto b = clock::now();
			{
				BT_PROFILE("replay_step");
				const btVector3 offset = lw.world.applyTranslationOffset();
				lw.world.stepSimulation(f.remainingTimeStep, 0, f.tick);
				lw.world.restoreTranslationOffset(offset);
			}
			const auto c = clock::now();
			{
				BT_PROFILE("replay_writeTransform");
				lw.world.writeTransform();
			}
			const auto d = clock::now();

			if (static_cast<int>(i) >= warmup) {
				timings.read.push_back(ms(b - a).count());
				timings.step.push_back(ms(c - b).count());
				timings.write.push_back(ms(d - c).count());
			}
		}

		return timings;
	}

	GoldenResult runReplayCheckGolden(LoadedWorld& lw, const Document& doc, float tolerance)
	{
		GoldenResult result;

		for (std::size_t fi = 0; fi < doc.frames.size(); ++fi) {
			applySceneEvents(lw, doc, static_cast<uint32_t>(fi));
			const Frame& f = doc.frames[fi];

			for (const auto& t : f.kinematicTargets) {
				auto it = lw.systems.find(t.systemId);
				if (it == lw.systems.end())
					continue;
				if (ReplayBone* bn = it->second->bone(t.boneIndex))
					bn->setFrameTarget(toBtQs(t.target), f.reset != 0);
			}

			lw.world.setWind(toBt(f.windSpeed));

			lw.world.readTransform(f.remainingTimeStep);
			const btVector3 offset = lw.world.applyTranslationOffset();
			lw.world.stepSimulation(f.remainingTimeStep, 0, f.tick);
			lw.world.restoreTranslationOffset(offset);
			lw.world.writeTransform();

			for (const auto& g : f.golden) {
				auto it = lw.systems.find(g.systemId);
				if (it == lw.systems.end())
					continue;
				ReplayBone* bn = it->second->bone(g.boneIndex);
				if (!bn)
					continue;
				const btVector3 got = bn->m_currentTransform.getOrigin();
				const btVector3 want = toBt(g.transform.origin);
				const double err = (got - want).length();
				result.maxError = std::max(result.maxError, err);
				++result.comparisons;
				if (err > tolerance)
					++result.mismatches;
			}
		}

		return result;
	}
}
