// smp_replay - standalone SMP physics replay & benchmark harness (PLAN §10, §11).
//
// Loads a capture produced in-game, rebuilds the Bullet world through the RE shim, and runs N frames
// of the exact in-game physics step with no Skyrim and no SKSE plugin. Two passes (D10):
//   Pass 1 (headline)  - profiler not dumped; reports median/p95 ms/frame (warmup discarded).
//   Pass 2 (breakdown) - Bullet's CProfileManager per-scope tree.
//
// `smp_replay --selftest` runs the doctest unit suite (format round-trip + trust-boundary, the
// applyKinematicTarget seam, and an end-to-end replay-pipeline test) and exits non-zero on failure.

#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "Replay/hdtReplayFormat.h"
#include "pprofWriter.h"
#include "replayProfiler.h"
#include "replayRunner.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

using namespace hdt::replay;

namespace
{
	/// Summary statistics for one phase's per-frame times (milliseconds).
	struct Stats
	{
		std::size_t count = 0;
		double median = 0, p95 = 0, mean = 0, lo = 0, hi = 0;
	};

	/// Computes count/median/p95/mean/min/max over a set of per-frame times. Takes the vector by value
	/// because it sorts in place. Percentiles use linear interpolation between samples.
	Stats summarize(std::vector<double> ms)
	{
		Stats s;
		s.count = ms.size();
		if (ms.empty())
			return s;
		std::sort(ms.begin(), ms.end());
		auto pct = [&](double p) {
			const double idx = p * (ms.size() - 1);
			const std::size_t lo = static_cast<std::size_t>(idx);
			const std::size_t hi = std::min(lo + 1, ms.size() - 1);
			const double frac = idx - lo;
			return ms[lo] * (1.0 - frac) + ms[hi] * frac;
		};
		s.median = pct(0.5);
		s.p95 = pct(0.95);
		s.lo = ms.front();
		s.hi = ms.back();
		double sum = 0;
		for (double v : ms)
			sum += v;
		s.mean = sum / ms.size();
		return s;
	}

	/// Prints CLI usage to stderr.
	void printUsage(const char* exe)
	{
		std::fprintf(stderr,
			"smp_replay - SMP physics replay & benchmark harness\n"
			"usage:\n"
			"  %s <capture.bin> [--frames N] [--threads N] [--warmup N] [--pprof PATH] [--no-pprof]\n"
			"  %s --selftest\n"
			"\n"
			"  <capture.bin>   capture produced in-game via the Papyrus replay-capture toggle\n"
			"  --frames N      number of frames to step (0 = the whole capture; loops if larger)\n"
			"  --threads N     pin TBB worker threads (1 = single-threaded, for parity)\n"
			"  --warmup N      discard the first N timed frames from the headline (default 0)\n"
			"  --pprof PATH    write the Pass 2 breakdown as a pprof profile (default: <capture>.pb)\n"
			"  --no-pprof      do not write a pprof profile\n"
			"\n"
			"  The pprof profile reflects the single-threaded Pass 2 scope composition; the Pass 1\n"
			"  headline remains the parallel wall-clock timing source. Open with `pprof -http=: PATH`\n"
			"  or compare two runs with `pprof -diff_base=A.pb B.pb`.\n",
			exe, exe);
	}

	/// Derives the default pprof output path from the capture path: replaces the capture's file
	/// extension with ".pb" (e.g. crowd.bin -> crowd.pb), or appends ".pb" when it has no extension.
	std::string derivePprofPath(const std::string& capture)
	{
		const auto slash = capture.find_last_of("/\\");
		const auto dot = capture.find_last_of('.');
		if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
			return capture.substr(0, dot) + ".pb";
		return capture + ".pb";
	}

	/// Runs the embedded doctest suite; returns its exit code (non-zero on any failure).
	int runSelftest()
	{
		doctest::Context ctx;
		ctx.setOption("no-breaks", true);
		return ctx.run();
	}
}

/// Entry point: `--selftest` runs the unit suite; otherwise loads a capture and reports the two-pass
/// performance readout (or runs the `--check-golden` parity gate). Exit codes: 0 ok, 1 load/parity
/// failure, 2 bad arguments.
int main(int argc, char** argv)
{
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--selftest") == 0)
			return runSelftest();
	}

	std::string path;
	std::size_t frames = 0;
	int threads = 0;
	int warmup = 0;
	bool checkGolden = false;
	float tolerance = 0.01f;
	bool pprofEnabled = true;
	bool pprofPathExplicit = false;
	std::string pprofPath;

	for (int i = 1; i < argc; ++i) {
		const char* a = argv[i];
		auto nextInt = [&](int& out) {
			if (i + 1 < argc)
				out = std::atoi(argv[++i]);
		};
		if (std::strcmp(a, "--frames") == 0) {
			int v = 0;
			nextInt(v);
			frames = v < 0 ? 0 : static_cast<std::size_t>(v);
		} else if (std::strcmp(a, "--threads") == 0) {
			nextInt(threads);
		} else if (std::strcmp(a, "--warmup") == 0) {
			nextInt(warmup);
		} else if (std::strcmp(a, "--check-golden") == 0) {
			checkGolden = true;
		} else if (std::strcmp(a, "--tolerance") == 0) {
			if (i + 1 < argc)
				tolerance = static_cast<float>(std::atof(argv[++i]));
		} else if (std::strcmp(a, "--pprof") == 0) {
			if (i + 1 < argc) {
				pprofPath = argv[++i];
				pprofPathExplicit = true;
			}
		} else if (std::strcmp(a, "--no-pprof") == 0) {
			pprofEnabled = false;
		} else if (a[0] == '-') {
			std::fprintf(stderr, "unknown option: %s\n", a);
			printUsage(argv[0]);
			return 2;
		} else {
			path = a;
		}
	}

	if (path.empty()) {
		printUsage(argv[0]);
		return 2;
	}

	// Pin the TBB worker count (D10): both passes must use the same, known thread count, and parity
	// replays run single-threaded. Must outlive the worlds below.
	std::optional<tbb::global_control> threadPin;
	if (threads > 0)
		threadPin.emplace(tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(threads));

	setFlushToZero();

	Document doc;
	try {
		doc = deserialize(readFile(path));
	} catch (const ReplayFormatError& e) {
		std::fprintf(stderr, "error: %s\n", e.what());
		return 1;
	}

	std::printf("loaded %s\n", path.c_str());
	std::printf("  format version : %u\n", doc.header.formatVersion);
	std::printf("  git sha        : %s\n", doc.header.gitSha.empty() ? "(none)" : doc.header.gitSha.c_str());
	std::printf("  capture threads: %u\n", doc.header.threadCountHint);
	std::printf("  initial systems: %zu\n", doc.initialSystems.size());
	std::printf("  scene-log evts : %zu\n", doc.sceneLog.size());
	std::printf("  frames         : %zu\n", doc.frames.size());
	std::printf("  replay threads : %s\n", threads > 0 ? std::to_string(threads).c_str() : "(default)");

	// ---- Build/read cost: the in-game XML parse + NIF skin extraction + construction, timed in-game at
	// capture time and recorded per system (it can't be re-run offline; the build path is engine-coupled).
	{
		std::vector<double> startupMs, duringMs;
		for (const auto& s : doc.initialSystems)
			startupMs.push_back(s.buildTimeMicros / 1000.0);
		for (const auto& e : doc.sceneLog)
			if (e.kind == SceneEventKind::AddSystem)
				duringMs.push_back(e.snapshot.buildTimeMicros / 1000.0);

		auto buildRow = [](const char* name, std::vector<double> ms) {
			if (ms.empty()) {
				std::printf("  %-14s (none)\n", name);
				return;
			}
			double total = 0;
			for (double v : ms)
				total += v;
			const Stats s = summarize(std::move(ms));
			std::printf("  %-14s %zu systems | total %.3f ms | median %.4f | p95 %.4f | max %.4f\n",
				name, s.count, total, s.median, s.p95, s.hi);
		};
		std::printf("\n[Build] captured in-game build/read cost — XML parse + NIF skin extraction + construction\n");
		std::printf("  (measured in-game; the build path is engine-coupled, so it is recorded, not re-run offline)\n");
		buildRow("startup", startupMs);         // systems present at capture start (built before capture)
		buildRow("during capture", duringMs);   // systems built mid-capture (scene-log AddSystem events)
	}

	if (doc.frames.empty()) {
		std::fprintf(stderr, "error: capture has no frames to replay\n");
		return 1;
	}

	// ---- Seam-parity gate (D8): compare dynamic-bone output against the captured golden ----
	if (checkGolden) {
		if (!doc.header.hasGolden) {
			std::fprintf(stderr, "error: --check-golden requires a capture recorded with golden outputs\n");
			return 1;
		}
		LoadedWorld lw;
		initWorld(lw, doc);
		GoldenResult gr = runReplayCheckGolden(lw, doc, tolerance);
		std::printf("\n[parity] %zu comparisons | %zu mismatches | max error %.6f | tolerance %.6f\n",
			gr.comparisons, gr.mismatches, gr.maxError, tolerance);
		if (gr.mismatches > 0) {
			std::fprintf(stderr, "PARITY FAILED: %zu dynamic-bone outputs drifted past tolerance\n", gr.mismatches);
			return 1;
		}
		std::printf("PARITY OK\n");
		return 0;
	}

	// ---- Pass 1: headline (profiler not dumped) ----
	{
		LoadedWorld lw;
		initWorld(lw, doc);
		PhaseTimings t = runReplay(lw, doc, frames, warmup);

		// total = read + step + write, per frame
		std::vector<double> total(t.timedFrames());
		for (std::size_t i = 0; i < total.size(); ++i)
			total[i] = t.read[i] + t.step[i] + t.write[i];

		auto row = [](const char* name, const Stats& s) {
			std::printf("  %-6s median %.4f | p95 %.4f | mean %.4f | min %.4f | max %.4f\n",
				name, s.median, s.p95, s.mean, s.lo, s.hi);
		};
		std::printf("\n[Pass 1] headline ms/frame (over %zu timed frames, %d warmup discarded)\n",
			t.timedFrames(), warmup);
		std::printf("  (read/write are the physics-core halves only; Skyrim NiNode I/O is not present offline)\n");
		row("read", summarize(t.read));
		row("step", summarize(t.step));
		row("write", summarize(t.write));
		row("total", summarize(total));
	}

	// ---- Pass 2: per-scope breakdown (custom BT_PROFILE hook; single-threaded for a race-free tree) ----
	{
		tbb::global_control pin1(tbb::global_control::max_allowed_parallelism, 1);
		LoadedWorld lw;
		initWorld(lw, doc);
		prof::reset();
		prof::install();
		runReplay(lw, doc, frames, warmup);
		prof::uninstall();
		std::printf("\n[Pass 2] per-scope breakdown (single-threaded; total time per BT_PROFILE scope)\n");
		prof::report();

		if (pprofEnabled) {
			const std::string outPath = pprofPathExplicit ? pprofPath : derivePprofPath(path);
			try {
				writePprofProfile(prof::tree(), outPath);
				std::printf("\nwrote pprof profile: %s  (open: pprof -http=: %s)\n", outPath.c_str(), outPath.c_str());
			} catch (const std::exception& e) {
				std::fprintf(stderr, "warning: failed to write pprof profile: %s\n", e.what());
			}
		}
	}

	return 0;
}
