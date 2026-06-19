#pragma once

// pprofWriter.h - serializes the Pass 2 profiler call tree into a pprof (profile.proto) file.
//
// pprof's on-disk format is a protobuf-encoded `Profile` message; we hand-roll the tiny subset of the
// wire format we need (varint + length-delimited records) rather than pull in protobuf, mirroring the
// hand-rolled little-endian ByteWriter in src/Replay/hdtReplayFormat.h. The output is written
// UNCOMPRESSED: pprof sniffs the gzip magic and falls back to parsing raw protobuf, so a plain .pb
// loads fine (gzip can be layered on later, or `gzip file.pb` after the fact, if a consumer needs it).

#include "replayProfiler.h"

#include <cstdint>
#include <string>
#include <vector>

namespace hdt::replay
{
	// Encodes the profiler call tree as a pprof Profile message (raw protobuf bytes). Pure - no IO -
	// so the self-test can assert on the bytes without touching disk. Each tree node becomes one
	// sample carrying its self-time (ns) and call count with a leaf-first location stack; pprof then
	// recomputes flat (= self) and cumulative (= subtree total) from those samples.
	std::vector<std::uint8_t> encodePprofProfile(const prof::ScopeNode& root);

	// Encodes the tree (see encodePprofProfile) and writes it to `path` as an uncompressed .pb file.
	// Throws std::runtime_error if the file cannot be opened or written (fail closed).
	void writePprofProfile(const prof::ScopeNode& root, const std::string& path);
}
