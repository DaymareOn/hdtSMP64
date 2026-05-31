#pragma once

#include <string>

namespace hdt
{
	// Removes obviously bogus NiNode blocks from NIF binaries and writes improved
	// files to:
	//   <outputDir>/<path-relative-to-data/>
	// only when at least one block was removed.
	//
	// Returns true  — a cleaned file was written.
	// Returns false — no changes, unsupported format, or I/O error.
	bool GenerateImprovedNIF(const std::string& srcNIFPath, const std::string& outputDir);
}
