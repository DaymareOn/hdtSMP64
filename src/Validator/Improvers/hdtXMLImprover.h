#pragma once

#include <string>

namespace hdt
{
	// Loads the XML at srcXMLPath, removes elements that are either completely
	// unknown to the hdtSMP64.xsd schema or placed inside a parent that does
	// not allow them, and prunes child tags whose values are redundant relative
	// to effective template inheritance in document order, then writes the result to
	//   <outputDir>/<path-relative-to-data/>
	// only when at least one element was actually removed.
	//
	// Returns true  — a cleaned file was written.
	// Returns false — no changes were needed, or the file could not be processed.
	bool GenerateImprovedXML(
		const std::string& srcXMLPath,
		const std::string& outputDir,
		bool copyOriginal = false,
		bool stateless = false);
}
