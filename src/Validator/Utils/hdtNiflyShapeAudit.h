#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace hdt
{
	enum class NiflyShapeVerdict
	{
		ReadableAndSane,   // nifly read the shape and every vertex/triangle index is in range
		GeometryBroken,    // nifly read the file but this shape's geometry is unusable
	};

	struct NiflyShapeAudit
	{
		bool fileLoaded = false;                                       // false = nifly could not load the file at all
		std::unordered_map<uint32_t, NiflyShapeVerdict> verdictByBlockIndex;  // one entry per shape block
	};

	/// Reads a NIF with nifly and grades every shape in it, keyed by block index.
	/// nifly understands layouts our hand-rolled binary parsers do not (e.g. SSE
	/// skinned shapes whose geometry lives only in the NiSkinPartition, with zeroed
	/// counts in the BSTriShape — the BodySlide/OutfitStudio export convention), so
	/// this is the second opinion consulted when the binary parser rejects a shape.
	/// A shape is ReadableAndSane when it has vertices and every triangle index —
	/// in the shape and in each skin partition — points at an existing vertex.
	NiflyShapeAudit AuditNifShapesWithNifly(const std::string& nifPath);
}
