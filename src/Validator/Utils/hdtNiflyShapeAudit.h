#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace hdt
{
	enum class NiflyShapeVerdict
	{
		ReadableAndSane,  // nifly read the shape and every vertex/triangle index is in range
		GeometryBroken,   // nifly read the file but this shape's geometry is unusable
	};

	struct NiflyShapeAudit
	{
		bool fileLoaded = false;                                              // false = nifly could not load the file at all
		std::unordered_map<uint32_t, NiflyShapeVerdict> verdictByBlockIndex;  // one entry per shape block
	};

	/// Reads a NIF with the nifly library and gives every shape in it a verdict,
	/// stored under the shape's block number. nifly can read file layouts our own
	/// binary parsers cannot — for example SSE skinned shapes that keep all their
	/// geometry inside the NiSkinPartition and store zero counts in the BSTriShape
	/// (how BodySlide/OutfitStudio exports them). So when our parser gives up on a
	/// shape, this audit is the second opinion that decides whether the shape is
	/// actually fine. A shape is ReadableAndSane when every triangle index — in the
	/// shape and in each skin partition — points at a vertex that exists. Empty
	/// shapes (zero vertices, zero triangles, empty partitions) count as sane:
	/// BodySlide ShapeData files legitimately contain them, and a shape that holds
	/// nothing cannot point at anything that would crash.
	NiflyShapeAudit AuditNifShapesWithNifly(const std::string& nifPath);
}
