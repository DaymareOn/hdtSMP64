#include "hdtNiflyShapeAudit.h"

#include <NifFile.hpp>

#include <filesystem>
#include <vector>

namespace hdt
{
	namespace
	{
		// Checks that nothing in this shape points at a vertex that doesn't exist —
		// the kind of mistake that crashes the game. By the time we get here, nifly's
		// loader has already moved any vertex/triangle data that the file kept inside
		// the skin partition back into the shape, so GetNumVertices/GetTriangles show
		// the real geometry no matter how the file chose to store it. We also check the
		// partition's own lists, because the game draws skinned meshes from the
		// partition, not from the shape.
		bool isShapeGeometrySane(nifly::NifFile& nif, nifly::NiShape* shape)
		{
			// A shape with zero vertices is NOT broken by itself: BodySlide ShapeData
			// files legitimately contain empty shapes (a part zeroed out at one slider
			// weight). An empty shape holds nothing, so nothing in it can go wrong —
			// and if triangles or partition entries DO point at vertices that don't
			// exist, every index check below catches them (with zero vertices, every
			// index counts as out of range).
			const uint16_t numVerts = shape->GetNumVertices();

			std::vector<nifly::Triangle> tris;
			if (!shape->GetTriangles(tris))
				return false;
			for (const auto& tri : tris) {
				if (tri.p1 >= numVerts || tri.p2 >= numVerts || tri.p3 >= numVerts)
					return false;
			}

			auto& hdr = nif.GetHeader();
			auto* skinInstRef = shape->SkinInstanceRef();
			auto* skinInst = skinInstRef ? hdr.GetBlock<nifly::NiSkinInstance>(skinInstRef->index) : nullptr;
			if (skinInst) {
				auto* skinPart = hdr.GetBlock(skinInst->skinPartitionRef);
				if (skinPart) {
					for (const auto& part : skinPart->partitions) {
						for (const uint16_t v : part.vertexMap) {
							if (v >= numVerts)
								return false;
						}
						for (const auto& tri : part.trueTriangles) {
							if (tri.p1 >= numVerts || tri.p2 >= numVerts || tri.p3 >= numVerts)
								return false;
						}
					}
				}
			}

			return true;
		}
	}  // anonymous namespace

	NiflyShapeAudit AuditNifShapesWithNifly(const std::string& nifPath)
	{
		NiflyShapeAudit out;

		try {
			nifly::NifFile nif;
			if (nif.Load(std::filesystem::u8path(nifPath)) != 0)
				return out;

			out.fileLoaded = true;
			for (auto* shape : nif.GetShapes()) {
				const uint32_t blockId = nif.GetHeader().GetBlockID(shape);
				out.verdictByBlockIndex[blockId] = isShapeGeometrySane(nif, shape) ? NiflyShapeVerdict::ReadableAndSane : NiflyShapeVerdict::GeometryBroken;
			}
		} catch (...) {
			// If nifly blows up with an error while loading, treat it exactly like a
			// load that returned failure: the file is not readable.
			out.fileLoaded = false;
			out.verdictByBlockIndex.clear();
		}

		return out;
	}
}
