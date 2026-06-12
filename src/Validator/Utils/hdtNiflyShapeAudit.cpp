#include "hdtNiflyShapeAudit.h"

#include <NifFile.hpp>

#include <filesystem>
#include <vector>

namespace hdt
{
	namespace
	{
		// Crash-class index checks on one shape, using nifly's post-load view.
		// nifly's PrepareData() already moved partition-stored vertex/triangle data
		// back into the shape, so GetNumVertices/GetTriangles see the real geometry
		// regardless of which layout the file used. The partition arrays are checked
		// too because the game renders skinned meshes from the partition, not the shape.
		bool isShapeGeometrySane(nifly::NifFile& nif, nifly::NiShape* shape)
		{
			const uint16_t numVerts = shape->GetNumVertices();
			if (numVerts == 0)
				return false;

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
			// A throwing load is treated the same as a failed load: not readable.
			out.fileLoaded = false;
			out.verdictByBlockIndex.clear();
		}

		return out;
	}
}
