#include "hdtNIFSkinMeshValidator.h"

#include "../Schema/hdtNifSchema.h"
#include "../Utils/hdtNIFBinaryUtils.h"
#include "../Utils/hdtNiflyShapeAudit.h"
#include "hdtNIFBinaryIO.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	namespace
	{
		// ── Domain types ──────────────────────────────────────────────────────────

		struct VertexLayout
		{
			uint64_t desc = 0;
			uint16_t attributes = 0;
			uint32_t vertexSize = 0;
		};

		struct TriShapeView
		{
			VertexLayout layout;
			uint16_t numVertices = 0;
			uint16_t numTriangles = 0;
			std::vector<uint8_t> vertexData;
			std::vector<std::array<uint16_t, 3>> triangles;
		};

		struct PartitionView
		{
			VertexLayout layout;
			uint16_t numVertices = 0;
			uint16_t numTriangles = 0;
			std::vector<uint8_t> vertexData;
			std::vector<uint16_t> vertexMap;
			std::vector<std::array<uint16_t, 3>> triangles;
			std::vector<std::array<uint16_t, 3>> trianglesCopy;
			size_t trianglesOffset = 0;
			size_t trianglesCopyOffset = 0;
			bool trianglesMismatch = false;
		};

		// ── Binary helpers ────────────────────────────────────────────────────────

		/// Read one byte as a strict boolean: 0 → false, 1 → true, anything else → parse failure.
		bool readBoolStrict(NifReader& r, bool& out)
		{
			uint8_t v = r.readU8();
			if (v > 1)
				return false;
			out = (v != 0);
			return true;
		}

		// ── Vertex layout ─────────────────────────────────────────────────────────

		/// Returns nullopt for unsupported attribute combinations or inconsistent field offsets.
		std::optional<VertexLayout> parseVertexLayout(uint64_t desc)
		{
			VertexLayout out;
			out.desc = desc;
			out.attributes = static_cast<uint16_t>((desc >> 44) & 0x0FFFu);
			out.vertexSize = static_cast<uint32_t>(desc & 0x0Fu) * 4u;

			const uint16_t required = nif::kVertexAttrVertex | nif::kVertexAttrSkinned;
			const uint16_t allowed = nif::kVertexAttrVertex | nif::kVertexAttrUV |
			                         nif::kVertexAttrNormal | nif::kVertexAttrTangent |
			                         nif::kVertexAttrColor | nif::kVertexAttrSkinned;
			if ((out.attributes & required) != required)
				return std::nullopt;
			if ((out.attributes & ~allowed) != 0)
				return std::nullopt;
			if ((out.attributes & nif::kVertexAttrTangent) != 0 &&
				(out.attributes & nif::kVertexAttrNormal) == 0)
				return std::nullopt;
			if (((desc >> 4) & 0x0Fu) != 0)
				return std::nullopt;

			auto expectOff = [&](uint32_t shift, uint32_t expectedDwords) {
				return static_cast<uint32_t>((desc >> shift) & 0x0Fu) == expectedDwords;
			};

			uint32_t cur = 4;
			if ((out.attributes & nif::kVertexAttrUV) != 0) {
				if (!expectOff(8, cur))
					return std::nullopt;
				cur += 1;
			} else if (((desc >> 8) & 0x0Fu) != 0)
				return std::nullopt;
			if ((out.attributes & nif::kVertexAttrNormal) != 0) {
				if (!expectOff(16, cur))
					return std::nullopt;
				cur += 1;
			} else if (((desc >> 16) & 0x0Fu) != 0)
				return std::nullopt;
			if ((out.attributes & nif::kVertexAttrTangent) != 0) {
				if (!expectOff(20, cur))
					return std::nullopt;
				cur += 1;
			} else if (((desc >> 20) & 0x0Fu) != 0)
				return std::nullopt;
			if ((out.attributes & nif::kVertexAttrColor) != 0) {
				if (!expectOff(24, cur))
					return std::nullopt;
				cur += 1;
			} else if (((desc >> 24) & 0x0Fu) != 0)
				return std::nullopt;
			if (!expectOff(28, cur))
				return std::nullopt;
			cur += 3;
			if (out.vertexSize != cur * 4u)
				return std::nullopt;
			return out;
		}

		// ── Block parsers ─────────────────────────────────────────────────────────

		/// Parse a BSTriShape/BSDynamicTriShape block with the same acceptance criteria
		/// as parseSupportedSSETriShapeBlock in hdtNIFDecimator.cpp.
		/// Stores only the fields needed for steps 4–11 detection.
		std::optional<TriShapeView> parseTriShape(
			const std::vector<uint8_t>& block,
			const std::string& shapeType,
			uint32_t bsVersion)
		{
			if (bsVersion != nif::kSSEBsVersion || !nif::isSupportedSSETriShapeType(shapeType))
				return std::nullopt;

			try {
				NifReader r(block);
				r.readU32();
				uint32_t numExtra = r.readU32();
				if (!r.canRead(static_cast<size_t>(numExtra) * 4 + 4 + 4 + 12 + 36 + 4 + 4 + 16 + 4 + 4 + 4 + 8))
					return std::nullopt;
				r.skip(static_cast<size_t>(numExtra) * 4);
				r.readU32();
				r.readU32();
				r.skip(12 + 36 + 4);
				r.readU32();
				r.skip(16);
				r.readU32();
				r.readU32();
				r.readU32();

				uint64_t vertexDesc = r.readU64();
				auto layoutOpt = parseVertexLayout(vertexDesc);
				if (!layoutOpt)
					return std::nullopt;

				TriShapeView out;
				out.layout = *layoutOpt;

				{
					uint32_t rawNumTri = r.readU32();
					if (rawNumTri > std::numeric_limits<uint16_t>::max())
						return std::nullopt;
					out.numTriangles = static_cast<uint16_t>(rawNumTri);
				}
				out.numVertices = r.readU16();
				uint32_t dataSize = r.readU32();
				const uint32_t expectedDataSize =
					static_cast<uint32_t>(out.numVertices) * out.layout.vertexSize +
					static_cast<uint32_t>(out.numTriangles) * nif::kTriangleByteSize;
				if (dataSize != expectedDataSize)
					return std::nullopt;

				out.vertexData = r.readBytes(static_cast<size_t>(out.numVertices) * out.layout.vertexSize);
				out.triangles.clear();
				out.triangles.reserve(out.numTriangles);
				for (uint16_t i = 0; i < out.numTriangles; ++i)
					out.triangles.push_back({ r.readU16(), r.readU16(), r.readU16() });

				uint32_t particleDataSize = r.readU32();
				const uint32_t expectedParticleDataSize =
					static_cast<uint32_t>(out.numVertices) * 12u +
					static_cast<uint32_t>(out.numTriangles) * nif::kTriangleByteSize;
				if (particleDataSize != expectedParticleDataSize)
					return std::nullopt;
				r.skip(static_cast<size_t>(out.numVertices) * 6);  // particle positions
				r.skip(static_cast<size_t>(out.numVertices) * 6);  // particle normals
				std::vector<std::array<uint16_t, 3>> particleTris;
				particleTris.reserve(out.numTriangles);
				for (uint16_t i = 0; i < out.numTriangles; ++i)
					particleTris.push_back({ r.readU16(), r.readU16(), r.readU16() });
				if (out.triangles != particleTris)
					return std::nullopt;

				if (shapeType == nif::kTypeBSDynamicTriShape) {
					uint32_t dynSize = r.readU32();
					if (dynSize != static_cast<uint32_t>(out.numVertices) * 16u)
						return std::nullopt;
					r.skip(dynSize);
				}
				if (r.pos() != block.size())
					return std::nullopt;
				return out;
			} catch (...) {
				return std::nullopt;
			}
		}

		/// Parse a NiSkinPartition block tolerantly: identical to the strict parser in
		/// hdtNIFDecimator.cpp except the triangles == trianglesCopy
		/// equality check is omitted.  Records trianglesOffset and trianglesCopyOffset
		/// so the repair pass can overwrite the copy in-place.
		std::optional<PartitionView> parsePartitionTolerant(
			const std::vector<uint8_t>& block,
			uint32_t bsVersion)
		{
			if (bsVersion != nif::kSSEBsVersion)
				return std::nullopt;

			try {
				NifReader r(block);
				if (r.readU32() != 1)
					return std::nullopt;  // numPartitions must be 1
				uint32_t dataSize = r.readU32();
				uint32_t vertexSize = r.readU32();
				uint64_t vertexDesc = r.readU64();
				auto layoutOpt = parseVertexLayout(vertexDesc);
				if (!layoutOpt || layoutOpt->vertexSize != vertexSize)
					return std::nullopt;
				if ((dataSize % vertexSize) != 0)
					return std::nullopt;

				PartitionView out;
				out.layout = *layoutOpt;
				out.vertexData = r.readBytes(dataSize);

				out.numVertices = r.readU16();
				out.numTriangles = r.readU16();
				uint16_t numBones = r.readU16();
				uint16_t numStrips = r.readU16();
				uint16_t weightsPerVertex = r.readU16();
				if (numStrips != 0 || weightsPerVertex != 4)
					return std::nullopt;
				if (out.vertexData.size() != static_cast<size_t>(out.numVertices) * vertexSize)
					return std::nullopt;

				for (uint16_t i = 0; i < numBones; ++i) r.readU16();

				bool hasVertexMap = false;
				if (!readBoolStrict(r, hasVertexMap) || !hasVertexMap)
					return std::nullopt;
				out.vertexMap.reserve(out.numVertices);
				for (uint16_t i = 0; i < out.numVertices; ++i)
					out.vertexMap.push_back(r.readU16());

				bool hasVertexWeights = false;
				if (!readBoolStrict(r, hasVertexWeights) || !hasVertexWeights)
					return std::nullopt;
				r.skip(static_cast<size_t>(out.numVertices) * weightsPerVertex * 4);

				bool hasFaces = false;
				if (!readBoolStrict(r, hasFaces) || !hasFaces)
					return std::nullopt;
				out.trianglesOffset = r.pos();
				out.triangles.clear();
				out.triangles.reserve(out.numTriangles);
				for (uint16_t i = 0; i < out.numTriangles; ++i)
					out.triangles.push_back({ r.readU16(), r.readU16(), r.readU16() });

				bool hasBoneIndices = false;
				if (!readBoolStrict(r, hasBoneIndices) || !hasBoneIndices)
					return std::nullopt;
				r.skip(static_cast<size_t>(out.numVertices) * weightsPerVertex);

				bool globalVB = false;
				r.readU8();  // lodLevel (value not checked)
				if (!readBoolStrict(r, globalVB) || globalVB)
					return std::nullopt;

				if (r.readU64() != vertexDesc)
					return std::nullopt;

				out.trianglesCopyOffset = r.pos();
				out.trianglesCopy.clear();
				out.trianglesCopy.reserve(out.numTriangles);
				for (uint16_t i = 0; i < out.numTriangles; ++i)
					out.trianglesCopy.push_back({ r.readU16(), r.readU16(), r.readU16() });

				// Trailing-bytes check kept — extra bytes indicate a different failure.
				if (r.pos() != block.size())
					return std::nullopt;

				out.trianglesMismatch = (out.triangles != out.trianglesCopy);
				return out;
			} catch (...) {
				return std::nullopt;
			}
		}

		// ── Vertex / triangle validation ──────────────────────────────────────────
		// TODO: deduplicate with identical functions in hdtNIFDecimator.cpp.

		bool isPermutationVertexMap(const std::vector<uint16_t>& vertexMap)
		{
			std::vector<uint8_t> seen(vertexMap.size(), 0);
			for (uint32_t i = 0; i < static_cast<uint32_t>(vertexMap.size()); ++i) {
				uint16_t mapped = vertexMap[i];
				if (mapped >= vertexMap.size())
					return false;
				if (seen[mapped] != 0)
					return false;
				seen[mapped] = 1;
			}
			for (uint8_t v : seen)
				if (v == 0)
					return false;
			return true;
		}

		bool vertexDataMatchesMappedOrder(
			const std::vector<uint8_t>& tsData,
			const std::vector<uint8_t>& partData,
			const std::vector<uint16_t>& vertexMap,
			uint32_t stride)
		{
			if (vertexMap.empty())
				return tsData.empty() && partData.empty();
			if (tsData.size() != vertexMap.size() * stride)
				return false;
			if (partData.size() != vertexMap.size() * stride)
				return false;
			for (uint32_t partIdx = 0; partIdx < static_cast<uint32_t>(vertexMap.size()); ++partIdx) {
				uint16_t shapeIdx = vertexMap[partIdx];
				if (std::memcmp(tsData.data() + static_cast<size_t>(shapeIdx) * stride,
						partData.data() + static_cast<size_t>(partIdx) * stride,
						stride) != 0)
					return false;
			}
			return true;
		}

		/// Returns the remapped triangles, or nullopt if any index is out of range of the map.
		std::optional<std::vector<std::array<uint16_t, 3>>> mapTriangles(
			const std::vector<std::array<uint16_t, 3>>& tris,
			const std::vector<uint16_t>& vertexMap)
		{
			std::vector<std::array<uint16_t, 3>> out;
			out.reserve(tris.size());
			for (const auto& tri : tris) {
				if (tri[0] >= vertexMap.size() || tri[1] >= vertexMap.size() || tri[2] >= vertexMap.size())
					return std::nullopt;
				out.push_back({ vertexMap[tri[0]], vertexMap[tri[1]], vertexMap[tri[2]] });
			}
			return out;
		}

		// ── Candidate discovery ───────────────────────────────────────────────────
		// TODO: extract to a shared header — mirrors discoverDecimationCandidates in
		// hdtNIFDecimator.cpp and buildRefOutbound in
		// hdtNIFOrphanedSkinImprover.cpp.

		struct SkinMeshCandidate
		{
			int triShapeBlockIndex = -1;
			int partitionBlockIndex = -1;
			std::string shapeType;
		};

		std::vector<SkinMeshCandidate> discoverSkinMeshCandidates(const ParsedNif& parsed)
		{
			const NifSchema& schema = globalNifSchema();
			const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());

			std::vector<std::vector<int32_t>> outbound(numBlocks);
			for (int32_t i = 0; i < numBlocks; ++i) {
				auto tOpt = blockTypeOf(parsed, i);
				if (!tOpt)
					continue;
				const auto& block = parsed.blocks[static_cast<size_t>(i)];
				auto offsets = walkBlockRefs(schema, *tOpt, block.data(), block.size(),
					numBlocks, LinkFilter::RefOnly);
				if (!offsets)
					continue;
				for (size_t off : *offsets) {
					if (off + 4 > block.size())
						continue;
					int32_t v = -1;
					std::memcpy(&v, block.data() + off, 4);
					if (v >= 0 && v < numBlocks)
						outbound[static_cast<size_t>(i)].push_back(v);
				}
			}

			std::vector<SkinMeshCandidate> out;
			for (int32_t i = 0; i < numBlocks; ++i) {
				auto tOpt = blockTypeOf(parsed, i);
				if (!tOpt)
					continue;
				if (!nif::isSupportedSSETriShapeType(*tOpt))
					continue;

				SkinMeshCandidate c;
				c.triShapeBlockIndex = i;
				c.shapeType = *tOpt;

				int skinInstanceIdx = -1;
				for (int32_t ref : outbound[static_cast<size_t>(i)]) {
					auto rt = blockTypeOf(parsed, ref);
					if (rt && (*rt == nif::kTypeNiSkinInstance ||
								  *rt == nif::kTypeBSDismemberSkinInstance)) {
						skinInstanceIdx = ref;
						break;
					}
				}
				if (skinInstanceIdx < 0)
					continue;

				for (int32_t ref : outbound[static_cast<size_t>(skinInstanceIdx)]) {
					auto rt = blockTypeOf(parsed, ref);
					if (rt && *rt == nif::kTypeNiSkinPartition) {
						c.partitionBlockIndex = ref;
						break;
					}
				}
				if (c.partitionBlockIndex < 0)
					continue;

				out.push_back(std::move(c));
			}
			return out;
		}

	}  // namespace

	// ── Public API ────────────────────────────────────────────────────────────────

	std::vector<NifSkinMeshIssue> detectNIFSkinMeshIssues(const ParsedNif& parsed, const std::string& nifPath)
	{
		std::vector<NifSkinMeshIssue> issues;
		auto candidates = discoverSkinMeshCandidates(parsed);

		// Second opinion for file layouts our binary parser does not know (e.g. SSE
		// shapes that keep all their geometry inside the NiSkinPartition). The file is
		// loaded through nifly at most once, and only when a shape was actually rejected.
		std::optional<NiflyShapeAudit> niflyAudit;
		auto niflyAcceptsShape = [&](int blockIndex) {
			if (!niflyAudit)
				niflyAudit = AuditNifShapesWithNifly(nifPath);
			if (!niflyAudit->fileLoaded)
				return false;
			auto it = niflyAudit->verdictByBlockIndex.find(static_cast<uint32_t>(blockIndex));
			return it != niflyAudit->verdictByBlockIndex.end() &&
			       it->second == NiflyShapeVerdict::ReadableAndSane;
		};

		for (const auto& c : candidates) {
			auto addIssue = [&](std::string reasonCode) {
				issues.push_back({ c.triShapeBlockIndex, c.shapeType, std::move(reasonCode) });
			};

			const auto& tsBlock = parsed.blocks[static_cast<size_t>(c.triShapeBlockIndex)];
			const auto& partBlock = parsed.blocks[static_cast<size_t>(c.partitionBlockIndex)];

			// Step 4
			auto tsOpt = parseTriShape(tsBlock, c.shapeType, parsed.bsVersion);
			if (!tsOpt) {
				if (!niflyAcceptsShape(c.triShapeBlockIndex))
					addIssue("unsupported-trishape-layout");
				continue;
			}

			// Step 5 (tolerant — accepts triangle copy mismatch)
			auto partOpt = parsePartitionTolerant(partBlock, parsed.bsVersion);
			if (!partOpt) {
				if (!niflyAcceptsShape(c.triShapeBlockIndex))
					addIssue("unsupported-skin-partition-layout");
				continue;
			}

			const auto& ts = *tsOpt;
			const auto& part = *partOpt;

			// Step 9: detected during tolerant parse
			if (part.trianglesMismatch)
				addIssue("partition-triangle-copy-mismatch");

			// Step 6: count/descriptor mismatch — fatal, subsequent checks are unreliable
			if (ts.layout.desc != part.layout.desc ||
				ts.numVertices != part.numVertices ||
				ts.numTriangles != part.numTriangles) {
				addIssue("shape-partition-count-mismatch");
				continue;
			}

			// Step 7
			const bool goodVertexMap = isPermutationVertexMap(part.vertexMap);
			if (!goodVertexMap)
				addIssue("unsupported-non-permutation-vertex-map");

			// Steps 8 and 10 require a valid vertex map to avoid false positives
			if (goodVertexMap) {
				// Step 8
				if (!vertexDataMatchesMappedOrder(ts.vertexData, part.vertexData,
						part.vertexMap, ts.layout.vertexSize))
					addIssue("shape-partition-vertexdata-mismatch");

				// Step 10
				auto remapped = mapTriangles(part.triangles, part.vertexMap);
				if (!remapped || ts.triangles != *remapped)
					addIssue("shape-partition-triangle-mismatch");
			}

			// Step 11
			for (const auto& tri : part.triangles) {
				if (tri[0] >= part.numVertices ||
					tri[1] >= part.numVertices ||
					tri[2] >= part.numVertices) {
					addIssue("triangle-index-out-of-range");
					break;
				}
			}
		}

		return issues;
	}

	std::optional<PartitionMismatchInfo> checkPartitionTriangleMismatch(
		const std::vector<uint8_t>& block, uint32_t bsVersion)
	{
		auto partOpt = parsePartitionTolerant(block, bsVersion);
		if (!partOpt || !partOpt->trianglesMismatch)
			return std::nullopt;
		return PartitionMismatchInfo{
			partOpt->trianglesOffset,
			partOpt->trianglesCopyOffset,
			partOpt->numTriangles
		};
	}
}
