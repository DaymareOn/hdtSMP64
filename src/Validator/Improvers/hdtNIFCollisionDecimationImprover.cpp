#include "hdtNIFCollisionDecimationImprover.h"

#include "hdtCollisionMeshDecimator.h"

#include "hdtNIFBinaryIO.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdt
{
	static constexpr uint16_t kVertexAttrVertex = 1u << 0;
	static constexpr uint16_t kVertexAttrUV = 1u << 1;
	static constexpr uint16_t kVertexAttrUV2 = 1u << 2;
	static constexpr uint16_t kVertexAttrNormal = 1u << 3;
	static constexpr uint16_t kVertexAttrTangent = 1u << 4;
	static constexpr uint16_t kVertexAttrColor = 1u << 5;
	static constexpr uint16_t kVertexAttrSkinned = 1u << 6;
	static constexpr uint16_t kVertexAttrLand = 1u << 7;
	static constexpr uint16_t kVertexAttrEye = 1u << 8;
	static constexpr uint16_t kVertexAttrInstance = 1u << 9;

	struct SupportedSSEVertexLayout
	{
		uint64_t desc = 0;
		uint16_t attributes = 0;
		uint32_t vertexSize = 0;
		uint32_t skinOffset = 0;
	};

	struct ParsedSupportedSSETriShape
	{
		std::vector<uint8_t> prefix;
		SupportedSSEVertexLayout layout;
		bool isDynamic = false;
		uint16_t numVertices = 0;
		uint16_t numTriangles = 0;
		std::vector<uint8_t> vertexData;
		std::vector<std::array<uint16_t, 3>> triangles;
		std::vector<std::array<uint8_t, 6>> particleNormals;
		std::vector<uint8_t> dynamicVertexData;
	};

	struct ParsedSupportedSSESkinPartition
	{
		SupportedSSEVertexLayout layout;
		uint16_t numVertices = 0;
		uint16_t numTriangles = 0;
		uint16_t numBones = 0;
		uint16_t numWeightsPerVertex = 0;
		std::vector<uint16_t> bones;
		std::vector<uint8_t> vertexData;
		std::vector<uint16_t> vertexMap;
		std::vector<float> vertexWeights;
		std::vector<uint8_t> boneIndices;
		std::vector<std::array<uint16_t, 3>> triangles;
		std::vector<std::array<uint16_t, 3>> trianglesCopy;
		uint8_t lodLevel = 0;
		bool globalVB = false;
	};

	enum class DecimationApplyDecision
	{
		Apply,
		SkipNoChange,
		SkipUnsafe
	};

	struct ShapeDecimationCandidate
	{
		int triShapeBlockIndex = -1;
		int skinInstanceBlockIndex = -1;
		int skinPartitionBlockIndex = -1;
		std::string shapeType;
	};

	struct ShapeDecimationResult
	{
		ShapeDecimationCandidate candidate;
		DecimationApplyDecision decision = DecimationApplyDecision::SkipUnsafe;
		std::string reason;
		CollisionMeshDecimationStats stats;
	};

	/// Convert a 32-bit float into IEEE-754 half precision for packed NIF fields.
	static uint16_t floatToHalf(float value)
	{
		uint32_t bits = 0;
		std::memcpy(&bits, &value, sizeof(bits));

		uint32_t sign = (bits >> 16) & 0x8000u;
		uint32_t mantissa = bits & 0x007FFFFFu;
		int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xFFu) - 127 + 15;

		if (exponent <= 0) {
			if (exponent < -10)
				return static_cast<uint16_t>(sign);
			mantissa = (mantissa | 0x00800000u) >> (1 - exponent);
			return static_cast<uint16_t>(sign | ((mantissa + 0x00001000u) >> 13));
		}

		if (exponent >= 31) {
			if (mantissa != 0)
				return static_cast<uint16_t>(sign | 0x7C00u | ((mantissa >> 13) ? (mantissa >> 13) : 1u));
			return static_cast<uint16_t>(sign | 0x7C00u);
		}

		return static_cast<uint16_t>(sign | (static_cast<uint32_t>(exponent) << 10) | ((mantissa + 0x00001000u) >> 13));
	}

	/// Convert an IEEE-754 half precision value back to a 32-bit float.
	static float halfToFloat(uint16_t value)
	{
		uint32_t sign = (static_cast<uint32_t>(value & 0x8000u)) << 16;
		uint32_t exponent = (value >> 10) & 0x1Fu;
		uint32_t mantissa = value & 0x03FFu;
		uint32_t out = 0;

		if (exponent == 0) {
			if (mantissa == 0) {
				out = sign;
			} else {
				exponent = 1;
				while ((mantissa & 0x0400u) == 0) {
					mantissa <<= 1;
					--exponent;
				}
				mantissa &= 0x03FFu;
				out = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
			}
		} else if (exponent == 31) {
			out = sign | 0x7F800000u | (mantissa << 13);
		} else {
			out = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
		}

		float result = 0.0f;
		std::memcpy(&result, &out, sizeof(result));
		return result;
	}

	/// Read a strict boolean from the stream; only 0 and 1 are accepted.
	static bool readBoolStrict(NifReader& r, bool& out)
	{
		uint8_t v = r.readU8();
		if (v > 1)
			return false;
		out = (v != 0);
		return true;
	}

	/// Write a vector as three half-floats (x, y, z).
	static void writeHalfVec3(std::vector<uint8_t>& out, const btVector3& v)
	{
		appendU16(out, floatToHalf(v.x()));
		appendU16(out, floatToHalf(v.y()));
		appendU16(out, floatToHalf(v.z()));
	}

	/// Serialize triangle indices as packed uint16 triplets.
	static void writeTriangleArray(std::vector<uint8_t>& out, const std::vector<std::array<uint32_t, 3>>& triangles)
	{
		for (const auto& tri : triangles) {
			appendU16(out, static_cast<uint16_t>(tri[0]));
			appendU16(out, static_cast<uint16_t>(tri[1]));
			appendU16(out, static_cast<uint16_t>(tri[2]));
		}
	}

	/// Parse and validate the supported SSE vertex descriptor layout.
	/// Returns nullopt when the descriptor uses unsupported attributes or offsets.
	static std::optional<SupportedSSEVertexLayout> parseSupportedSSEVertexLayout(uint64_t desc)
	{
		SupportedSSEVertexLayout out;
		out.desc = desc;
		out.attributes = static_cast<uint16_t>((desc >> 44) & 0x0FFFu);
		out.vertexSize = static_cast<uint32_t>(desc & 0x0Fu) * 4u;
		out.skinOffset = static_cast<uint32_t>((desc >> 28) & 0x0Fu);

		const uint16_t required = kVertexAttrVertex | kVertexAttrSkinned;
		const uint16_t allowed = kVertexAttrVertex | kVertexAttrUV | kVertexAttrNormal | kVertexAttrTangent | kVertexAttrColor | kVertexAttrSkinned;
		if ((out.attributes & required) != required)
			return std::nullopt;
		if ((out.attributes & ~allowed) != 0)
			return std::nullopt;
		if ((out.attributes & kVertexAttrTangent) != 0 && (out.attributes & kVertexAttrNormal) == 0)
			return std::nullopt;
		if (((desc >> 4) & 0x0Fu) != 0)
			return std::nullopt;

		auto expectOffset = [&](uint32_t shift, uint32_t expectedDwords) {
			return static_cast<uint32_t>((desc >> shift) & 0x0Fu) == expectedDwords;
		};

		uint32_t cursorDwords = 4;
		if ((out.attributes & kVertexAttrUV) != 0) {
			if (!expectOffset(8, cursorDwords))
				return std::nullopt;
			cursorDwords += 1;
		} else if (((desc >> 8) & 0x0Fu) != 0) {
			return std::nullopt;
		}

		if ((out.attributes & kVertexAttrNormal) != 0) {
			if (!expectOffset(16, cursorDwords))
				return std::nullopt;
			cursorDwords += 1;
		} else if (((desc >> 16) & 0x0Fu) != 0) {
			return std::nullopt;
		}

		if ((out.attributes & kVertexAttrTangent) != 0) {
			if (!expectOffset(20, cursorDwords))
				return std::nullopt;
			cursorDwords += 1;
		} else if (((desc >> 20) & 0x0Fu) != 0) {
			return std::nullopt;
		}

		if ((out.attributes & kVertexAttrColor) != 0) {
			if (!expectOffset(24, cursorDwords))
				return std::nullopt;
			cursorDwords += 1;
		} else if (((desc >> 24) & 0x0Fu) != 0) {
			return std::nullopt;
		}

		if (!expectOffset(28, cursorDwords))
			return std::nullopt;
		cursorDwords += 3;

		if (out.vertexSize != cursorDwords * 4u)
			return std::nullopt;

		return out;
	}

	/// Read a fixed number of triangle index triplets from the stream.
	static bool readTriangleList(NifReader& r, uint32_t numTriangles, std::vector<std::array<uint16_t, 3>>& out)
	{
		out.clear();
		out.reserve(numTriangles);
		for (uint32_t i = 0; i < numTriangles; ++i)
			out.push_back({ r.readU16(), r.readU16(), r.readU16() });
		return true;
	}

	/// Validate that vertexMap is a full permutation of [0, N).
	/// This guarantees each source vertex maps to one unique destination vertex.
	static bool isPermutationVertexMap(const std::vector<uint16_t>& vertexMap)
	{
		std::vector<uint8_t> seen(vertexMap.size(), 0);
		for (uint32_t i = 0; i < vertexMap.size(); ++i) {
			uint16_t mapped = vertexMap[i];
			if (mapped >= vertexMap.size())
				return false;
			if (seen[mapped] != 0)
				return false;
			seen[mapped] = 1;
		}
		for (uint8_t v : seen) {
			if (v == 0)
				return false;
		}
		return true;
	}

	/// Check that partition vertex bytes match tri-shape vertex bytes after applying vertexMap.
	static bool vertexDataMatchesMappedOrder(
		const std::vector<uint8_t>& triShapeVertexData,
		const std::vector<uint8_t>& partitionVertexData,
		const std::vector<uint16_t>& partitionToShapeVertexMap,
		uint32_t vertexStride)
	{
		if (partitionToShapeVertexMap.empty())
			return triShapeVertexData.empty() && partitionVertexData.empty();
		if (triShapeVertexData.size() != partitionToShapeVertexMap.size() * vertexStride)
			return false;
		if (partitionVertexData.size() != partitionToShapeVertexMap.size() * vertexStride)
			return false;

		for (uint32_t partIdx = 0; partIdx < partitionToShapeVertexMap.size(); ++partIdx) {
			uint16_t shapeIdx = partitionToShapeVertexMap[partIdx];
			const uint8_t* triBytes = triShapeVertexData.data() + static_cast<size_t>(shapeIdx) * vertexStride;
			const uint8_t* partBytes = partitionVertexData.data() + static_cast<size_t>(partIdx) * vertexStride;
			if (std::memcmp(triBytes, partBytes, vertexStride) != 0)
				return false;
		}

		return true;
	}

	/// Remap triangle indices from partition-local vertex indices to tri-shape indices.
	/// Sets ok=false if any index is out of range.
	static std::vector<std::array<uint16_t, 3>> mapTrianglesWithVertexMap(
		const std::vector<std::array<uint16_t, 3>>& triangles,
		const std::vector<uint16_t>& partitionToShapeVertexMap,
		bool& ok)
	{
		ok = true;
		std::vector<std::array<uint16_t, 3>> out;
		out.reserve(triangles.size());
		for (const auto& tri : triangles) {
			if (tri[0] >= partitionToShapeVertexMap.size() || tri[1] >= partitionToShapeVertexMap.size() || tri[2] >= partitionToShapeVertexMap.size()) {
				ok = false;
				return {};
			}
			out.push_back({
				partitionToShapeVertexMap[tri[0]],
				partitionToShapeVertexMap[tri[1]],
				partitionToShapeVertexMap[tri[2]]
			});
		}
		return out;
	}

	/// Compare two triangle lists for exact equality.
	static bool trianglesEqual(const std::vector<std::array<uint16_t, 3>>& a, const std::vector<std::array<uint16_t, 3>>& b)
	{
		return a == b;
	}

	/// Parse a BSTriShape/BSDynamicTriShape block when it matches the supported SSE layout.
	/// Returns nullopt for any unsupported format or size mismatch.
	static std::optional<ParsedSupportedSSETriShape> parseSupportedSSETriShapeBlock(
		const std::vector<uint8_t>& block,
		const std::string& shapeType,
		uint32_t bsVersion)
	{
		if (bsVersion != 100 || (shapeType != "BSTriShape" && shapeType != "BSDynamicTriShape"))
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
			auto layoutOpt = parseSupportedSSEVertexLayout(vertexDesc);
			if (!layoutOpt)
				return std::nullopt;

			ParsedSupportedSSETriShape out;
			out.layout = *layoutOpt;
			out.isDynamic = (shapeType == "BSDynamicTriShape");
			out.prefix.assign(block.begin(), block.begin() + r.pos());
			out.numTriangles = r.readU16();
			out.numVertices = r.readU16();
			uint32_t dataSize = r.readU32();
			const uint32_t expectedDataSize = static_cast<uint32_t>(out.numVertices) * out.layout.vertexSize + static_cast<uint32_t>(out.numTriangles) * 6u;
			if (dataSize != expectedDataSize)
				return std::nullopt;

			out.vertexData = r.readBytes(static_cast<size_t>(out.numVertices) * out.layout.vertexSize);
			if (!readTriangleList(r, out.numTriangles, out.triangles))
				return std::nullopt;

			uint32_t particleDataSize = r.readU32();
			const uint32_t expectedParticleDataSize = static_cast<uint32_t>(out.numVertices) * 12u + static_cast<uint32_t>(out.numTriangles) * 6u;
			if (particleDataSize != expectedParticleDataSize)
				return std::nullopt;

			r.skip(static_cast<size_t>(out.numVertices) * 6);
			out.particleNormals.resize(out.numVertices);
			for (uint16_t i = 0; i < out.numVertices; ++i) {
				auto bytes = r.readBytes(6);
				std::copy(bytes.begin(), bytes.end(), out.particleNormals[i].begin());
			}

			std::vector<std::array<uint16_t, 3>> particleTriangles;
			if (!readTriangleList(r, out.numTriangles, particleTriangles))
				return std::nullopt;
			if (!trianglesEqual(out.triangles, particleTriangles))
				return std::nullopt;

			if (out.isDynamic) {
				uint32_t dynamicDataSize = r.readU32();
				const uint32_t expectedDynamicDataSize = static_cast<uint32_t>(out.numVertices) * 16u;
				if (dynamicDataSize != expectedDynamicDataSize)
					return std::nullopt;
				out.dynamicVertexData = r.readBytes(dynamicDataSize);
			}
			if (r.pos() != block.size())
				return std::nullopt;

			return out;
		} catch (...) {
			return std::nullopt;
		}
	}

	/// Parse a NiSkinPartition block when it matches the supported SSE single-partition format.
	/// Returns nullopt for any unsupported field combination or validation failure.
	static std::optional<ParsedSupportedSSESkinPartition> parseSupportedSSESkinPartitionBlock(
		const std::vector<uint8_t>& block,
		uint32_t bsVersion)
	{
		if (bsVersion != 100)
			return std::nullopt;

		try {
			NifReader r(block);
			uint32_t numPartitions = r.readU32();
			if (numPartitions != 1)
				return std::nullopt;
			uint32_t dataSize = r.readU32();
			uint32_t vertexSize = r.readU32();
			uint64_t vertexDesc = r.readU64();
			auto layoutOpt = parseSupportedSSEVertexLayout(vertexDesc);
			if (!layoutOpt || layoutOpt->vertexSize != vertexSize)
				return std::nullopt;
			if ((dataSize % vertexSize) != 0)
				return std::nullopt;

			ParsedSupportedSSESkinPartition out;
			out.layout = *layoutOpt;
			out.vertexData = r.readBytes(dataSize);
			out.numVertices = r.readU16();
			out.numTriangles = r.readU16();
			out.numBones = r.readU16();
			uint16_t numStrips = r.readU16();
			out.numWeightsPerVertex = r.readU16();
			if (numStrips != 0 || out.numWeightsPerVertex != 4)
				return std::nullopt;
			if (out.vertexData.size() != static_cast<size_t>(out.numVertices) * out.layout.vertexSize)
				return std::nullopt;

			out.bones.reserve(out.numBones);
			for (uint16_t i = 0; i < out.numBones; ++i)
				out.bones.push_back(r.readU16());

			bool hasVertexMap = false;
			if (!readBoolStrict(r, hasVertexMap) || !hasVertexMap)
				return std::nullopt;
			out.vertexMap.reserve(out.numVertices);
			for (uint16_t i = 0; i < out.numVertices; ++i)
				out.vertexMap.push_back(r.readU16());

			bool hasVertexWeights = false;
			if (!readBoolStrict(r, hasVertexWeights) || !hasVertexWeights)
				return std::nullopt;
			out.vertexWeights.reserve(static_cast<size_t>(out.numVertices) * out.numWeightsPerVertex);
			for (uint32_t i = 0; i < static_cast<uint32_t>(out.numVertices) * out.numWeightsPerVertex; ++i)
				out.vertexWeights.push_back(r.readF32());

			bool hasFaces = false;
			if (!readBoolStrict(r, hasFaces) || !hasFaces)
				return std::nullopt;
			if (!readTriangleList(r, out.numTriangles, out.triangles))
				return std::nullopt;

			bool hasBoneIndices = false;
			if (!readBoolStrict(r, hasBoneIndices) || !hasBoneIndices)
				return std::nullopt;
			out.boneIndices = r.readBytes(static_cast<size_t>(out.numVertices) * out.numWeightsPerVertex);

			out.lodLevel = r.readU8();
			if (!readBoolStrict(r, out.globalVB) || out.globalVB)
				return std::nullopt;

			uint64_t partitionDesc = r.readU64();
			if (partitionDesc != vertexDesc)
				return std::nullopt;
			if (!readTriangleList(r, out.numTriangles, out.trianglesCopy))
				return std::nullopt;
			if (!trianglesEqual(out.triangles, out.trianglesCopy))
				return std::nullopt;
			if (r.pos() != block.size())
				return std::nullopt;

			return out;
		} catch (...) {
			return std::nullopt;
		}
	}

	/// Build decimator input vertices from skin partition data (position, weights, and bone ids).
	static std::vector<Vertex> extractSupportedPartitionVertices(const ParsedSupportedSSESkinPartition& partition)
	{
		std::vector<Vertex> out;
		out.reserve(partition.numVertices);
		for (uint16_t i = 0; i < partition.numVertices; ++i) {
			Vertex v;
			const size_t base = static_cast<size_t>(i) * partition.layout.vertexSize;
			float px = 0.0f;
			float py = 0.0f;
			float pz = 0.0f;
			std::memcpy(&px, partition.vertexData.data() + base + 0, 4);
			std::memcpy(&py, partition.vertexData.data() + base + 4, 4);
			std::memcpy(&pz, partition.vertexData.data() + base + 8, 4);
			v.m_skinPos.setValue(px, py, pz);

			const size_t skinBase = base + partition.layout.skinOffset * 4u;
			for (uint32_t k = 0; k < 4; ++k) {
				uint16_t half = 0;
				std::memcpy(&half, partition.vertexData.data() + skinBase + k * 2, 2);
				v.m_weight[k] = partition.vertexWeights[static_cast<size_t>(i) * 4 + k];
				v.setBoneIdx(static_cast<int>(k), partition.boneIndices[static_cast<size_t>(i) * 4 + k]);
				if (std::fabs(v.m_weight[k] - halfToFloat(half)) > 0.01f)
					v.m_weight[k] = halfToFloat(half);
			}
			v.sortWeight();
			out.push_back(v);
		}
		return out;
	}

	/// Build a representative old vertex index for each new decimated vertex.
	/// Returns an empty vector if the old-to-new map is invalid or incomplete.
	static std::vector<uint32_t> buildRepresentativeOldVertices(const std::vector<uint32_t>& oldToNew, uint32_t newVertexCount)
	{
		std::vector<uint32_t> rep(newVertexCount, std::numeric_limits<uint32_t>::max());
		for (uint32_t oldIdx = 0; oldIdx < oldToNew.size(); ++oldIdx) {
			uint32_t newIdx = oldToNew[oldIdx];
			if (newIdx >= newVertexCount)
				return {};
			if (rep[newIdx] == std::numeric_limits<uint32_t>::max())
				rep[newIdx] = oldIdx;
		}
		for (uint32_t i = 0; i < rep.size(); ++i) {
			if (rep[i] == std::numeric_limits<uint32_t>::max())
				return {};
		}
		return rep;
	}

	/// Overwrite position and packed skin data for one vertex inside a serialized vertex buffer.
	static void overwritePositionAndSkin(
		std::vector<uint8_t>& vertexBytes,
		const SupportedSSEVertexLayout& layout,
		uint32_t vertexIndex,
		const Vertex& v)
	{
		const size_t base = static_cast<size_t>(vertexIndex) * layout.vertexSize;
		float px = v.m_skinPos.x();
		float py = v.m_skinPos.y();
		float pz = v.m_skinPos.z();
		std::memcpy(vertexBytes.data() + base + 0, &px, 4);
		std::memcpy(vertexBytes.data() + base + 4, &py, 4);
		std::memcpy(vertexBytes.data() + base + 8, &pz, 4);

		const size_t skinBase = base + layout.skinOffset * 4u;
		for (uint32_t k = 0; k < 4; ++k) {
			uint16_t half = floatToHalf(v.m_weight[k]);
			std::memcpy(vertexBytes.data() + skinBase + k * 2, &half, 2);
			vertexBytes[skinBase + 8 + k] = static_cast<uint8_t>(v.getBoneIdx(static_cast<int>(k)));
		}
	}

	/// Rebuild a supported tri-shape block using decimated vertices and triangles.
	/// Preserves stable per-vertex payload by copying from representative source vertices.
	static std::vector<uint8_t> buildSupportedSSETriShapeBlock(
		const ParsedSupportedSSETriShape& triShape,
		const CollisionMeshDecimationOutput& output,
		const std::vector<uint32_t>& representativeOldPartitionVertices,
		const std::vector<uint16_t>& oldPartitionToShapeVertexMap)
	{
		std::vector<uint8_t> vertexData(static_cast<size_t>(output.vertices.size()) * triShape.layout.vertexSize);
		for (uint32_t newIdx = 0; newIdx < output.vertices.size(); ++newIdx) {
			uint32_t oldPartIdx = representativeOldPartitionVertices[newIdx];
			uint32_t oldShapeIdx = oldPartitionToShapeVertexMap[oldPartIdx];
			std::memcpy(
				vertexData.data() + static_cast<size_t>(newIdx) * triShape.layout.vertexSize,
				triShape.vertexData.data() + static_cast<size_t>(oldShapeIdx) * triShape.layout.vertexSize,
				triShape.layout.vertexSize);
			overwritePositionAndSkin(vertexData, triShape.layout, newIdx, output.vertices[newIdx]);
		}

		std::vector<uint8_t> out;
		out.reserve(
			triShape.prefix.size() + 8 + vertexData.size() + output.triangles.size() * 6 + 4 +
			output.vertices.size() * 12 + output.triangles.size() * 6 + (triShape.isDynamic ? (4 + output.vertices.size() * 16) : 0));
		out.insert(out.end(), triShape.prefix.begin(), triShape.prefix.end());
		appendU16(out, static_cast<uint16_t>(output.triangles.size()));
		appendU16(out, static_cast<uint16_t>(output.vertices.size()));
		appendU32(out, static_cast<uint32_t>(vertexData.size() + output.triangles.size() * 6));
		out.insert(out.end(), vertexData.begin(), vertexData.end());
		writeTriangleArray(out, output.triangles);
		appendU32(out, static_cast<uint32_t>(output.vertices.size() * 12 + output.triangles.size() * 6));
		for (uint32_t newIdx = 0; newIdx < output.vertices.size(); ++newIdx)
			writeHalfVec3(out, output.vertices[newIdx].m_skinPos);
		for (uint32_t newIdx = 0; newIdx < output.vertices.size(); ++newIdx) {
			uint32_t oldPartIdx = representativeOldPartitionVertices[newIdx];
			uint32_t oldShapeIdx = oldPartitionToShapeVertexMap[oldPartIdx];
			out.insert(out.end(), triShape.particleNormals[oldShapeIdx].begin(), triShape.particleNormals[oldShapeIdx].end());
		}
		writeTriangleArray(out, output.triangles);

		if (triShape.isDynamic) {
			appendU32(out, static_cast<uint32_t>(output.vertices.size() * 16));
			for (uint32_t newIdx = 0; newIdx < output.vertices.size(); ++newIdx) {
				uint32_t oldPartIdx = representativeOldPartitionVertices[newIdx];
				uint32_t oldShapeIdx = oldPartitionToShapeVertexMap[oldPartIdx];
				const size_t srcBase = static_cast<size_t>(oldShapeIdx) * 16;
				float px = output.vertices[newIdx].m_skinPos.x();
				float py = output.vertices[newIdx].m_skinPos.y();
				float pz = output.vertices[newIdx].m_skinPos.z();
				out.insert(out.end(), triShape.dynamicVertexData.begin() + srcBase, triShape.dynamicVertexData.begin() + srcBase + 16);
				std::memcpy(out.data() + out.size() - 16 + 0, &px, 4);
				std::memcpy(out.data() + out.size() - 16 + 4, &py, 4);
				std::memcpy(out.data() + out.size() - 16 + 8, &pz, 4);
			}
		}

		return out;
	}

	/// Rebuild a supported skin partition block using decimated mesh output.
	static std::vector<uint8_t> buildSupportedSSESkinPartitionBlock(
		const ParsedSupportedSSESkinPartition& partition,
		const CollisionMeshDecimationOutput& output,
		const std::vector<uint32_t>& representativeOldVertices)
	{
		std::vector<uint8_t> vertexData(static_cast<size_t>(output.vertices.size()) * partition.layout.vertexSize);
		for (uint32_t newIdx = 0; newIdx < output.vertices.size(); ++newIdx) {
			uint32_t oldIdx = representativeOldVertices[newIdx];
			std::memcpy(
				vertexData.data() + static_cast<size_t>(newIdx) * partition.layout.vertexSize,
				partition.vertexData.data() + static_cast<size_t>(oldIdx) * partition.layout.vertexSize,
				partition.layout.vertexSize);
			overwritePositionAndSkin(vertexData, partition.layout, newIdx, output.vertices[newIdx]);
		}

		std::vector<uint8_t> out;
		out.reserve(64 + vertexData.size() + output.vertices.size() * 32 + output.triangles.size() * 12);
		appendU32(out, 1);
		appendU32(out, static_cast<uint32_t>(vertexData.size()));
		appendU32(out, partition.layout.vertexSize);
		appendU64(out, partition.layout.desc);
		out.insert(out.end(), vertexData.begin(), vertexData.end());

		appendU16(out, static_cast<uint16_t>(output.vertices.size()));
		appendU16(out, static_cast<uint16_t>(output.triangles.size()));
		appendU16(out, partition.numBones);
		appendU16(out, 0);
		appendU16(out, 4);
		for (uint16_t bone : partition.bones)
			appendU16(out, bone);

		appendU8(out, 1);
		for (uint32_t i = 0; i < output.vertices.size(); ++i)
			appendU16(out, static_cast<uint16_t>(i));

		appendU8(out, 1);
		for (const auto& v : output.vertices) {
			for (uint32_t k = 0; k < 4; ++k)
				appendF32(out, v.m_weight[k]);
		}

		appendU8(out, 1);
		writeTriangleArray(out, output.triangles);

		appendU8(out, 1);
		for (const auto& v : output.vertices) {
			for (uint32_t k = 0; k < 4; ++k)
				appendU8(out, static_cast<uint8_t>(v.getBoneIdx(static_cast<int>(k))));
		}

		appendU8(out, partition.lodLevel);
		appendU8(out, 0);
		appendU64(out, partition.layout.desc);
		writeTriangleArray(out, output.triangles);
		return out;
	}

	/// Convert validator-level NIF decimation options into decimator engine options.
	static CollisionMeshDecimationOptions makeCollisionDecimationOptions(const NIFDecimationOptions& o)
	{
		CollisionMeshDecimationOptions out;
		out.enabled = o.enableCollisionMeshDecimation;
		out.preserveBoundary = o.preserveBoundary;
		out.preserveFeatures = o.preserveFeatures;
		out.targetVertexCount = o.targetVertexCount;
		out.targetVertexRatio = o.targetVertexRatio;
		out.qemCostThreshold = o.qemCostThreshold;
		out.shortEdgeRatio = o.shortEdgeRatio;
		out.skinWeightPenalty = o.skinWeightPenalty;
		out.maxSkinWeightDrift = o.maxSkinWeightDrift;
		out.maxVolumeLossPercent = o.maxVolumeLossPercent;
		out.maxNormalDeviationDegrees = o.maxNormalDeviationDegrees;
		out.maxLocalVolumeChangePercent = o.maxLocalVolumeChangePercent;
		out.maxPointRemovals = o.maxPointRemovals;
		out.maxEdgeCollapses = o.maxEdgeCollapses;
		return out;
	}

	/// Build outbound reference sets for each block in the parsed NIF.
	static std::vector<std::unordered_set<int32_t>> buildOutboundRefs(const ParsedNif& parsed)
	{
		const int32_t numBlocks = static_cast<int32_t>(parsed.blocks.size());
		std::vector<std::unordered_set<int32_t>> outbound(parsed.blocks.size());
		for (size_t i = 0; i < parsed.blocks.size(); ++i)
			outbound[i] = collectPotentialRefs(parsed.blocks[i], numBlocks);
		return outbound;
	}

	/// Safely get a block type name by block index.
	/// Returns nullopt if the index is out of range or malformed.
	static std::optional<std::string> blockTypeAt(const ParsedNif& parsed, int idx)
	{
		if (idx < 0 || idx >= static_cast<int>(parsed.blockTypeIndex.size()))
			return std::nullopt;
		uint16_t tIdx = parsed.blockTypeIndex[static_cast<size_t>(idx)];
		if (tIdx >= parsed.blockTypes.size())
			return std::nullopt;
		return parsed.blockTypes[tIdx];
	}

	/// Discover tri-shape candidates and their linked skin-instance/skin-partition blocks.
	static std::vector<ShapeDecimationCandidate> discoverDecimationCandidates(const ParsedNif& parsed)
	{
		auto outbound = buildOutboundRefs(parsed);
		std::vector<ShapeDecimationCandidate> out;

		for (int i = 0; i < static_cast<int>(parsed.blocks.size()); ++i) {
			auto typeOpt = blockTypeAt(parsed, i);
			if (!typeOpt)
				continue;

			const std::string& typeName = *typeOpt;
			if (typeName != "BSTriShape" && typeName != "BSDynamicTriShape")
				continue;

			ShapeDecimationCandidate c;
			c.triShapeBlockIndex = i;
			c.shapeType = typeName;

			for (int32_t ref : outbound[static_cast<size_t>(i)]) {
				auto refType = blockTypeAt(parsed, ref);
				if (!refType)
					continue;
				if (hasTypeName(*refType, { "NiSkinInstance", "BSDismemberSkinInstance", "BSSkin::Instance" })) {
					if (c.skinInstanceBlockIndex == -1)
						c.skinInstanceBlockIndex = ref;
				}
			}

			if (c.skinInstanceBlockIndex >= 0) {
				for (int32_t ref : outbound[static_cast<size_t>(c.skinInstanceBlockIndex)]) {
					auto refType = blockTypeAt(parsed, ref);
					if (!refType)
						continue;
					if (*refType == "NiSkinPartition") {
						if (c.skinPartitionBlockIndex == -1)
							c.skinPartitionBlockIndex = ref;
					}
				}
			}

			out.push_back(std::move(c));
		}

		return out;
	}

	/// Try to decimate one candidate using strict safety checks.
	/// This function is fail-closed: any unsupported or inconsistent state is skipped.
	static ShapeDecimationResult decimateCandidateFailClosed(
		ParsedNif& parsed,
		const ShapeDecimationCandidate& c,
		const CollisionMeshDecimationOptions& options)
	{
		ShapeDecimationResult r;
		r.candidate = c;

		if (!options.enabled) {
			r.decision = DecimationApplyDecision::SkipNoChange;
			r.reason = "decimation-disabled";
			return r;
		}

		if (c.skinInstanceBlockIndex < 0) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "missing-skin-instance-link";
			return r;
		}

		if (c.skinPartitionBlockIndex < 0) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "missing-skin-partition-link";
			return r;
		}

		auto skinInstanceType = blockTypeAt(parsed, c.skinInstanceBlockIndex);
		if (!skinInstanceType || !hasTypeName(*skinInstanceType, { "NiSkinInstance", "BSDismemberSkinInstance" })) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "unsupported-skin-instance-type";
			return r;
		}

		auto triShapeOpt = parseSupportedSSETriShapeBlock(parsed.blocks[static_cast<size_t>(c.triShapeBlockIndex)], c.shapeType, parsed.bsVersion);
		if (!triShapeOpt) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "unsupported-trishape-layout";
			return r;
		}

		auto partitionOpt = parseSupportedSSESkinPartitionBlock(parsed.blocks[static_cast<size_t>(c.skinPartitionBlockIndex)], parsed.bsVersion);
		if (!partitionOpt) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "unsupported-skin-partition-layout";
			return r;
		}

		const auto& triShape = *triShapeOpt;
		const auto& partition = *partitionOpt;
		if (triShape.layout.desc != partition.layout.desc || triShape.numVertices != partition.numVertices || triShape.numTriangles != partition.numTriangles) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "shape-partition-count-mismatch";
			return r;
		}

		if (!isPermutationVertexMap(partition.vertexMap)) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "unsupported-non-permutation-vertex-map";
			return r;
		}

		if (!vertexDataMatchesMappedOrder(triShape.vertexData, partition.vertexData, partition.vertexMap, triShape.layout.vertexSize)) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "shape-partition-vertexdata-mismatch";
			return r;
		}

		if (!trianglesEqual(partition.triangles, partition.trianglesCopy)) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "partition-triangle-copy-mismatch";
			return r;
		}

		bool triangleMapOK = false;
		auto mappedPartitionTriangles = mapTrianglesWithVertexMap(partition.triangles, partition.vertexMap, triangleMapOK);
		if (!triangleMapOK || !trianglesEqual(triShape.triangles, mappedPartitionTriangles)) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "shape-partition-triangle-mismatch";
			return r;
		}

		auto inputVertices = extractSupportedPartitionVertices(partition);
		std::vector<std::array<uint32_t, 3>> inputTriangles;
		inputTriangles.reserve(partition.triangles.size());
		for (const auto& tri : partition.triangles) {
			if (tri[0] >= partition.numVertices || tri[1] >= partition.numVertices || tri[2] >= partition.numVertices) {
				r.decision = DecimationApplyDecision::SkipUnsafe;
				r.reason = "triangle-index-out-of-range";
				return r;
			}
			inputTriangles.push_back({ tri[0], tri[1], tri[2] });
		}

		auto output = DecimateCollisionMesh(inputVertices, inputTriangles, options);
		r.stats = output.stats;
		if (output.vertices.empty() || output.triangles.empty()) {
			r.decision = DecimationApplyDecision::SkipNoChange;
			r.reason = "decimator-produced-empty-mesh";
			return r;
		}
		if (output.vertices.size() > std::numeric_limits<uint16_t>::max()) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "decimated-vertex-count-overflow";
			return r;
		}
		for (const auto& v : output.vertices) {
			for (int k = 0; k < 4; ++k) {
				if (v.getBoneIdx(k) >= partition.numBones) {
					r.decision = DecimationApplyDecision::SkipUnsafe;
					r.reason = "decimated-bone-index-out-of-range";
					return r;
				}
			}
		}
		if (output.stats.outputVertexCount == output.stats.originalVertexCount && output.stats.outputTriangleCount == output.stats.originalTriangleCount) {
			r.decision = DecimationApplyDecision::SkipNoChange;
			r.reason = "decimator-produced-no-reduction";
			return r;
		}
		if (output.oldToNewVertex.size() != inputVertices.size()) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "decimator-mapping-size-mismatch";
			return r;
		}

		auto representativeOldVertices = buildRepresentativeOldVertices(output.oldToNewVertex, static_cast<uint32_t>(output.vertices.size()));
		if (representativeOldVertices.empty()) {
			r.decision = DecimationApplyDecision::SkipUnsafe;
			r.reason = "decimator-representative-map-failed";
			return r;
		}

		parsed.blocks[static_cast<size_t>(c.triShapeBlockIndex)] = buildSupportedSSETriShapeBlock(triShape, output, representativeOldVertices, partition.vertexMap);
		parsed.blocks[static_cast<size_t>(c.skinPartitionBlockIndex)] = buildSupportedSSESkinPartitionBlock(partition, output, representativeOldVertices);
		r.decision = DecimationApplyDecision::Apply;
		r.reason = "applied-supported-sse-single-partition";
		return r;
	}

	/// Run offline collision decimation across all discovered candidates.
	/// Aggregates outcomes and skip reasons, and marks changed=true when any rewrite is applied.
	DecimationBridgeDiagnostics runOfflineCollisionDecimationBridge(
		ParsedNif& parsed,
		const NIFDecimationOptions& options,
		bool& changed)
	{
		DecimationBridgeDiagnostics diag;
		auto decimOptions = makeCollisionDecimationOptions(options);
		auto candidates = discoverDecimationCandidates(parsed);
		diag.candidatesDiscovered = static_cast<int>(candidates.size());

		std::map<std::string, int> reasonCounts;
		for (const auto& c : candidates) {
			++diag.candidatesAttempted;
			auto result = decimateCandidateFailClosed(parsed, c, decimOptions);
			++reasonCounts[result.reason];
			switch (result.decision) {
			case DecimationApplyDecision::Apply:
				++diag.candidatesApplied;
				changed = true;
				break;
			case DecimationApplyDecision::SkipNoChange:
				++diag.candidatesSkippedNoChange;
				break;
			case DecimationApplyDecision::SkipUnsafe:
				++diag.candidatesSkippedUnsafe;
				break;
			}
		}

		diag.skipReasons.reserve(reasonCounts.size());
		for (const auto& [reason, count] : reasonCounts)
			diag.skipReasons.push_back({ reason, count });

		return diag;
	}
}