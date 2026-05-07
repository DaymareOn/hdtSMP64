#pragma once

#include "hdtSkinnedMesh/hdtVertex.h"

#include <array>
#include <string>
#include <vector>

namespace hdt
{
	struct CollisionMeshDecimationOptions
	{
		bool enabled = false;
		bool preserveBoundary = true;
		bool preserveFeatures = true;
		int targetVertexCount = 0;
		float targetVertexRatio = 1.0f;  // fraction of vertices to retain (0.99 => ~1% reduction)
		float qemCostThreshold = 1.0f;
		float shortEdgeRatio = 0.01f;
		float maxVolumeLossPercent = 1.0f;
		float maxNormalDeviationDegrees = 25.0f;
		float maxLocalVolumeChangePercent = 1.0f;
		int maxPointRemovals = 256;
		int maxEdgeCollapses = 4096;
	};

	struct CollisionMeshDecimationStats
	{
		int originalVertexCount = 0;
		int originalTriangleCount = 0;
		int outputVertexCount = 0;
		int outputTriangleCount = 0;
		int pointRemovals = 0;
		int edgeCollapses = 0;
		int rejectedCollapses = 0;
		int rejectedNormal = 0;
		int rejectedVolume = 0;
		int rejectedBoundary = 0;
		int rejectedFeature = 0;
		int rejectedDegenerate = 0;
		float initialSignedVolume = 0.0f;
		float outputSignedVolume = 0.0f;
		float volumeDeltaPercent = 0.0f;
		bool usedFallback = false;
		std::string fallbackReason;
	};

	struct CollisionMeshDecimationOutput
	{
		std::vector<Vertex> vertices;
		std::vector<std::array<uint32_t, 3>> triangles;
		std::vector<uint32_t> oldToNewVertex;
		CollisionMeshDecimationStats stats;
	};

	[[nodiscard]] CollisionMeshDecimationOutput DecimateCollisionMesh(
		const std::vector<Vertex>& inputVertices,
		const std::vector<std::array<uint32_t, 3>>& inputTriangles,
		const CollisionMeshDecimationOptions& options);
}
