#include "hdtCollisionMeshDecimator.h"

#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

namespace hdt
{
	namespace
	{
		static constexpr float kMatrixSingularityEpsilon = 1e-9f;
		static constexpr float kDegenerateAreaThreshold = 1e-7f;

		struct EdgeKey
		{
			uint32_t a = 0;
			uint32_t b = 0;

			EdgeKey() = default;

			EdgeKey(uint32_t v0, uint32_t v1)
			{
				if (v0 < v1) {
					a = v0;
					b = v1;
				} else {
					a = v1;
					b = v0;
				}
			}

			bool operator==(const EdgeKey& rhs) const { return a == rhs.a && b == rhs.b; }
		};

		struct EdgeKeyHash
		{
			size_t operator()(const EdgeKey& e) const
			{
				uint64_t packed = (static_cast<uint64_t>(e.a) << 32) | static_cast<uint64_t>(e.b);
				return std::hash<uint64_t>{}(packed);
			}
		};

		struct Triangle
		{
			uint32_t v[3] = { 0, 0, 0 };
			bool alive = true;
		};

		struct Quadric
		{
			float m[10] = { 0 };

			void addPlane(float a, float b, float c, float d)
			{
				m[0] += a * a;
				m[1] += a * b;
				m[2] += a * c;
				m[3] += a * d;
				m[4] += b * b;
				m[5] += b * c;
				m[6] += b * d;
				m[7] += c * c;
				m[8] += c * d;
				m[9] += d * d;
			}

			void add(const Quadric& rhs)
			{
				for (int i = 0; i < 10; ++i)
					m[i] += rhs.m[i];
			}

			float evaluate(const btVector3& p) const
			{
				float x = p.x(), y = p.y(), z = p.z();
				return m[0] * x * x + 2.0f * m[1] * x * y + 2.0f * m[2] * x * z + 2.0f * m[3] * x +
				       m[4] * y * y + 2.0f * m[5] * y * z + 2.0f * m[6] * y + m[7] * z * z + 2.0f * m[8] * z + m[9];
			}
		};

		struct WorkingVertex
		{
			Vertex skin;
			btVector3 pos = btVector3(0, 0, 0);
			bool alive = true;
			bool boundary = false;
			bool feature = false;
			Quadric quadric;
		};

		static btVector3 toVec3(const Vertex& v)
		{
			return btVector3(v.m_skinPos.x(), v.m_skinPos.y(), v.m_skinPos.z());
		}

		static float triDoubleArea(const btVector3& a, const btVector3& b, const btVector3& c)
		{
			return (b - a).cross(c - a).length();
		}

		static btVector3 triNormal(const btVector3& a, const btVector3& b, const btVector3& c)
		{
			btVector3 n = (b - a).cross(c - a);
			float len = n.length();
			if (len < FLT_EPSILON)
				return btVector3(0, 0, 0);
			return n / len;
		}

		static float clamp01(float v)
		{
			return std::max(0.0f, std::min(1.0f, v));
		}

		static float signedVolume(const std::vector<WorkingVertex>& vertices, const std::vector<Triangle>& triangles)
		{
			double vol = 0.0;
			for (const auto& tri : triangles) {
				if (!tri.alive)
					continue;
				const auto& a = vertices[tri.v[0]].pos;
				const auto& b = vertices[tri.v[1]].pos;
				const auto& c = vertices[tri.v[2]].pos;
				vol += static_cast<double>(a.dot(b.cross(c)));
			}
			return static_cast<float>(vol / 6.0);
		}

		static float signedVolumeContribution(const btVector3& a, const btVector3& b, const btVector3& c)
		{
			return a.dot(b.cross(c)) / 6.0f;
		}

		static void normalizeTop4(Vertex& v)
		{
			float sum = 0.0f;
			for (int i = 0; i < 4; ++i) {
				if (v.m_weight[i] < 0.0f)
					v.m_weight[i] = 0.0f;
				sum += v.m_weight[i];
			}

			if (sum < FLT_EPSILON) {
				v.m_weight[0] = 1.0f;
				v.m_weight[1] = v.m_weight[2] = v.m_weight[3] = 0.0f;
				return;
			}

			float inv = 1.0f / sum;
			for (int i = 0; i < 4; ++i)
				v.m_weight[i] *= inv;
			v.sortWeight();
		}

		static float collapseFactor(const btVector3& keepPos, const btVector3& removePos, const btVector3& newPos)
		{
			btVector3 edge = removePos - keepPos;
			float len2 = edge.length2();
			if (len2 < FLT_EPSILON)
				return 0.5f;
			return clamp01((newPos - keepPos).dot(edge) / len2);
		}

		static float skinDistanceL1(const Vertex& a, const Vertex& b)
		{
			std::unordered_map<uint32_t, float> delta;
			delta.reserve(8);

			for (int i = 0; i < 4; ++i) {
				if (a.m_weight[i] > FLT_EPSILON)
					delta[a.m_boneIdx[i]] += a.m_weight[i];
			}
			for (int i = 0; i < 4; ++i) {
				if (b.m_weight[i] > FLT_EPSILON)
					delta[b.m_boneIdx[i]] -= b.m_weight[i];
			}

			float l1 = 0.0f;
			for (const auto& [_, v] : delta)
				l1 += std::fabs(v);
			return l1;
		}

		static void mergeSkin(Vertex& keep, const Vertex& remove, float t)
		{
			t = clamp01(t);
			float kKeep = 1.0f - t;
			float kRemove = t;

			std::unordered_map<uint32_t, float> accum;
			for (int i = 0; i < 4; ++i) {
				if (keep.m_weight[i] > FLT_EPSILON)
					accum[keep.m_boneIdx[i]] += keep.m_weight[i] * kKeep;
			}
			for (int i = 0; i < 4; ++i) {
				if (remove.m_weight[i] > FLT_EPSILON)
					accum[remove.m_boneIdx[i]] += remove.m_weight[i] * kRemove;
			}

			std::vector<std::pair<uint32_t, float>> ordered;
			ordered.reserve(accum.size());
			for (const auto& it : accum) {
				ordered.push_back(it);
			}
			std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) {
				if (a.second == b.second)
					return a.first < b.first;
				return a.second > b.second;
			});

			for (int i = 0; i < 4; ++i) {
				if (static_cast<size_t>(i) < ordered.size()) {
					keep.m_boneIdx[i] = ordered[i].first;
					keep.m_weight[i] = ordered[i].second;
				} else {
					keep.m_boneIdx[i] = 0;
					keep.m_weight[i] = 0.0f;
				}
			}

			normalizeTop4(keep);
		}

		static Vertex predictedMergedSkin(const Vertex& keep, const Vertex& remove, float t)
		{
			Vertex merged = keep;
			mergeSkin(merged, remove, t);
			return merged;
		}

		static bool solve3x3(const float A[9], const float b[3], btVector3& out)
		{
			float m[12] = {
				A[0], A[1], A[2], b[0],
				A[3], A[4], A[5], b[1],
				A[6], A[7], A[8], b[2]
			};

			for (int col = 0; col < 3; ++col) {
				int pivot = col;
				float maxAbs = std::fabs(m[col * 4 + col]);
				for (int r = col + 1; r < 3; ++r) {
					float v = std::fabs(m[r * 4 + col]);
					if (v > maxAbs) {
						maxAbs = v;
						pivot = r;
					}
				}
				if (maxAbs < kMatrixSingularityEpsilon)
					return false;
				if (pivot != col) {
					for (int c = col; c < 4; ++c)
						std::swap(m[col * 4 + c], m[pivot * 4 + c]);
				}
				float inv = 1.0f / m[col * 4 + col];
				for (int c = col; c < 4; ++c)
					m[col * 4 + c] *= inv;
				for (int r = 0; r < 3; ++r) {
					if (r == col)
						continue;
					float f = m[r * 4 + col];
					for (int c = col; c < 4; ++c)
						m[r * 4 + c] -= f * m[col * 4 + c];
				}
			}

			out.setValue(m[3], m[7], m[11]);
			return true;
		}

		static btVector3 optimalPlacement(const Quadric& q, const btVector3& fallbackMid)
		{
			float A[9] = {
				q.m[0], q.m[1], q.m[2],
				q.m[1], q.m[4], q.m[5],
				q.m[2], q.m[5], q.m[7]
			};
			float b[3] = { -q.m[3], -q.m[6], -q.m[8] };

			btVector3 out;
			if (solve3x3(A, b, out))
				return out;
			return fallbackMid;
		}

		static void recomputeQuadrics(std::vector<WorkingVertex>& vertices, const std::vector<Triangle>& triangles)
		{
			for (auto& v : vertices)
				v.quadric = {};

			for (const auto& tri : triangles) {
				if (!tri.alive)
					continue;
				const btVector3& p0 = vertices[tri.v[0]].pos;
				const btVector3& p1 = vertices[tri.v[1]].pos;
				const btVector3& p2 = vertices[tri.v[2]].pos;
				btVector3 n = triNormal(p0, p1, p2);
				if (n.fuzzyZero())
					continue;
				float d = -n.dot(p0);
				vertices[tri.v[0]].quadric.addPlane(n.x(), n.y(), n.z(), d);
				vertices[tri.v[1]].quadric.addPlane(n.x(), n.y(), n.z(), d);
				vertices[tri.v[2]].quadric.addPlane(n.x(), n.y(), n.z(), d);
			}
		}

		struct EdgeInfo
		{
			EdgeKey key;
			int count = 0;
			btVector3 n0 = btVector3(0, 0, 0);
			btVector3 n1 = btVector3(0, 0, 0);
			bool hasN0 = false;
			bool hasN1 = false;
		};

		static void recomputeEdgeFlags(
			std::vector<WorkingVertex>& vertices,
			const std::vector<Triangle>& triangles,
			const CollisionMeshDecimationOptions& options)
		{
			std::unordered_map<EdgeKey, EdgeInfo, EdgeKeyHash> edges;
			edges.reserve(triangles.size() * 3);

			for (const auto& tri : triangles) {
				if (!tri.alive)
					continue;
				btVector3 p[3] = {
					vertices[tri.v[0]].pos,
					vertices[tri.v[1]].pos,
					vertices[tri.v[2]].pos
				};
				btVector3 n = triNormal(p[0], p[1], p[2]);
				for (int i = 0; i < 3; ++i) {
					uint32_t a = tri.v[i];
					uint32_t b = tri.v[(i + 1) % 3];
					EdgeKey k(a, b);
					auto& info = edges[k];
					info.key = k;
					++info.count;
					if (!info.hasN0) {
						info.n0 = n;
						info.hasN0 = true;
					} else if (!info.hasN1) {
						info.n1 = n;
						info.hasN1 = true;
					}
				}
			}

			for (auto& v : vertices) {
				if (v.alive) {
					v.boundary = false;
					v.feature = false;
				}
			}

			for (const auto& [_, e] : edges) {
				if (e.count == 1) {
					vertices[e.key.a].boundary = true;
					vertices[e.key.b].boundary = true;
				}

				if (options.preserveFeatures && e.hasN0 && e.hasN1 && !e.n0.fuzzyZero() && !e.n1.fuzzyZero()) {
					float dotv = clamp01(std::fabs(e.n0.dot(e.n1)));
					float angle = std::acos(dotv) * (180.0f / SIMD_PI);
					if (angle > options.maxNormalDeviationDegrees) {
						vertices[e.key.a].feature = true;
						vertices[e.key.b].feature = true;
					}
				}
			}
		}

		struct Candidate
		{
			uint32_t keep = 0;
			uint32_t remove = 0;
			btVector3 newPos = btVector3(0, 0, 0);
			float cost = std::numeric_limits<float>::max();
			float skinDrift = 0.0f;
		};

		static bool gatherIncidentTriangles(
			uint32_t vKeep,
			uint32_t vRemove,
			const std::vector<Triangle>& triangles,
			std::vector<int>& incident)
		{
			incident.clear();
			int triCount = static_cast<int>(triangles.size());
			for (int ti = 0; ti < triCount; ++ti) {
				const auto& t = triangles[ti];
				if (!t.alive)
					continue;
				if (t.v[0] == vKeep || t.v[1] == vKeep || t.v[2] == vKeep ||
					t.v[0] == vRemove || t.v[1] == vRemove || t.v[2] == vRemove)
					incident.push_back(ti);
			}
			return !incident.empty();
		}

		static bool validateCandidate(
			const Candidate& cand,
			const std::vector<WorkingVertex>& vertices,
			const std::vector<Triangle>& triangles,
			const CollisionMeshDecimationOptions& options,
			float baseAbsVolume,
			CollisionMeshDecimationStats& stats)
		{
			const auto& vk = vertices[cand.keep];
			const auto& vr = vertices[cand.remove];

			if (options.preserveBoundary && (vk.boundary != vr.boundary)) {
				++stats.rejectedBoundary;
				return false;
			}

			if (options.preserveFeatures && (vk.feature || vr.feature)) {
				++stats.rejectedFeature;
				return false;
			}

			if (options.maxSkinWeightDrift > 0.0f && cand.skinDrift > options.maxSkinWeightDrift) {
				++stats.rejectedSkin;
				return false;
			}

			std::vector<int> incident;
			gatherIncidentTriangles(cand.keep, cand.remove, triangles, incident);
			if (incident.empty())
				return false;

			float normalDotMin = std::cos(options.maxNormalDeviationDegrees * SIMD_PI / 180.0f);
			float localBefore = 0.0f;
			float localAfter = 0.0f;

			for (int ti : incident) {
				const auto& tri = triangles[ti];
				btVector3 oldP[3] = {
					vertices[tri.v[0]].pos,
					vertices[tri.v[1]].pos,
					vertices[tri.v[2]].pos
				};

				uint32_t newIdx[3] = { tri.v[0], tri.v[1], tri.v[2] };
				for (int i = 0; i < 3; ++i) {
					if (newIdx[i] == cand.remove)
						newIdx[i] = cand.keep;
				}

				if (newIdx[0] == newIdx[1] || newIdx[1] == newIdx[2] || newIdx[2] == newIdx[0])
					continue;

				btVector3 newP[3] = {
					(newIdx[0] == cand.keep ? cand.newPos : vertices[newIdx[0]].pos),
					(newIdx[1] == cand.keep ? cand.newPos : vertices[newIdx[1]].pos),
					(newIdx[2] == cand.keep ? cand.newPos : vertices[newIdx[2]].pos)
				};

				float oldArea2 = triDoubleArea(oldP[0], oldP[1], oldP[2]);
				float newArea2 = triDoubleArea(newP[0], newP[1], newP[2]);
				if (newArea2 < kDegenerateAreaThreshold || oldArea2 < kDegenerateAreaThreshold) {
					++stats.rejectedDegenerate;
					return false;
				}

				btVector3 nOld = triNormal(oldP[0], oldP[1], oldP[2]);
				btVector3 nNew = triNormal(newP[0], newP[1], newP[2]);
				if (nOld.fuzzyZero() || nNew.fuzzyZero()) {
					++stats.rejectedDegenerate;
					return false;
				}

				float nd = nOld.dot(nNew);
				if (nd < normalDotMin) {
					++stats.rejectedNormal;
					return false;
				}

				localBefore += signedVolumeContribution(oldP[0], oldP[1], oldP[2]);
				localAfter += signedVolumeContribution(newP[0], newP[1], newP[2]);
			}

			float localDelta = std::fabs(localAfter - localBefore);
			float localCap = baseAbsVolume * (options.maxLocalVolumeChangePercent / 100.0f);
			if (localCap > FLT_EPSILON && localDelta > localCap) {
				++stats.rejectedVolume;
				return false;
			}

			return true;
		}

		static void applyCandidate(
			const Candidate& cand,
			std::vector<WorkingVertex>& vertices,
			std::vector<Triangle>& triangles)
		{
			auto& vk = vertices[cand.keep];
			auto& vr = vertices[cand.remove];
			btVector3 oldKeepPos = vk.pos;
			float t = collapseFactor(oldKeepPos, vr.pos, cand.newPos);

			vk.pos = cand.newPos;
			vk.skin.m_skinPos.setValue(cand.newPos.x(), cand.newPos.y(), cand.newPos.z());
			mergeSkin(vk.skin, vr.skin, t);
			vr.alive = false;

			for (auto& tri : triangles) {
				if (!tri.alive)
					continue;
				for (int i = 0; i < 3; ++i) {
					if (tri.v[i] == cand.remove)
						tri.v[i] = cand.keep;
				}
				if (tri.v[0] == tri.v[1] || tri.v[1] == tri.v[2] || tri.v[2] == tri.v[0])
					tri.alive = false;
			}
		}

		static int aliveVertexCount(const std::vector<WorkingVertex>& vertices)
		{
			int n = 0;
			for (const auto& v : vertices)
				if (v.alive)
					++n;
			return n;
		}

		static void collectCandidateEdges(
			const std::vector<WorkingVertex>& vertices,
			const std::vector<Triangle>& triangles,
			std::vector<EdgeKey>& outEdges)
		{
			std::unordered_set<EdgeKey, EdgeKeyHash> edges;
			edges.reserve(triangles.size() * 3);
			for (const auto& tri : triangles) {
				if (!tri.alive)
					continue;
				edges.insert(EdgeKey(tri.v[0], tri.v[1]));
				edges.insert(EdgeKey(tri.v[1], tri.v[2]));
				edges.insert(EdgeKey(tri.v[2], tri.v[0]));
			}
			outEdges.assign(edges.begin(), edges.end());
			outEdges.erase(std::remove_if(outEdges.begin(), outEdges.end(), [&](const EdgeKey& e) {
				return !vertices[e.a].alive || !vertices[e.b].alive;
			}),
				outEdges.end());
		}

		static float edgeLength2(const std::vector<WorkingVertex>& vertices, const EdgeKey& e)
		{
			return (vertices[e.a].pos - vertices[e.b].pos).length2();
		}

		static Candidate makeCandidate(
			uint32_t keep,
			uint32_t remove,
			const std::vector<WorkingVertex>& vertices,
			const CollisionMeshDecimationOptions& options,
			float diagSquared)
		{
			Candidate c;
			c.keep = keep;
			c.remove = remove;

			Quadric q = vertices[keep].quadric;
			q.add(vertices[remove].quadric);
			btVector3 mid = (vertices[keep].pos + vertices[remove].pos) * 0.5f;
			c.newPos = optimalPlacement(q, mid);
			float baseCost = q.evaluate(c.newPos);

			float t = collapseFactor(vertices[keep].pos, vertices[remove].pos, c.newPos);
			Vertex merged = predictedMergedSkin(vertices[keep].skin, vertices[remove].skin, t);
			c.skinDrift = 0.5f * (
				skinDistanceL1(merged, vertices[keep].skin) +
				skinDistanceL1(merged, vertices[remove].skin));

			float skinPenalty = options.skinWeightPenalty > 0.0f ?
				(options.skinWeightPenalty * c.skinDrift * std::max(diagSquared, 1.0f)) : 0.0f;

			c.cost = baseCost + skinPenalty;
			return c;
		}

		static bool buildOutput(
			const std::vector<WorkingVertex>& vertices,
			const std::vector<Triangle>& triangles,
			CollisionMeshDecimationOutput& output)
		{
			output.oldToNewVertex.assign(vertices.size(), std::numeric_limits<uint32_t>::max());

			for (uint32_t i = 0; i < vertices.size(); ++i) {
				if (!vertices[i].alive)
					continue;
				output.oldToNewVertex[i] = static_cast<uint32_t>(output.vertices.size());
				Vertex v = vertices[i].skin;
				v.m_skinPos.setValue(vertices[i].pos.x(), vertices[i].pos.y(), vertices[i].pos.z());
				output.vertices.push_back(v);
			}

			output.triangles.reserve(triangles.size());
			for (const auto& tri : triangles) {
				if (!tri.alive)
					continue;
				uint32_t a = output.oldToNewVertex[tri.v[0]];
				uint32_t b = output.oldToNewVertex[tri.v[1]];
				uint32_t c = output.oldToNewVertex[tri.v[2]];
				if (a == std::numeric_limits<uint32_t>::max() || b == std::numeric_limits<uint32_t>::max() ||
					c == std::numeric_limits<uint32_t>::max())
					return false;
				if (a == b || b == c || c == a)
					continue;
				output.triangles.push_back({ a, b, c });
			}

			return !output.vertices.empty() && !output.triangles.empty();
		}
	}

	CollisionMeshDecimationOutput DecimateCollisionMesh(
		const std::vector<Vertex>& inputVertices,
		const std::vector<std::array<uint32_t, 3>>& inputTriangles,
		const CollisionMeshDecimationOptions& options)
	{
		CollisionMeshDecimationOutput out;
		out.stats.originalVertexCount = static_cast<int>(inputVertices.size());
		out.stats.originalTriangleCount = static_cast<int>(inputTriangles.size());

		if (!options.enabled || inputVertices.empty() || inputTriangles.empty()) {
			out.vertices = inputVertices;
			out.triangles = inputTriangles;
			out.oldToNewVertex.resize(inputVertices.size());
			for (uint32_t i = 0; i < out.oldToNewVertex.size(); ++i)
				out.oldToNewVertex[i] = i;
			out.stats.outputVertexCount = static_cast<int>(out.vertices.size());
			out.stats.outputTriangleCount = static_cast<int>(out.triangles.size());
			return out;
		}

		std::vector<WorkingVertex> vertices(inputVertices.size());
		for (uint32_t i = 0; i < inputVertices.size(); ++i) {
			vertices[i].skin = inputVertices[i];
			vertices[i].pos = toVec3(inputVertices[i]);
		}

		std::vector<Triangle> triangles(inputTriangles.size());
		for (uint32_t i = 0; i < inputTriangles.size(); ++i) {
			triangles[i].v[0] = inputTriangles[i][0];
			triangles[i].v[1] = inputTriangles[i][1];
			triangles[i].v[2] = inputTriangles[i][2];
			if (triangles[i].v[0] >= vertices.size() || triangles[i].v[1] >= vertices.size() || triangles[i].v[2] >= vertices.size())
				triangles[i].alive = false;
		}

		out.stats.initialSignedVolume = signedVolume(vertices, triangles);
		float baseAbsVolume = std::fabs(out.stats.initialSignedVolume);
		float diag = 0.0f;
		if (!vertices.empty()) {
			btVector3 mn = vertices[0].pos;
			btVector3 mx = vertices[0].pos;
			for (const auto& v : vertices) {
				if (!v.alive)
					continue;
				mn.setMin(v.pos);
				mx.setMax(v.pos);
			}
			diag = (mx - mn).length();
		}

		// Normalize the QEM cost threshold by diag² so it becomes scale-invariant,
		// matching the way shortEdgeRatio already self-normalizes via the mesh diagonal.
		// A qemCostThreshold of 1.0 therefore permits collapses whose squared positional
		// error is at most diag² (i.e. the full bounding-box diagonal), which is a
		// mesh-size-relative quality bound rather than an absolute distance.
		float scaledQemThreshold = 0.0f;
		if (options.qemCostThreshold > 0.0f && diag > 0.0f)
			scaledQemThreshold = options.qemCostThreshold * diag * diag;
		const float diagSquared = diag > 0.0f ? (diag * diag) : 1.0f;

		int targetVertexCount = static_cast<int>(inputVertices.size());
		if (options.targetVertexCount > 0) {
			targetVertexCount = std::max(3, std::min(static_cast<int>(inputVertices.size()), options.targetVertexCount));
		} else if (options.targetVertexRatio > 0.0f && options.targetVertexRatio < 1.0f) {
			targetVertexCount = std::max(3, static_cast<int>(std::ceil(options.targetVertexRatio * inputVertices.size())));
		}

		recomputeEdgeFlags(vertices, triangles, options);
		recomputeQuadrics(vertices, triangles);

		// Pass A: short-edge point-decimation style removal.
		for (;;) {
			if (aliveVertexCount(vertices) <= targetVertexCount)
				break;
			if (options.maxPointRemovals > 0 && out.stats.pointRemovals >= options.maxPointRemovals)
				break;

			std::vector<EdgeKey> edges;
			collectCandidateEdges(vertices, triangles, edges);
			if (edges.empty())
				break;

			float shortEdge = diag * std::max(0.0f, options.shortEdgeRatio);
			float shortEdge2 = shortEdge * shortEdge;
			EdgeKey best;
			float bestLen2 = std::numeric_limits<float>::max();

			for (const auto& e : edges) {
				float len2 = edgeLength2(vertices, e);
				if (len2 <= shortEdge2 && len2 < bestLen2) {
					best = e;
					bestLen2 = len2;
				}
			}

			if (bestLen2 == std::numeric_limits<float>::max())
				break;

			Candidate c0 = makeCandidate(best.a, best.b, vertices, options, diagSquared);
			Candidate c1 = makeCandidate(best.b, best.a, vertices, options, diagSquared);
			Candidate c = (c0.cost <= c1.cost) ? c0 : c1;

			if (!validateCandidate(c, vertices, triangles, options, baseAbsVolume, out.stats)) {
				++out.stats.rejectedCollapses;
				break;
			}

			applyCandidate(c, vertices, triangles);
			++out.stats.pointRemovals;
			out.stats.maxAcceptedSkinDrift = std::max(out.stats.maxAcceptedSkinDrift, c.skinDrift);
			recomputeEdgeFlags(vertices, triangles, options);
			recomputeQuadrics(vertices, triangles);
		}

		// Pass B: constrained QEM edge collapse.
		for (;;) {
			if (aliveVertexCount(vertices) <= targetVertexCount)
				break;
			if (options.maxEdgeCollapses > 0 && out.stats.edgeCollapses >= options.maxEdgeCollapses)
				break;

			std::vector<EdgeKey> edges;
			collectCandidateEdges(vertices, triangles, edges);
			if (edges.empty())
				break;

			Candidate best;
			for (const auto& e : edges) {
				Candidate c0 = makeCandidate(e.a, e.b, vertices, options, diagSquared);
				Candidate c1 = makeCandidate(e.b, e.a, vertices, options, diagSquared);
				const Candidate& c = (c0.cost <= c1.cost) ? c0 : c1;
				if (c.cost < best.cost)
					best = c;
			}

			if (best.cost == std::numeric_limits<float>::max())
				break;
			if (scaledQemThreshold > 0.0f && best.cost > scaledQemThreshold)
				break;

			if (!validateCandidate(best, vertices, triangles, options, baseAbsVolume, out.stats)) {
				++out.stats.rejectedCollapses;
				break;
			}

			applyCandidate(best, vertices, triangles);
			++out.stats.edgeCollapses;
			out.stats.maxAcceptedSkinDrift = std::max(out.stats.maxAcceptedSkinDrift, best.skinDrift);
			recomputeEdgeFlags(vertices, triangles, options);
			recomputeQuadrics(vertices, triangles);

			float currentSignedVolume = signedVolume(vertices, triangles);
			float volumeDelta = std::fabs(currentSignedVolume - out.stats.initialSignedVolume);
			float volumeCap = baseAbsVolume * (std::max(0.0f, options.maxVolumeLossPercent) / 100.0f);
			if (volumeCap > FLT_EPSILON && volumeDelta > volumeCap) {
				out.stats.usedFallback = true;
				out.stats.fallbackReason = "volume-loss-threshold";
				out.vertices = inputVertices;
				out.triangles = inputTriangles;
				out.oldToNewVertex.resize(inputVertices.size());
				for (uint32_t i = 0; i < out.oldToNewVertex.size(); ++i)
					out.oldToNewVertex[i] = i;
				out.stats.outputVertexCount = static_cast<int>(out.vertices.size());
				out.stats.outputTriangleCount = static_cast<int>(out.triangles.size());
				out.stats.outputSignedVolume = out.stats.initialSignedVolume;
				out.stats.volumeDeltaPercent = 0.0f;
				return out;
			}
		}

		if (!buildOutput(vertices, triangles, out)) {
			out.stats.usedFallback = true;
			out.stats.fallbackReason = "invalid-output";
			out.vertices = inputVertices;
			out.triangles = inputTriangles;
			out.oldToNewVertex.resize(inputVertices.size());
			for (uint32_t i = 0; i < out.oldToNewVertex.size(); ++i)
				out.oldToNewVertex[i] = i;
		}

		// Validate output geometry.
		bool hasDegenerate = false;
		for (const auto& tri : out.triangles) {
			const auto& a = out.vertices[tri[0]].m_skinPos;
			const auto& b = out.vertices[tri[1]].m_skinPos;
			const auto& c = out.vertices[tri[2]].m_skinPos;
			if (triDoubleArea(btVector3(a.x(), a.y(), a.z()), btVector3(b.x(), b.y(), b.z()), btVector3(c.x(), c.y(), c.z())) < kDegenerateAreaThreshold) {
				hasDegenerate = true;
				break;
			}
		}
		if (hasDegenerate) {
			out.stats.usedFallback = true;
			out.stats.fallbackReason = "degenerate-output";
			out.vertices = inputVertices;
			out.triangles = inputTriangles;
			out.oldToNewVertex.resize(inputVertices.size());
			for (uint32_t i = 0; i < out.oldToNewVertex.size(); ++i)
				out.oldToNewVertex[i] = i;
		}

		std::vector<WorkingVertex> outWork(out.vertices.size());
		for (uint32_t i = 0; i < out.vertices.size(); ++i) {
			outWork[i].pos = toVec3(out.vertices[i]);
			outWork[i].alive = true;
		}
		std::vector<Triangle> outTris(out.triangles.size());
		for (uint32_t i = 0; i < out.triangles.size(); ++i) {
			outTris[i].v[0] = out.triangles[i][0];
			outTris[i].v[1] = out.triangles[i][1];
			outTris[i].v[2] = out.triangles[i][2];
			outTris[i].alive = true;
		}

		out.stats.outputVertexCount = static_cast<int>(out.vertices.size());
		out.stats.outputTriangleCount = static_cast<int>(out.triangles.size());
		out.stats.outputSignedVolume = signedVolume(outWork, outTris);
		if (std::fabs(out.stats.initialSignedVolume) > FLT_EPSILON) {
			out.stats.volumeDeltaPercent =
				100.0f * std::fabs(out.stats.outputSignedVolume - out.stats.initialSignedVolume) /
				std::fabs(out.stats.initialSignedVolume);
		}

		return out;
	}
}
