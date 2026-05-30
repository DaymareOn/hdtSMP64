#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtCollider.h"
#include <tbb/task_arena.h>
#include <LinearMath/btQuickprof.h>

namespace hdt
{

	// CollisionCheckBase1 provides data members and the basic constructor for the target types. Note that we
	// always collide a vertex shape against something else, so only the second type is templated.
	template <typename T>
	struct CollisionCheckBase1
	{
		typedef typename PerVertexShape::ShapeProp SP0;
		typedef typename T::ShapeProp SP1;

		CollisionCheckBase1(PerVertexShape* a, T* b, CollisionResult* r)
		{
			v0 = a->m_owner->m_vpos.data();
			v1 = b->m_owner->m_vpos.data();
			c0 = &a->m_tree;
			c1 = &b->m_tree;
			sp0 = &a->m_shapeProp;
			sp1 = &b->m_shapeProp;
			// Per-frame precomputed face normals of the second shape, indexed in parallel with its
			// collider array (faceColBase). Only meaningful when b is a PerTriangleShape; for the
			// sphere-sphere path m_faceNormal is empty and these are never read.
			faceNormals = b->m_faceNormal.data();
			faceColBase = b->m_colliders.data();
			results = r;
			numResults = 0;
		}

		VertexPos* v0;
		VertexPos* v1;
		ColliderTree* c0;
		ColliderTree* c1;
		SP0* sp0;
		SP1* sp1;
		const __m128* faceNormals;
		const Collider* faceColBase;

		std::atomic_long numResults;
		CollisionResult* results;
	};

	// CollisionCheckBase2 provides the method to add results, swapping the colliders if necessary. This
	// means we can support triangle-sphere collisions by reversing the input shapes and setting SwapResults
	// to true, instead of having two almost identical versions of the same lower-level algorithm.
	template <typename T, bool SwapResults>
	struct CollisionCheckBase2;

	template <typename T>
	struct CollisionCheckBase2<T, false> : public CollisionCheckBase1<T>
	{
		template <typename... Ts>
		CollisionCheckBase2(Ts&&... ts) :
			CollisionCheckBase1<T>(std::forward<Ts>(ts)...)
		{}

		bool addResult(const CollisionResult& res)
		{
			int p = this->numResults.fetch_add(1);
			if (p < SkinnedMeshAlgorithm::MaxCollisionCount) {
				this->results[p] = res;
				return true;
			}
			return false;
		}
	};

	template <typename T>
	struct CollisionCheckBase2<T, true> : public CollisionCheckBase1<T>
	{
		template <typename... Ts>
		CollisionCheckBase2(Ts&&... ts) :
			CollisionCheckBase1<T>(std::forward<Ts>(ts)...)
		{}

		bool addResult(const CollisionResult& res)
		{
			int p = this->numResults.fetch_add(1);
			if (p < SkinnedMeshAlgorithm::MaxCollisionCount) {
				this->results[p].posA = res.posB;
				this->results[p].posB = res.posA;
				this->results[p].colliderA = res.colliderB;
				this->results[p].colliderB = res.colliderA;
				this->results[p].normOnB = -res.normOnB;
				this->results[p].depth = res.depth;
				return true;
			}
			return false;
		}
	};

	// CollisionChecker provides the checkCollide method, which handles a single pair of colliders. This does
	// the accurate collision check for the CPU algorithms. GPU algorithms will provide their own methods for
	// this, and should derive directly from CollisionCheckBase2.
	template <typename T, bool SwapResults>
	struct CollisionChecker;

	template <bool SwapResults>
	struct CollisionChecker<PerVertexShape, SwapResults> : public CollisionCheckBase2<PerVertexShape, SwapResults>
	{
		template <typename... Ts>
		CollisionChecker(Ts&&... ts) :
			CollisionCheckBase2<PerVertexShape, SwapResults>(std::forward<Ts>(ts)...)
		{}

		bool checkCollide(Collider* a, Collider* b, CollisionResult& res)
		{
			auto s0 = this->v0[a->vertex];
			auto r0 = s0.marginMultiplier() * this->sp0->margin;
			auto s1 = this->v1[b->vertex];
			auto r1 = s1.marginMultiplier() * this->sp1->margin;

			auto ret = checkSphereSphere(s0.pos(), s1.pos(), r0, r1, res);
			res.colliderA = a;
			res.colliderB = b;
			return ret;
		}
	};

	namespace
	{
		__forceinline __m128 cross_product(__m128 const& vec0, __m128 const& vec1)
		{
			__m128 tmp0 = _mm_shuffle_ps(vec0, vec0, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 tmp1 = _mm_shuffle_ps(vec1, vec1, _MM_SHUFFLE(3, 1, 0, 2));
			__m128 tmp2 = _mm_mul_ps(tmp0, vec1);
			__m128 tmp3 = _mm_mul_ps(tmp0, tmp1);
			__m128 tmp4 = _mm_shuffle_ps(tmp2, tmp2, _MM_SHUFFLE(3, 0, 2, 1));
			return _mm_sub_ps(tmp3, tmp4);
		}
	}

	template <bool SwapResults>
	struct CollisionChecker<PerTriangleShape, SwapResults> : public CollisionCheckBase2<PerTriangleShape, SwapResults>
	{
		template <typename... Ts>
		CollisionChecker(Ts&&... ts) :
			CollisionCheckBase2<PerTriangleShape, SwapResults>(std::forward<Ts>(ts)...)
		{}

		bool checkCollide(Collider* a, Collider* b, CollisionResult& res)
		{
			auto s = this->v0[a->vertex];
			auto p0 = this->v1[b->vertices[0]];
			auto p1 = this->v1[b->vertices[1]];
			auto p2 = this->v1[b->vertices[2]];

			// Triangle face normal (unnormalized) and its squared length, precomputed once per frame in
			// PerTriangleShape::internalUpdate (it depends only on the triangle, not the sphere). xyz =
			// raw_normal = (p1-p0)x(p2-p0); w = len2 = |raw_normal|^2 = (2 * triangle area)^2.
			auto raw_normal = this->faceNormals[b - this->faceColBase];
			// len2 rejects degenerate (zero-area) triangles without a sqrt. The sqrt/normalize is
			// deferred to the accept path (bottom of the function) so reject paths never pay for it.
			// (raw_normal's w-lane holds len2; the dp/cross below all mask to xyz so it is harmless.)
			float len2 = _mm_cvtss_f32(pshufd<0xFF>(raw_normal));
			if (len2 < FLT_EPSILON * FLT_EPSILON) {
				return false;
			}

			// Point-in-triangle test, evaluated at the SPHERE CENTRE directly (no projection needed).
			//
			// We need to know whether the sphere centre's projection onto the triangle plane lands
			// inside the triangle. Barycentric coordinates are invariant under translation along the
			// plane normal, and the projection is exactly the centre shifted along raw_normal — so the
			// projection's barycentric weights equal the centre's. The projection cancels out of the
			// determinants algebraically (the cross-terms vanish because each contains raw_normal
			// twice):
			//   det[(s-p1) - N*k, (s-p2) - N*k, N] = det[s-p1, s-p2, N].
			// Testing the centre lets us skip the projection on every reject path, which in turn
			// removes the scalar division (distRaw / len2) the projection required from the hot path
			// (that division sat on the critical dependency chain).
			//
			// Each sub-triangle area vector dotted with raw_normal gives a (scaled) barycentric weight:
			//   da = dot(cross(s-p1, s-p2), raw_normal)   (∝ weight of p0)
			//   db = dot(cross(s-p2, s-p0), raw_normal)   (∝ weight of p1)
			// The three area vectors sum identically to raw_normal, so the third weight is
			// dc = len2 - da - db — no need to compute its cross product or dot. The centre projects
			// inside the triangle iff all three weights are non-negative:
			//   da >= 0 && db >= 0 && (da + db) <= len2.
			// raw_normal (not a sign-flipped normal) is the correct reference, since the sub-triangle
			// windings follow the original triangle winding.
			//
			// da is tested first so we can bail before computing db: db needs its own subtract, cross
			// and dot, all wasted when the centre already fails on the first edge. (sp1/sp2 feed da;
			// sp0 feeds only db.)
			auto sp1 = (s.pos() - p1.pos()).get128();
			auto sp2 = (s.pos() - p2.pos()).get128();
			float da = _mm_cvtss_f32(_mm_dp_ps(cross_product(sp1, sp2), raw_normal, 0x71));
			if (da < 0) {
				return false;
			}
			auto sp0 = (s.pos() - p0.pos()).get128();
			float db = _mm_cvtss_f32(_mm_dp_ps(cross_product(sp2, sp0), raw_normal, 0x71));
			if (db < 0 || da + db > len2) {
				return false;
			}

			// ---- Accept path: the centre projects inside the triangle. Everything below (margin/
			// penetration, the sqrt/normalize, the plane test, and the contact output) is needed only
			// on this surviving path, so it is deferred past the cheaper rejects above. ----

			// Per-contact margin and penetration band.
			auto r = s.marginMultiplier() * this->sp0->margin;
			// Average the three vertex margin multipliers in SIMD: sum the packed VertexPos data (the
			// margin multiplier lives in the w-lane) and read w once, instead of three scalar
			// w-extracts plus scalar adds.
			auto marginSum = _mm_add_ps(_mm_add_ps(p0.m_data, p1.m_data), p2.m_data);
			auto margin = _mm_cvtss_f32(pshufd<0xFF>(marginSum)) / 3;
			auto penetration = this->sp1->penetration * margin;
			margin *= this->sp1->margin;
			if (penetration > -FLT_EPSILON && penetration < FLT_EPSILON) {
				penetration = 0;
			}

			// Signed plane distance for the contact-plane test. distRaw = dot(s-p0, raw_normal) =
			// distanceFromPlane * len (original winding). len (the sqrt) is needed here because the test
			// compares the true distance against the margin. The unit normal VECTOR, however, is only
			// needed for the contact output, so we merely track its sign flips here (a bool) and build
			// the vector — a divide plus a conditional negate — once, on the accept path below. That
			// keeps both off the plane-test reject path.
			float distRaw = _mm_cvtss_f32(_mm_dp_ps(sp0, raw_normal, 0x71));
			auto len = _mm_sqrt_ps(_mm_set_ss(len2));
			float distanceFromPlane = distRaw / _mm_cvtss_f32(len);
			bool negateNormal = false;
			if (penetration < 0) {
				distanceFromPlane = -distanceFromPlane;
				penetration = -penetration;
				negateNormal = true;
			}

			float radiusWithMargin = r + margin;
			bool isInsideContactPlane;
			if (penetration >= FLT_EPSILON)
				isInsideContactPlane = distanceFromPlane < radiusWithMargin && distanceFromPlane >= -penetration;
			else {
				if (distanceFromPlane < 0) {
					distanceFromPlane = -distanceFromPlane;
					negateNormal = !negateNormal;
				}
				isInsideContactPlane = distanceFromPlane < radiusWithMargin;
			}
			if (!isInsideContactPlane) {
				return false;
			}

			// Accept path. Build the unit normal now (deferred past the plane-test reject above) and
			// apply the accumulated sign. distanceFromPlane already carries the matching sign.
			auto normal = _mm_div_ps(raw_normal, pshufd<0>(len));
			if (negateNormal) {
				normal = _mm_sub_ps(_mm_setzero_ps(), normal);
			}

			// Projection of the sphere centre onto the triangle plane (only needed for the contact
			// point). It uses the unflipped raw_normal and distRaw, so it is independent of the normal
			// sign tracking above; the len2 form is exact regardless of winding:
			//   projection = s - normal * distanceFromPlane = s - raw_normal * (distRaw / len2).
			auto projection = nmsub_ps(raw_normal, _mm_set1_ps(distRaw / len2), s.pos().get128());

			res.colliderA = a;
			res.colliderB = b;
			res.normOnB.set128(normal);
			res.posA = s.pos() - res.normOnB * r;
			res.posB.set128(projection);
			res.depth = distanceFromPlane - radiusWithMargin;
			return res.depth < -FLT_EPSILON;
		}

#if 0
		// PREVIOUS REFERENCE IMPLEMENTATION (kept as dead code for comparison / rollback).
		//
		// Functionally equivalent to the active checkCollide above. The key difference: this version
		// computes the in-plane 'projection' of the sphere centre up front and runs the point-in-
		// triangle test on it, which forces the scalar division (distRaw / len2) onto the hot reject
		// path. The active version proves the projection cancels out of the barycentric determinants
		// (translation along the plane normal preserves barycentric coordinates), tests the sphere
		// centre directly, and defers the projection + division to the accept path.
		bool checkCollide_projectionBased(Collider* a, Collider* b, CollisionResult& res)
		{
			auto s = this->v0[a->vertex];
			auto p0 = this->v1[b->vertices[0]];
			auto p1 = this->v1[b->vertices[1]];
			auto p2 = this->v1[b->vertices[2]];

			auto ab = (p1.pos() - p0.pos()).get128();
			auto ac = (p2.pos() - p0.pos()).get128();
			auto raw_normal = cross_product(ab, ac);
			// len2 = |raw_normal|^2 = (2*triangle area)^2. Reject degenerate triangles without a
			// sqrt. The actual sqrt/normalize is deferred to the very end (see below) so the common
			// reject paths never pay for it.
			float len2 = _mm_cvtss_f32(_mm_dp_ps(raw_normal, raw_normal, 0x71));
			if (len2 < FLT_EPSILON * FLT_EPSILON) {
				return false;
			}

			auto ap = (s.pos() - p0.pos()).get128();
			// distRaw = dot(ap, raw_normal) = (signed plane distance) * len. Unnormalized, original winding.
			float distRaw = _mm_cvtss_f32(_mm_dp_ps(ap, raw_normal, 0x71));

			// Projection of the sphere center onto the triangle plane.
			//   projection = s - normal * dot(ap, normal)
			//             = s - (raw_normal/len) * (distRaw/len)
			//             = s - raw_normal * (distRaw / len2)
			// The normal-flips below cancel in this product, so this is exact regardless of winding
			// and needs only len2 (no sqrt).
			auto projection = nmsub_ps(raw_normal, _mm_set1_ps(distRaw / len2), s.pos().get128());

			// Point-in-triangle via barycentric coordinates from sub-triangle area vectors.
			//
			// Build the sub-triangles fanning out from the projected point to each edge and take the
			// scalar triple product of each against raw_normal (= cross(p1-p0, p2-p0)):
			//   da = dot(cross(bp, cp), raw_normal)   (∝ barycentric weight of p0)
			//   db = dot(cross(cp, ap), raw_normal)   (∝ barycentric weight of p1)
			// The three sub-triangle area vectors sum identically to raw_normal for any in-plane
			// point, so the third weight is dc = len2 - da - db — no need to compute its cross
			// product or dot. The projection is inside if all three weights are non-negative:
			//   da >= 0 && db >= 0 && (da + db) <= len2.
			// raw_normal (not the possibly-flipped 'normal') is the correct reference since the
			// sub-triangle windings follow the original triangle winding. No sqrt here.
			//
			// This runs BEFORE the plane-distance test (the two checks are AND-combined, so order is
			// irrelevant to the result) precisely so out-of-triangle rejects skip the sqrt/normalize.
			// Test da first and bail before computing db: db needs its own cross product, dot, and the
			// 'ap' subtract, all of which are wasted work when the projection already fails on the
			// first edge. (bp/cp feed da; ap feeds only db.)
			auto bp = _mm_sub_ps(projection, p1.pos().get128());
			auto cp = _mm_sub_ps(projection, p2.pos().get128());
			float da = _mm_cvtss_f32(_mm_dp_ps(cross_product(bp, cp), raw_normal, 0x71));
			if (da < 0) {
				return false;
			}
			ap = _mm_sub_ps(projection, p0.pos().get128());
			float db = _mm_cvtss_f32(_mm_dp_ps(cross_product(cp, ap), raw_normal, 0x71));
			if (db < 0 || da + db > len2) {
				return false;
			}

			// Confirmed inside the triangle. Everything below — margin/penetration, the sqrt/normalize,
			// the plane test and the contact output — is needed only on this surviving path, so it is
			// deferred past the cheaper degenerate and point-in-triangle rejects above.
			auto r = s.marginMultiplier() * this->sp0->margin;
			// Average the three vertex margin multipliers in SIMD: sum the packed VertexPos data
			// (margin multiplier lives in the w-lane) and read w once, instead of three scalar
			// w-extracts plus scalar adds.
			auto marginSum = _mm_add_ps(_mm_add_ps(p0.m_data, p1.m_data), p2.m_data);
			auto margin = _mm_cvtss_f32(pshufd<0xFF>(marginSum)) / 3;
			auto penetration = this->sp1->penetration * margin;
			margin *= this->sp1->margin;
			if (penetration > -FLT_EPSILON && penetration < FLT_EPSILON) {
				penetration = 0;
			}

			// distanceFromPlane = dot(ap_orig, normal) = distRaw / len.
			auto len = _mm_sqrt_ps(_mm_set_ss(len2));
			auto normal = _mm_div_ps(raw_normal, pshufd<0>(len));
			float distanceFromPlane = distRaw / _mm_cvtss_f32(len);
			if (penetration < 0) {
				normal = _mm_sub_ps(_mm_setzero_ps(), normal);
				distanceFromPlane = -distanceFromPlane;
				penetration = -penetration;
			}

			float radiusWithMargin = r + margin;
			bool isInsideContactPlane;
			if (penetration >= FLT_EPSILON)
				isInsideContactPlane = distanceFromPlane < radiusWithMargin && distanceFromPlane >= -penetration;
			else {
				if (distanceFromPlane < 0) {
					distanceFromPlane = -distanceFromPlane;
					normal = _mm_sub_ps(_mm_setzero_ps(), normal);
				}
				isInsideContactPlane = distanceFromPlane < radiusWithMargin;
			}
			if (!isInsideContactPlane) {
				return false;
			}

			res.colliderA = a;
			res.colliderB = b;
			res.normOnB.set128(normal);
			res.posA = s.pos() - res.normOnB * r;
			res.posB.set128(projection);
			res.depth = distanceFromPlane - radiusWithMargin;
			return res.depth < -FLT_EPSILON;
		}
#endif
	};

	template <typename T, bool SwapResults>
	struct CollisionCheckDispatcher : public CollisionChecker<T, SwapResults>
	{
		template <typename... Ts>
		CollisionCheckDispatcher(Ts&&... ts) :
			CollisionChecker<T, SwapResults>(std::forward<Ts>(ts)...)
		{}

		// We intentionally don't use a 'Dynamic 1D Sweep and Prune Algorithm' here.
		// O(N*M) is nearly always faster or within a margin of error. Not worth the extra boilerplate code
		void dispatch(ColliderTree* a, ColliderTree* b, std::vector<Aabb*>& listA, std::vector<Aabb*>& listB, const Aabb& refinedBForPruningA)
		{
			if (this->numResults.load(std::memory_order_relaxed) >= SkinnedMeshAlgorithm::MaxCollisionCount)
				return;
			BT_PROFILE("dispatch_body");
			CollisionResult result;
			CollisionResult temp;
			bool hasResult = false;

			auto abeg = a->aabb;
			auto bbeg = b->aabb;

			if (listA.size() && listB.size()) {
				for (auto i : listA) {
					if (!i->collideWith(refinedBForPruningA))
						continue;
					for (auto j : listB) {
						if (!i->collideWith(*j))
							continue;
						if (this->checkCollide(&a->cbuf[i - abeg], &b->cbuf[j - bbeg], temp)) {
							if (!hasResult || result.depth > temp.depth) {
								hasResult = true;
								result = temp;
							}
						}
					}
				}
			}

			if (hasResult) {
				this->addResult(result);
			}
		}
	};

	template <typename T, bool SwapResults = false>
	struct CollisionCheckAlgorithm : public CollisionCheckDispatcher<T, SwapResults>
	{
		template <typename... Ts>
		CollisionCheckAlgorithm(Ts&&... ts) :
			CollisionCheckDispatcher<T, SwapResults>(std::forward<Ts>(ts)...)
		{}

		int operator()()
		{
			std::vector<std::pair<ColliderTree*, ColliderTree*>> pairs;
			pairs.reserve(this->c0->colliders.size() + this->c1->colliders.size());
			{
				BT_PROFILE("BVH");
				this->c0->checkCollisionL(this->c1, pairs);
			}
			if (pairs.empty())
				return 0;

			// The collision is just too complex to solve. We must quit before we explode the user's computer
			// If this DOES solve, it seems to only cause the collision to become significantly more tangles up
			if (pairs.size() > MaxCollisionPairs) {
				return 0;
			}

			decltype(auto) func = [this](const std::pair<ColliderTree*, ColliderTree*>& pair) {
				if (this->numResults.load(std::memory_order_relaxed) >= SkinnedMeshAlgorithm::MaxCollisionCount)
					return;

				auto a = pair.first, b = pair.second;

				auto abeg = a->aabb;
				auto bbeg = b->aabb;
				auto asize = b->isKinematic ? a->dynCollider : a->numCollider;
				auto bsize = a->isKinematic ? b->dynCollider : b->numCollider;
				auto aend = abeg + asize;
				auto bend = bbeg + bsize;

				Aabb aabbA;
				auto aabbB = b->aabbMe;

				thread_local std::vector<Aabb*> listA;
				thread_local std::vector<Aabb*> listB;

				listA.clear();
				listB.clear();
				listA.reserve(asize);
				listB.reserve(bsize);

				{
					BT_PROFILE("filter_lists");
					// Colliders in A that intersect full bounding box of B. Compute a new bounding box for just those - this
					// can be MUCH smaller than the original bounding box for A (consider the case where we have two spheres
					// colliding, offset by an equal amount in all three axes).
					for (auto i = abeg; i < aend; ++i) {
						if (i->collideWith(aabbB)) {
							listA.push_back(i);
							aabbA.merge(*i);
						}
					}

					// Colliders in B that intersect the new bounding box for A. Compute a new bounding box for those too.
					if (listA.size()) {
						aabbB.invalidate();
						for (auto i = bbeg; i < bend; ++i) {
							if (i->collideWith(aabbA)) {
								listB.push_back(i);
								aabbB.merge(*i);
							}
						}
					}
				}

				// Now go through both lists and do the real collision (if needed).
				this->dispatch(a, b, listA, listB, aabbB);
			};

			{
				BT_PROFILE("dispatch");
				if (pairs.size() >= 32)
					// isolate: thread parked here waiting for inner work must not steal an outer
					// processCollision task — that would alias thread_local MergeBuffer/listA/listB.
					tbb::this_task_arena::isolate([&] { tbb::parallel_for_each(pairs.begin(), pairs.end(), func); });
				else
					for (auto& i : pairs) func(i);
			}

			return this->numResults;
		}
	};

	template <class T1>
	int checkCollide(PerVertexShape* a, T1* b, CollisionResult* results)
	{
		return CollisionCheckAlgorithm<T1>(a, b, results)();
	}

	int checkCollide(PerTriangleShape* a, PerVertexShape* b, CollisionResult* results)
	{
		return CollisionCheckAlgorithm<PerTriangleShape, true>(b, a, results)();
	}

	template <class T0, class T1>
	void SkinnedMeshAlgorithm::MergeBuffer::doMerge(T0* a, T1* b, CollisionResult* collision, int count)
	{
		// Cache these outside the collision loop: they don't change across results and each
		// access otherwise requires two pointer dereferences inside a tight nested loop.
		const int bpcA = a->getBonePerCollider();
		const int bpcB = b->getBonePerCollider();
		auto* bonesA = a->m_owner->m_skinnedBones.data();
		auto* bonesB = b->m_owner->m_skinnedBones.data();

		for (int i = 0; i < count; ++i) {
			auto& res = collision[i];
			if (res.depth >= -FLT_EPSILON)
				break;

			auto flexible = std::max(res.colliderA->flexible, res.colliderB->flexible);
			// [3/13/2026]
			// Note: This was using a break before, but logically that doesn't make sense?
			// if we hit a stiffer collider earlier than our depth target, it'd early exit..
			if (flexible < FLT_EPSILON)
				continue;

			float w = flexible * res.depth;
			float w2 = w * w;

			// pre-scale outside the bone loop, these don't depend on bone indices and the inner
			// loop runs bonePerCollider^2 times, so this matters
			auto normScaled = res.normOnB * w * w2;  // cubic weight: bakes depth into normal magnitude
			auto posAScaled = res.posA * w2;
			auto posBScaled = res.posB * w2;

			for (int ib = 0; ib < bpcA; ++ib) {
				auto w0 = a->getColliderBoneWeight(res.colliderA, ib);
				int boneIdx0 = a->getColliderBoneIndex(res.colliderA, ib);
				if (w0 <= bonesA[boneIdx0].weightThreshold)
					continue;

				bool kinA = bonesA[boneIdx0].isKinematic;

				for (int jb = 0; jb < bpcB; ++jb) {
					auto w1 = b->getColliderBoneWeight(res.colliderB, jb);
					int boneIdx1 = b->getColliderBoneIndex(res.colliderB, jb);
					if (w1 <= bonesB[boneIdx1].weightThreshold)
						continue;

					if (kinA && bonesB[boneIdx1].isKinematic)
						continue;

					auto c = getAndTrack(boneIdx0, boneIdx1);

					// If we already have a primary direction (weight > 0),
					// and this new contact pushes in the opposite direction (dot < 0),
					// reject it entirely. This prevents the vector cancellation that creates
					// unpredictable movement, and preserves the depth/weight ratio
					// [If we get jitter, try removing this]
					if (c->weight > FLT_EPSILON && c->normal.dot(normScaled) < 0) {
						continue;
					}

					c->weight += w2;
					c->normal += normScaled;
					c->pos[0] += posAScaled;
					c->pos[1] += posBScaled;
				}
			}
		}
	}

	void SkinnedMeshAlgorithm::MergeBuffer::apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1,
		CollisionDispatcher* dispatcher)
	{
		// only visit cells that were actually written to this frame,
		// instead of looping all bones0 * bones1 (far fewer iterations)
		for (int flatIdx : activeCells) {
			int i = flatIdx / mergeStride;
			int j = flatIdx % mergeStride;

			auto* c = &buffer[flatIdx];
			if (c->weight < FLT_EPSILON)
				continue;

			if (!body1->canCollideWith(body0->m_skinnedBones[i].ptr))
				continue;
			if (!body0->canCollideWith(body1->m_skinnedBones[j].ptr))
				continue;
			if (body0->m_skinnedBones[i].isKinematic && body1->m_skinnedBones[j].isKinematic)
				continue;

			auto rb0 = body0->m_skinnedBones[i].ptr;
			auto rb1 = body1->m_skinnedBones[j].ptr;
			if (rb0 == rb1)
				continue;

			float invWeight = 1.0f / c->weight;

			auto worldA = c->pos[0] * invWeight;
			auto worldB = c->pos[1] * invWeight;
			auto localA = rb0->m_rig.getWorldTransform().invXform(worldA);
			auto localB = rb1->m_rig.getWorldTransform().invXform(worldB);
			auto normal = c->normal * invWeight;
			if (normal.fuzzyZero())
				continue;

			// depth was baked into normal magnitude during doMerge (weighted cubically instead of storing a separate depth field)
			auto depth = -normal.length();
			normal = -normal.normalized();

			if (depth >= -FLT_EPSILON)
				continue;
			btManifoldPoint newPt(localA, localB, normal, depth);
			newPt.m_positionWorldOnA = worldA;
			newPt.m_positionWorldOnB = worldB;
			newPt.m_combinedFriction = rb0->m_rig.getFriction() * rb1->m_rig.getFriction();
			newPt.m_combinedRestitution = rb0->m_rig.getRestitution() * rb1->m_rig.getRestitution();
			newPt.m_combinedRollingFriction = rb0->m_rig.getRollingFriction() * rb1->m_rig.getRollingFriction();

			auto maniford = dispatcher->getNewManifold(&rb0->m_rig, &rb1->m_rig);
			maniford->addManifoldPoint(newPt);
		}
	}

	template <class T0, class T1>
	void SkinnedMeshAlgorithm::processCollision(T0* shape0, T1* shape1, MergeBuffer& merge, CollisionResult* collision)
	{
		int count;
		{
			BT_PROFILE("checkCollide");
			count = std::min(checkCollide(shape0, shape1, collision), MaxCollisionCount);
		}
		if (count > 0) {
			{
				BT_PROFILE("sort");
				// results come back in random order from parallel workers, sort so doMerge's
				// early break actually bails on shallow contacts instead of random ones
				std::sort(collision, collision + count, [](const CollisionResult& a, const CollisionResult& b) {
					return a.depth < b.depth;
				});
			}
			{
				BT_PROFILE("doMerge");
				merge.doMerge(shape0, shape1, collision, count);
			}
		}
	}

	void SkinnedMeshAlgorithm::processCollision(SkinnedMeshBody* body0, SkinnedMeshBody* body1,
		CollisionDispatcher* dispatcher)
	{
		BT_PROFILE("processCollision");

		// thread_local so we don't heap-alloc these 200+ times per frame.
		// MergeBuffer::resize() is O(1) after first call (generation counter, no zeroing).
		// Safe against TBB work-stealing re-entrancy: SkinnedMeshAlgorithm::processCollision
		// is called from CollisionCheckAlgorithm::operator() (hdtSkinnedMeshAlgorithm.cpp),
		// which wraps its inner parallel_for_each in tbb::this_task_arena::isolate.
		thread_local MergeBuffer merge;
		thread_local auto collision = std::make_unique<CollisionResult[]>(MaxCollisionCount);

		merge.resize(static_cast<int>(body0->m_skinnedBones.size()), static_cast<int>(body1->m_skinnedBones.size()));

		if (body0->m_shape->asPerTriangleShape() && body1->m_shape->asPerTriangleShape()) {
			// Todo: This can actually be further optimized, but would need a re-factor.. However, would the performance increase be worth
			// the extra boilerplate code..?
			processCollision(body0->m_shape->asPerTriangleShape(), body1->m_shape->asPerVertexShape(), merge,
				collision.get());
			processCollision(body0->m_shape->asPerVertexShape(), body1->m_shape->asPerTriangleShape(), merge,
				collision.get());
		} else if (body0->m_shape->asPerTriangleShape())
			processCollision(body0->m_shape->asPerTriangleShape(), body1->m_shape->asPerVertexShape(), merge,
				collision.get());
		else if (body1->m_shape->asPerTriangleShape())
			processCollision(body0->m_shape->asPerVertexShape(), body1->m_shape->asPerTriangleShape(), merge,
				collision.get());
		else
			processCollision(body0->m_shape->asPerVertexShape(), body1->m_shape->asPerVertexShape(), merge, collision.get());

		{
			BT_PROFILE("apply");
			merge.apply(body0, body1, dispatcher);
		}
	}
}
