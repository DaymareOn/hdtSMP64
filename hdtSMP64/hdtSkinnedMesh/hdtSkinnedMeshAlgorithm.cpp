#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtCollider.h"

namespace hdt
{
	SkinnedMeshAlgorithm::SkinnedMeshAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
		: btCollisionAlgorithm(ci)
	{
	}

	static const CollisionResult zero;

	// Algorithm selection for collision checking.
	// e_CPU is the original one, optimized for CPU performance.
	// e_CPURefactored is an alternate CPU one, modified for conversion to GPU but still using CPU in practice.
	// e_CUDA will (eventually) be an actual GPGPU algorithm.
	enum CollisionCheckAlgorithmType
	{
		e_CPU,
		e_CPURefactored,
		e_CUDA
	};


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
			results = r;
			numResults = 0;
		}

		VertexPos* v0;
		VertexPos* v1;
		ColliderTree* c0;
		ColliderTree* c1;
		SP0* sp0;
		SP1* sp1;

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
		CollisionCheckBase2(Ts&&... ts)
			: CollisionCheckBase1(std::forward<Ts>(ts)...)
		{}

		bool addResult(const CollisionResult& res)
		{
			int p = numResults.fetch_add(1);
			if (p < SkinnedMeshAlgorithm::MaxCollisionCount)
			{
				results[p] = res;
				return true;
			}
			return false;
		}
	};

	template <typename T>
	struct CollisionCheckBase2<T, true> : public CollisionCheckBase1<T>
	{
		template <typename... Ts>
		CollisionCheckBase2(Ts&&... ts)
			: CollisionCheckBase1(std::forward<Ts>(ts)...)
		{}

		bool addResult(const CollisionResult& res)
		{
			int p = numResults.fetch_add(1);
			if (p < SkinnedMeshAlgorithm::MaxCollisionCount)
			{
				results[p].posA = res.posB;
				results[p].posB = res.posA;
				results[p].colliderA = res.colliderB;
				results[p].colliderB = res.colliderA;
				results[p].normOnB = -res.normOnB;
				results[p].depth = res.depth;
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
		CollisionChecker(Ts&&... ts)
			: CollisionCheckBase2(std::forward<Ts>(ts)...)
		{}
		bool checkCollide(Collider* a, Collider* b, CollisionResult& res)
		{
			auto s0 = v0[a->vertex];
			auto r0 = s0.marginMultiplier() * sp0->margin;
			auto s1 = v0[a->vertex];
			auto r1 = s1.marginMultiplier() * sp0->margin;

			auto ret = checkSphereSphere(s0.pos(), s1.pos(), r0, r1, res);
			res.colliderA = a;
			res.colliderB = b;
			return ret;
		}
	};

	template <bool SwapResults>
	struct CollisionChecker<PerTriangleShape, SwapResults> : public CollisionCheckBase2<PerTriangleShape, SwapResults>
	{
		template <typename... Ts>
		CollisionChecker(Ts&&... ts)
			: CollisionCheckBase2(std::forward<Ts>(ts)...)
		{}

		bool checkCollide(Collider* a, Collider* b, CollisionResult& res)
		{
			auto s = v0[a->vertex];
			auto r = s.marginMultiplier() * sp0->margin;
			auto p0 = v1[b->vertices[0]];
			auto p1 = v1[b->vertices[1]];
			auto p2 = v1[b->vertices[2]];
			auto margin = (p0.marginMultiplier() + p1.marginMultiplier() + p2.marginMultiplier()) / 3;
			auto penetration = sp1->penetration * margin;
			margin *= sp1->margin;

			CheckTriangle tri(p0.pos(), p1.pos(), p2.pos(), margin, penetration);
			if (!tri.valid) return false;
			auto ret = checkSphereTriangle(s.pos(), r, tri, res);
			res.colliderA = a;
			res.colliderB = b;
			return ret;
		}
	};

	// CollisionCheckDispatcher provides a dispatch method to process two lists of colliders. It is needed for
	// the new (GPU-oriented) algorithm, but we provide a CPU-only version as well.
	template <typename T, bool SwapResults, CollisionCheckAlgorithmType Algorithm>
	struct CollisionCheckDispatcher : public CollisionChecker<T, SwapResults>
	{
		template <typename... Ts>
		CollisionCheckDispatcher(Ts&&... ts)
			: CollisionChecker(std::forward<Ts>(ts)...)
		{}

		void dispatch(ColliderTree* a, ColliderTree* b, const std::vector<Aabb*>& listA, const std::vector<Aabb*>& listB)
		{
			CollisionResult result;
			CollisionResult temp;
			bool hasResult = false;

			auto abeg = a->aabb;
			auto bbeg = b->aabb;

			if (listA.size() && listB.size())
			{
				for (auto i : listA)
				{
					for (auto j : listB)
					{
						if (!i->collideWith(*j))
							continue;
						if (checkCollide(&a->cbuf[i - abeg], &b->cbuf[j - bbeg], temp))
						{
							if (!hasResult || result.depth > temp.depth)
							{
								hasResult = true;
								result = temp;
							}
						}
					}
				}
			}

			if (hasResult)
			{
				addResult(result);
			}
		}
	};

	// Dispatcher specialization for sphere-triangle collisions on CUDA. Sphere-sphere collisions will
	// continue to use the CPU dispatcher. Doesn't actually do anything yet (and will fail to compile).
	template <bool SwapResults>
	struct CollisionCheckDispatcher<PerTriangleShape, SwapResults, e_CUDA>
		: public CollisionCheckBase2<PerTriangleShape, SwapResults>
	{

	};

	// Finally, CollisionCheckAlgorithm does the full check between collider trees.
	template <typename T, bool SwapResults = false, CollisionCheckAlgorithmType Algorithm = e_CPURefactored>
	struct CollisionCheckAlgorithm : public CollisionCheckDispatcher<T, SwapResults, Algorithm>
	{
		template <typename... Ts>
		CollisionCheckAlgorithm(Ts&&... ts)
			: CollisionCheckDispatcher(std::forward<Ts>(ts)...)
		{}

		int operator()()
		{
			static_assert(Algorithm != e_CPU, "Old CPU algorithm specialization missing");

			std::vector<std::pair<ColliderTree*, ColliderTree*>> pairs;
			pairs.reserve(c0->colliders.size() + c1->colliders.size());
			c0->checkCollisionL(c1, pairs);
			if (pairs.empty()) return 0;

			decltype(auto) func = [this](const std::pair<ColliderTree*, ColliderTree*>& pair)
			{
				if (numResults >= SkinnedMeshAlgorithm::MaxCollisionCount)
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

				listA.reserve(asize);
				listB.reserve(bsize);

				// Colliders in A that intersect full bounding box of B. Compute a new bounding box for just those - this
				// can be MUCH smaller than the original bounding box for A (consider the case where we have two spheres
				// colliding, offset by an equal amount in all three axes).
				for (auto i = abeg; i < aend; ++i)
				{
					if (i->collideWith(aabbB))
					{
						listA.push_back(i);
						aabbA.merge(*i);
					}
				}

				// Colliders in B that intersect the new bounding box for A. Compute a new bounding box for those too.
				if (listA.size())
				{
					aabbB.invalidate();
					for (auto i = bbeg; i < bend; ++i)
					{
						if (i->collideWith(aabbA))
						{
							listB.push_back(i);
							aabbB.merge(*i);
						}
					}
				}

				// Remove any colliders from A that don't intersect the new bounding box for B
				if (listB.size())
				{
					listA.erase(std::remove_if(listA.begin(), listA.end(), [&](Aabb* aabb) { return !aabb->collideWith(aabbB); }), listA.end());
				}

				// Now go through both lists and do the real collision (if needed).
				dispatch(a, b, listA, listB);

				listA.clear();
				listB.clear();
			};

			if (pairs.size() >= std::thread::hardware_concurrency())
				concurrency::parallel_for_each(pairs.begin(), pairs.end(), func);
			else for (auto& i : pairs) func(i);

			return numResults;
		}
	};

	// Old algorithm - lower memory use, possibly faster (for CPU), but not at all suited to GPU processing
	template <typename T, bool SwapResults>
	struct CollisionCheckAlgorithm<T, SwapResults, e_CPU> : public CollisionChecker<T, SwapResults>
	{
		template <typename... Ts>
		CollisionCheckAlgorithm(Ts&&... ts)
			: CollisionChecker(std::forward<Ts>(ts)...)
		{}

		int operator()()
		{
			std::vector<std::pair<ColliderTree*, ColliderTree*>> pairs;
			pairs.reserve(c0->colliders.size() + c1->colliders.size());
			c0->checkCollisionL(c1, pairs);
			if (pairs.empty()) return 0;

			decltype(auto) func = [this](const std::pair<ColliderTree*, ColliderTree*>& pair)
			{
				if (numResults >= SkinnedMeshAlgorithm::MaxCollisionCount)
					return;

				auto a = pair.first, b = pair.second;

				auto aabbA = a->aabbMe;
				auto aabbB = b->aabbMe;
				auto abeg = a->aabb;
				auto bbeg = b->aabb;
				auto asize = b->isKinematic ? a->dynCollider : a->numCollider;
				auto bsize = a->isKinematic ? b->dynCollider : b->numCollider;
				auto aend = abeg + asize;
				auto bend = bbeg + bsize;

				CollisionResult result;
				CollisionResult temp;
				bool hasResult = false;

				thread_local std::vector<Aabb*> list;
				if (asize > bsize)
				{
					list.reserve(std::max<size_t>(bsize, list.capacity()));
					for (auto i = bbeg; i < bend; ++i)
					{
						if (i->collideWith(aabbA))
							list.push_back(i);
					}

					for (auto i = abeg; i < aend; ++i)
					{
						if (!i->collideWith(aabbB))
							continue;

						for (auto j : list)
						{
							if (!i->collideWith(*j))
								continue;
							if (checkCollide(&a->cbuf[i - abeg], &b->cbuf[j - bbeg], temp))
							{
								if (!hasResult || result.depth > temp.depth)
								{
									hasResult = true;
									result = temp;
								}
							}
						}
					}
				}
				else
				{
					list.reserve(std::max<size_t>(bsize, list.capacity()));
					for (auto i = abeg; i < aend; ++i)
					{
						if (i->collideWith(aabbB))
							list.push_back(i);
					}

					for (auto j = bbeg; j < bend; ++j)
					{
						if (!j->collideWith(aabbA))
							continue;

						for (auto i : list)
						{
							if (!i->collideWith(*j))
								continue;
							if (checkCollide(&a->cbuf[i - abeg], &b->cbuf[j - bbeg], temp))
							{
								if (!hasResult || result.depth > temp.depth)
								{
									hasResult = true;
									result = temp;
								}
							}
						}
					}
				}
				list.clear();

				if (hasResult)
				{
					addResult(result);
				}
			};

			if (pairs.size() >= std::thread::hardware_concurrency())
				concurrency::parallel_for_each(pairs.begin(), pairs.end(), func);
			else for (auto& i : pairs) func(i);

			return numResults;
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

	void SkinnedMeshAlgorithm::MergeBuffer::doMerge(SkinnedMeshShape* a, SkinnedMeshShape* b,
	                                                CollisionResult* collision, int count)
	{
		for (int i = 0; i < count; ++i)
		{
			auto& res = collision[i];
			if (res.depth >= -FLT_EPSILON) break;

			auto flexible = std::max(res.colliderA->flexible, res.colliderB->flexible);
			if (flexible < FLT_EPSILON) return;

			for (int ib = 0; ib < a->getBonePerCollider(); ++ib)
			{
				auto w0 = a->getColliderBoneWeight(res.colliderA, ib);
				int boneIdx0 = a->getColliderBoneIndex(res.colliderA, ib);
				if (w0 <= a->m_owner->m_skinnedBones[boneIdx0].weightThreshold) continue;

				for (int jb = 0; jb < b->getBonePerCollider(); ++jb)
				{
					auto w1 = b->getColliderBoneWeight(res.colliderB, jb);
					int boneIdx1 = b->getColliderBoneIndex(res.colliderB, jb);
					if (w1 <= b->m_owner->m_skinnedBones[boneIdx1].weightThreshold) continue;

					if (a->m_owner->m_skinnedBones[boneIdx0].isKinematic && b->m_owner->m_skinnedBones[boneIdx1].
						isKinematic)
						continue;

					float w = flexible * res.depth;
					float w2 = w * w;
					auto c = get(boneIdx0, boneIdx1);
					c->weight += w2;
					c->normal += res.normOnB * w * w2;
					c->pos[0] += res.posA * w2;
					c->pos[1] += res.posB * w2;
				}
			}
		}
	}

	void SkinnedMeshAlgorithm::MergeBuffer::apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1,
	                                              CollisionDispatcher* dispatcher)
	{
		for (int i = 0; i < body0->m_skinnedBones.size(); ++i)
		{
			if (!body1->canCollideWith(body0->m_skinnedBones[i].ptr)) continue;
			for (int j = 0; j < body1->m_skinnedBones.size(); ++j)
			{
				if (!body0->canCollideWith(body1->m_skinnedBones[j].ptr)) continue;
				if (get(i, j)->weight < FLT_EPSILON) continue;

				if (body0->m_skinnedBones[i].isKinematic && body1->m_skinnedBones[j].isKinematic) continue;

				auto rb0 = body0->m_skinnedBones[i].ptr;
				auto rb1 = body1->m_skinnedBones[j].ptr;
				if (rb0 == rb1) continue;

				auto c = get(i, j);
				float invWeight = 1.0f / c->weight;

				auto maniford = dispatcher->getNewManifold(&rb0->m_rig, &rb1->m_rig);
				auto worldA = c->pos[0] * invWeight;
				auto worldB = c->pos[1] * invWeight;
				auto localA = rb0->m_rig.getWorldTransform().invXform(worldA);
				auto localB = rb1->m_rig.getWorldTransform().invXform(worldB);
				auto normal = c->normal * invWeight;
				if (normal.fuzzyZero()) continue;
				auto depth = -normal.length();
				normal = -normal.normalized();

				if (depth >= -FLT_EPSILON) continue;

				btManifoldPoint newPt(localA, localB, normal, depth);
				newPt.m_positionWorldOnA = worldA;
				newPt.m_positionWorldOnB = worldB;
				newPt.m_combinedFriction = rb0->m_rig.getFriction() * rb1->m_rig.getFriction();
				newPt.m_combinedRestitution = rb0->m_rig.getRestitution() * rb1->m_rig.getRestitution();
				newPt.m_combinedRollingFriction = rb0->m_rig.getRollingFriction() * rb1->m_rig.getRollingFriction();
				maniford->addManifoldPoint(newPt);
			}
		}
	}

	template <class T0, class T1>
	void SkinnedMeshAlgorithm::processCollision(T0* shape0, T1* shape1, MergeBuffer& merge, CollisionResult* collision)
	{
		int count = std::min(checkCollide(shape0, shape1, collision), MaxCollisionCount);
		if (count > 0)
			merge.doMerge(shape0, shape1, collision, count);
	}

	void SkinnedMeshAlgorithm::processCollision(SkinnedMeshBody* body0, SkinnedMeshBody* body1,
	                                            CollisionDispatcher* dispatcher)
	{
		MergeBuffer merge;
		merge.alloc(body0->m_skinnedBones.size(), body1->m_skinnedBones.size());

		auto collision = new CollisionResult[MaxCollisionCount];
		if (body0->m_shape->asPerTriangleShape() && body1->m_shape->asPerTriangleShape())
		{
			processCollision(body0->m_shape->asPerTriangleShape(), body1->m_shape->asPerVertexShape(), merge,
			                 collision);
			processCollision(body0->m_shape->asPerVertexShape(), body1->m_shape->asPerTriangleShape(), merge,
			                 collision);
		}
		else if (body0->m_shape->asPerTriangleShape())
			processCollision(body0->m_shape->asPerTriangleShape(), body1->m_shape->asPerVertexShape(), merge,
			                 collision);
		else if (body1->m_shape->asPerTriangleShape())
			processCollision(body0->m_shape->asPerVertexShape(), body1->m_shape->asPerTriangleShape(), merge,
			                 collision);
		else processCollision(body0->m_shape->asPerVertexShape(), body1->m_shape->asPerVertexShape(), merge, collision);

		delete[] collision;
		merge.apply(body0, body1, dispatcher);
		merge.release();
	}

	void SkinnedMeshAlgorithm::registerAlgorithm(btCollisionDispatcher* dispatcher)
	{
		static CreateFunc s_gimpact_cf;
		dispatcher->registerCollisionCreateFunc(CUSTOM_CONCAVE_SHAPE_TYPE, CUSTOM_CONCAVE_SHAPE_TYPE, &s_gimpact_cf);
	}
}
