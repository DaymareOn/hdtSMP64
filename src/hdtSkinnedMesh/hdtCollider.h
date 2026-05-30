#pragma once

#include "hdtAABB.h"
#include <functional>

namespace hdt
{
	static const int MaxCollisionPairs = 4024;

	// Collider-tree depth cap. insertCollider descends one level per skinning bone (sorted by weight),
	// so this bounds how finely colliders are bucketed into leaves. Lower values give fewer, larger
	// leaves => fewer broad-phase node-pairs, but looser per-leaf AABBs and a larger listA x listB
	// narrow-phase per pair. Profiling (depths 2/3/4) found 4 is the sweet spot: depth 3 was within
	// noise and depth 2 was markedly worse (per-pair narrow-phase cost roughly tripled, outweighing
	// the ~20% pair-count reduction). Change here and recompile to re-test.
	static constexpr size_t MaxTreeDepth = 4;

	struct alignas(16) Collider
	{
		Collider()
		{
		}

		Collider(int i0) { vertex = i0; }
		Collider(int i0, int i1, int i2) { vertices[0] = i0, vertices[1] = i1, vertices[2] = i2; }
		Collider(const Collider& rhs) { operator=(rhs); }

		Collider& operator=(const Collider& rhs)
		{
			__m128i* dst = (__m128i*)this;
			__m128i* src = (__m128i*)&rhs;
			auto xmm0 = _mm_load_si128(src + 0);
			_mm_store_si128(dst, xmm0);
			return *this;
		}

		union
		{
			U32 vertex;       // vertexshape
			U32 vertices[3];  // triangleshape
		};

		float flexible;
		//		inline bool operator <(const Collider& rhs){ return aligned < rhs.aligned; }
	};

	struct alignas(16) ColliderTree
	{
		ColliderTree()
		{
			aabbAll.invalidate();
			aabbMe.invalidate();
		}

		ColliderTree(U32 k) :
			key(k)
		{
			aabbAll.invalidate();
			aabbMe.invalidate();
		}

		Aabb aabbAll;
		Aabb aabbMe;

		U32 isKinematic;

		Collider* cbuf = nullptr;
		Aabb* aabb;
		U32 numCollider;
		U32 dynCollider;

		U32 dynChild;
		vectorA16<ColliderTree> children;

		vectorA16<Collider> colliders;
		U32 key;

		void insertCollider(const U32* keys, size_t keyCount, const Collider& c);
		void exportColliders(vectorA16<Collider>& exportTo);
		void remapColliders(Collider* start, Aabb* startAabb);

		void checkCollisionL(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret);
		void checkCollisionR(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret);
		void clipCollider(const std::function<bool(const Collider&)>& func);
		void updateKinematic(const std::function<float(const Collider*)>& func);
		void visitColliders(const std::function<void(Collider*)>& func);
		void updateAabb();
		void optimize();

		bool empty() const { return children.empty() && colliders.empty(); }

		bool collapseCollideL(ColliderTree* r);
		bool collapseCollideR(ColliderTree* r);
	};
}
