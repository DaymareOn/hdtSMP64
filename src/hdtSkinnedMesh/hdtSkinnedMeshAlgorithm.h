#pragma once

#include "hdtDispatcher.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
	class SkinnedMeshAlgorithm : public btCollisionAlgorithm
	{
	public:
		SkinnedMeshAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);

		void processCollision([[maybe_unused]] const btCollisionObjectWrapper* body0Wrap, [[maybe_unused]] const btCollisionObjectWrapper* body1Wrap, [[maybe_unused]] const btDispatcherInfo& dispatchInfo, [[maybe_unused]] btManifoldResult* resultOut) override
		{
		}

		btScalar calculateTimeOfImpact([[maybe_unused]] btCollisionObject* body0, [[maybe_unused]] btCollisionObject* body1, [[maybe_unused]] const btDispatcherInfo& dispatchInfo, [[maybe_unused]] btManifoldResult* resultOut) override
		{
			return 1;
		}  // TOI cost too much
		void getAllContactManifolds([[maybe_unused]] btManifoldArray& manifoldArray) override
		{
		}

		struct CreateFunc : public btCollisionAlgorithmCreateFunc
		{
			btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
				[[maybe_unused]] const btCollisionObjectWrapper* body0Wrap,
				[[maybe_unused]] const btCollisionObjectWrapper* body1Wrap) override
			{
				void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(SkinnedMeshAlgorithm));
				return new (mem) SkinnedMeshAlgorithm(ci);
			}
		};

		static void registerAlgorithm(btCollisionDispatcherMt* dispatcher);

		// Note: It's possible to exceed this with complex outfits, which is why we cap it.
		// We don't want to stress a simulation island too much!
		static const int MaxCollisionCount = 256;

		static void processCollision(SkinnedMeshBody* body0Wrap, SkinnedMeshBody* body1Wrap,
			CollisionDispatcher* dispatcher);

	protected:
		struct CollisionMerge
		{
			btVector3 normal;  // accumulated weighted normal: length encodes depth, direction encodes contact normal
			btVector3 pos[2];
			float weight;

			CollisionMerge()
			{
				_mm_store_ps(((float*)this), _mm_setzero_ps());
				_mm_store_ps(((float*)this) + 4, _mm_setzero_ps());
				_mm_store_ps(((float*)this) + 8, _mm_setzero_ps());
				_mm_store_ps(((float*)this) + 12, _mm_setzero_ps());
			}

			void reset()
			{
				_mm_store_ps(((float*)this), _mm_setzero_ps());
				_mm_store_ps(((float*)this) + 4, _mm_setzero_ps());
				_mm_store_ps(((float*)this) + 8, _mm_setzero_ps());
				_mm_store_ps(((float*)this) + 12, _mm_setzero_ps());
			}
		};

		struct MergeBuffer
		{
			MergeBuffer() :
				mergeStride(0), mergeSize(0), currentGen(0)
			{
				activeCells.reserve(256);
			}

			~MergeBuffer()
			{
				std::free(buffer);
				std::free(generations);
			}

			MergeBuffer(const MergeBuffer&) = delete;
			MergeBuffer& operator=(const MergeBuffer&) = delete;

			// Just in case!
			MergeBuffer(MergeBuffer&&) = delete;
			MergeBuffer& operator=(MergeBuffer&&) = delete;

			// Buffer is ~2-10% occupied in basic scenes, so instead of zeroing old cells every resize() call we just bump a generation counter.
			// cells get lazily reset on first touch in getAndTrack(). Makes resize() O(1).
			void resize(int x, int y)
			{
				mergeStride = y;
				int needed = x * y;
				if (needed > mergeSize) {
					std::free(buffer);
					std::free(generations);
					mergeSize = needed;
					buffer = static_cast<CollisionMerge*>(std::malloc(needed * sizeof(CollisionMerge)));
					generations = static_cast<uint32_t*>(std::calloc(needed, sizeof(uint32_t)));
				}
				if (++currentGen == 0) {
					// wrap around, shouldn't realistically happen (~4 billion frames lol)
					// This is virtually skipped entirely by the cpu, 0 cost. Just in case since it would
					// create difficult to track down inconsistencies..
					std::memset(generations, 0, mergeSize * sizeof(uint32_t));
					currentGen = 1;
				}
				activeCells.clear();
			}

			CollisionMerge* get(int x, int y) { return &buffer[x * mergeStride + y]; }

			CollisionMerge* getAndTrack(int x, int y)
			{
				int idx = x * mergeStride + y;
				auto* c = &buffer[idx];
				if (generations[idx] != currentGen) {
					c->reset();
					generations[idx] = currentGen;
					activeCells.push_back(idx);
				}
				return c;
			}

			void doMerge(SkinnedMeshShape* shape0, SkinnedMeshShape* shape1, CollisionResult* collisions, int count);
			void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher);

			int mergeStride;
			int mergeSize;
			uint32_t currentGen;
			CollisionMerge* buffer = nullptr;
			uint32_t* generations = nullptr;
			std::vector<int> activeCells;
		};

		template <class T0, class T1>
		static void processCollision(T0* shape0, T1* shape1, MergeBuffer& merge, CollisionResult* collision);
	};
}
