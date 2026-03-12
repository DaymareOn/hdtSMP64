#pragma once

#include "hdtDispatcher.h"
#include "hdtSkinnedMeshShape.h"
#ifdef CUDA
#	include "hdtCudaInterface.h"

// Define this to do actual collision checking on GPU. This is currently slow and has very inconsistent
// framerate. If not defined, the GPU will still be used if available for vertex and bounding box
// calculations, but collision will be done on the CPU.
#	define USE_GPU_COLLISION
#endif

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

		static const int MaxCollisionCount = 256;

#ifdef CUDA
		static std::function<void()> queueCollision(
			SkinnedMeshBody* body0Wrap,
			SkinnedMeshBody* body1Wrap,
			CollisionDispatcher* dispatcher);
#endif

		static void processCollision(SkinnedMeshBody* body0Wrap, SkinnedMeshBody* body1Wrap,
			CollisionDispatcher* dispatcher);

	protected:
		struct CollisionMerge
		{
			btVector3 normal;
			btVector3 pos[2];
			float weight;

			CollisionMerge()
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
				mergeStride(0), mergeSize(0) {}

			MergeBuffer(int x, int y) :
				mergeStride(y), mergeSize(x * y), buffer(std::make_unique<CollisionMerge[]>(x * y))
			{}

			~MergeBuffer() = default;

			CollisionMerge* begin() const { return buffer.get(); }
			CollisionMerge* end() const { return buffer.get() + mergeSize; }
			CollisionMerge* get(int x, int y) { return &buffer[x * mergeStride + y]; }

			void doMerge(SkinnedMeshShape* shape0, SkinnedMeshShape* shape1, CollisionResult* collisions, int count);
			void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher);

			int mergeStride;
			int mergeSize;
			std::unique_ptr<CollisionMerge[]> buffer;
		};

		template <class T0, class T1>
		static void processCollision(T0* shape0, T1* shape1, MergeBuffer& merge, CollisionResult* collision);
	};
}
