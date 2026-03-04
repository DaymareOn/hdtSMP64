#pragma once

#include "hdtBulletHelper.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include <ppl.h>
#include <ppltasks.h>
#include <vector>

namespace hdt
{
	class SkinnedMeshBody;

	class CollisionDispatcher : public btCollisionDispatcherMt
	{
	public:

		CollisionDispatcher(btCollisionConfiguration* collisionConfiguration) : btCollisionDispatcherMt(
			collisionConfiguration)
		{
		}

		btPersistentManifold* getNewManifold(const btCollisionObject* b0, const btCollisionObject* b1) override
		{
			std::lock_guard<decltype(m_lock)> l(m_lock);
			auto ret = btCollisionDispatcherMt::getNewManifold(b0, b1);
			return ret;
		}

		void releaseManifold(btPersistentManifold* manifold) override
		{
			std::lock_guard<decltype(m_lock)> l(m_lock);
			btCollisionDispatcherMt::releaseManifold(manifold);
		}

		bool needsCollision(const btCollisionObject* body0, const btCollisionObject* body1) override;
		void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo,
		                               btDispatcher* dispatcher) override;

		int getNumManifolds() const override;
		btPersistentManifold** getInternalManifoldPointer() override;
		btPersistentManifold* getManifoldByIndexInternal(int index) override;

		void clearAllManifold();

		std::mutex m_lock;
		std::vector<std::pair<SkinnedMeshBody*, SkinnedMeshBody*>> m_pairs;
#ifdef CUDA
		std::vector<std::function<void()>> m_immediateFuncs;
		std::vector<std::function<void()>> m_delayedFuncs;
#endif
	};
}
