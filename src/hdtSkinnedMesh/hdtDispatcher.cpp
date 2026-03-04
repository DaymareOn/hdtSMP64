#include "hdtDispatcher.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshAlgorithm.h"
#ifdef CUDA
#include "hdtCudaInterface.h"
#include "hdtFrameTimer.h"
#endif

#include <LinearMath/btPoolAllocator.h>

#ifdef CUDA
// If defined, triangle-vertex and vertex-vertex collision results aren't applied until the next frame. This
// allows GPU collision detection to run concurrently with the rest of the game engine, instead of leaving
// the CPU idle waiting for the results. Triangle-triangle collisions are assumed to require the higher
// accuracy, and are always applied in the current frame.
#define CUDA_DELAYED_COLLISIONS
#endif

namespace hdt
{
	void CollisionDispatcher::clearAllManifold()
	{
		std::lock_guard<decltype(m_lock)> l(m_lock);
		for (int i = 0; i < m_manifoldsPtr.size(); ++i)
		{
			auto manifold = m_manifoldsPtr[i];
			manifold->~btPersistentManifold();
			if (m_persistentManifoldPoolAllocator->validPtr(manifold))
				m_persistentManifoldPoolAllocator->freeMemory(manifold);
			else
				btAlignedFree(manifold);
		}
		m_manifoldsPtr.clear();
	}

	bool needsCollision(const SkinnedMeshBody* shape0, const SkinnedMeshBody* shape1)
	{
		if (!shape0 || !shape1 || shape0 == shape1)
			return false;

		if (shape0->m_isKinematic && shape1->m_isKinematic)
			return false;

		return shape0->canCollideWith(shape1) && shape1->canCollideWith(shape0);
	}

	bool CollisionDispatcher::needsCollision(const btCollisionObject* body0, const btCollisionObject* body1)
	{
		auto shape0 = dynamic_cast<const SkinnedMeshBody*>(body0);
		auto shape1 = dynamic_cast<const SkinnedMeshBody*>(body1);

		if (shape0 || shape1)
		{
			return hdt::needsCollision(shape0, shape1);
		}
		if (body0->isStaticOrKinematicObject() && body1->isStaticOrKinematicObject())
			return false;
		if (body0->checkCollideWith(body1) || body1->checkCollideWith(body0))
		{
			auto rb0 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());
			auto rb1 = static_cast<SkinnedMeshBone*>(body0->getUserPointer());

			return rb0->canCollideWith(rb1) && rb1->canCollideWith(rb0);
		}
		else return false;
	}

	void CollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, [[maybe_unused]] const btDispatcherInfo& dispatchInfo, [[maybe_unused]] btDispatcher* dispatcher)
	{
		auto size = pairCache->getNumOverlappingPairs();
		if (!size) return;

		m_pairs.reserve(size);
		auto pairs = pairCache->getOverlappingPairArrayPtr();
#ifdef CUDA
		using UpdateMap = std::unordered_map<SkinnedMeshBody*, std::pair<PerVertexShape*, PerTriangleShape*> >;
		UpdateMap to_update;
		// Find bodies and meshes that need collision checking. We want to keep them together in a map so they can
		// be grouped by CUDA stream
		for (int i = 0; i < size; ++i)
#else
		SpinLock lock;
		std::unordered_set<SkinnedMeshBody*> bodies;
		std::unordered_set<PerVertexShape*> vertex_shapes;
		std::unordered_set<PerTriangleShape*> triangle_shapes;

		concurrency::parallel_for(0, size, [&](int i)
#endif
			{
				auto& pair = pairs[i];

				auto shape0 = dynamic_cast<SkinnedMeshBody*>(static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject));
				auto shape1 = dynamic_cast<SkinnedMeshBody*>(static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject));

				if (shape0 || shape1)
				{
					if (hdt::needsCollision(shape0, shape1) && shape0->isBoundingSphereCollided(shape1))
					{
#ifdef CUDA
						auto it0 = to_update.insert({ shape0, {nullptr, nullptr} }).first;
						auto it1 = to_update.insert({ shape1, {nullptr, nullptr} }).first;
#else
						HDT_LOCK_GUARD(l, lock);
						bodies.insert(shape0);
						bodies.insert(shape1);
#endif
						m_pairs.push_back(std::make_pair(shape0, shape1));

						auto a = shape0->m_shape->asPerTriangleShape();
						auto b = shape1->m_shape->asPerTriangleShape();
#ifdef CUDA
						if (a)
							it0->second.second = a;
						else
							it0->second.first = shape0->m_shape->asPerVertexShape();
						if (b)
							it1->second.second = b;
						else
							it1->second.first = shape1->m_shape->asPerVertexShape();
						if (a && b)
						{
							it0->second.first = a->m_verticesCollision;
							it1->second.first = b->m_verticesCollision;
						}
#else
						if (a)
							triangle_shapes.insert(a);
						else
							vertex_shapes.insert(shape0->m_shape->asPerVertexShape());
						if (b)
							triangle_shapes.insert(b);
						else
							vertex_shapes.insert(shape1->m_shape->asPerVertexShape());
						if (a && b)
						{
							vertex_shapes.insert(a->m_verticesCollision.get());
							vertex_shapes.insert(b->m_verticesCollision.get());
						}
#endif
					}
				}
				else getNearCallback()(pair, *this, dispatchInfo);
#ifdef CUDA
			}
		bool haveCuda = CudaInterface::instance()->hasCuda() && (!FrameTimer::instance()->running() || FrameTimer::instance()->cudaFrame());
		FrameTimer::instance()->logEvent(FrameTimer::e_Start);
		if (haveCuda)
		{
			bool initialized = true;
			int deviceId = CudaInterface::currentDevice;

			// Build simple vectors of the things to update, and determine whether any new CUDA objects need
			// to be created - either because there isn't one already, or because it's on the wrong device
			for (auto& o : to_update)
			{
				initialized &= static_cast<bool>(o.first->m_cudaObject) && o.first->m_cudaObject->deviceId() == deviceId;
				if (o.second.first)
				{
					initialized &= static_cast<bool>(o.second.first->m_cudaObject) && o.second.first->m_cudaObject->deviceId() == deviceId;
				}
				if (o.second.second)
				{
					initialized &= static_cast<bool>(o.second.second->m_cudaObject) && o.second.second->m_cudaObject->deviceId() == deviceId;
				}
			}

			// Create any new CUDA objects if necessary
			if (!initialized)
			{
				concurrency::parallel_for_each(to_update.begin(), to_update.end(), [deviceId](UpdateMap::value_type& o)
					{
						CudaInterface::instance()->setCurrentDevice();

						if (!o.first->m_cudaObject || o.first->m_cudaObject->deviceId() != deviceId)
						{
							o.first->m_cudaObject.reset(new CudaBody(o.first));
						}
						if (o.second.first && (!o.second.first->m_cudaObject || o.second.first->m_cudaObject->deviceId() != deviceId))
						{
							o.second.first->m_cudaObject.reset(new CudaPerVertexShape(o.second.first));
						}
						if (o.second.second && (!o.second.second->m_cudaObject || o.second.second->m_cudaObject->deviceId() != deviceId))
						{
							o.second.second->m_cudaObject.reset(new CudaPerTriangleShape(o.second.second));
						}
					});
			}

			// FIXME: This is probably broken if the current CUDA device changes and any tasks haven't finished yet.
			// But delayed collisions are disabled for now anyway.
			for (auto f : m_delayedFuncs)
			{
				f();
			}

			CudaInterface::instance()->setCurrentDevice();
			for (auto o : to_update)
			{
				o.first->updateBones();
				CudaInterface::launchInternalUpdate(
					o.first->m_cudaObject,
					o.second.first ? o.second.first->m_cudaObject : nullptr,
					o.second.second ? o.second.second->m_cudaObject : nullptr);
			}

			// Update the aggregate parts of the AABB trees
			for (auto o : to_update)
			{
				o.first->m_cudaObject->synchronize();

				if (o.second.first)
				{
					o.second.first->m_cudaObject->updateTree();
				}
				if (o.second.second)
				{
					o.second.second->m_cudaObject->updateTree();
				}
				o.first->m_bulletShape.m_aabb = o.first->m_shape->m_tree.aabbAll;
			}
		}
		else
		{
			concurrency::parallel_for_each(to_update.begin(), to_update.end(), [](UpdateMap::value_type& o)
				{
					o.first->internalUpdate();
					if (o.second.first)
					{
						o.second.first->internalUpdate();
					}
					if (o.second.second)
					{
						o.second.second->internalUpdate();
					}
					o.first->m_bulletShape.m_aabb = o.first->m_shape->m_tree.aabbAll;
				});
		}

		FrameTimer::instance()->logEvent(FrameTimer::e_Internal);
		m_delayedFuncs.clear();

		if (haveCuda)
		{
			CudaInterface::instance()->clearBufferPool();

			// Launch collision checking
			m_delayedFuncs.reserve(m_pairs.size());
			m_immediateFuncs.reserve(m_pairs.size());

			for (int i = 0; i < m_pairs.size(); ++i)
			{
				auto& pair = m_pairs[i];
				if (pair.first->m_shape->m_tree.collapseCollideL(&pair.second->m_shape->m_tree))
				{
					if (!pair.first->m_shape->asPerTriangleShape() || !pair.second->m_shape->asPerTriangleShape())
					{
						m_delayedFuncs.push_back(SkinnedMeshAlgorithm::queueCollision(pair.first, pair.second, this));
					}
					else if (pair.first->m_shape->asPerTriangleShape() && pair.second->m_shape->asPerTriangleShape())
					{
						m_immediateFuncs.push_back(SkinnedMeshAlgorithm::queueCollision(pair.first, pair.second, this));
					}
				}
			}

			FrameTimer::instance()->logEvent(FrameTimer::e_Launched);

			for (auto f : m_immediateFuncs)
			{
				f();
			}
			m_immediateFuncs.clear();
#ifndef CUDA_DELAYED_COLLISIONS
			for (auto f : m_delayedFuncs)
			{
				f();
			}
			m_delayedFuncs.clear();
#endif
		}
		else
		{
			// Now we can process the collisions
			concurrency::parallel_for_each(m_pairs.begin(), m_pairs.end(),
				[this](std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i)
				{
					if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
					{
						SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
					}
				});
			FrameTimer::instance()->logEvent(FrameTimer::e_Launched);
		}

		m_pairs.clear();

		FrameTimer::instance()->addManifoldCount(getNumManifolds());
		FrameTimer::instance()->logEvent(FrameTimer::e_End);
	}
#else
			});
		concurrency::parallel_for_each(bodies.begin(), bodies.end(), [](SkinnedMeshBody* shape)
		{
			shape->internalUpdate();
		});
		concurrency::parallel_for_each(vertex_shapes.begin(), vertex_shapes.end(), [](PerVertexShape* shape)
		{
			shape->internalUpdate();
		});
		concurrency::parallel_for_each(triangle_shapes.begin(), triangle_shapes.end(), [](PerTriangleShape* shape)
		{
			shape->internalUpdate();
		});
		for (auto body : bodies)
		{
			body->m_bulletShape.m_aabb = body->m_shape->m_tree.aabbAll;
		}
		concurrency::parallel_for_each(m_pairs.begin(), m_pairs.end(), [&, this](const std::pair<SkinnedMeshBody*, SkinnedMeshBody*>& i)
		{
			if (i.first->m_shape->m_tree.collapseCollideL(&i.second->m_shape->m_tree))
				SkinnedMeshAlgorithm::processCollision(i.first, i.second, this);
		});
		m_pairs.clear();
	}
#endif

	int CollisionDispatcher::getNumManifolds() const
	{
		return m_manifoldsPtr.size();
	}

	btPersistentManifold* CollisionDispatcher::getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	btPersistentManifold** CollisionDispatcher::getInternalManifoldPointer()
	{
		return btCollisionDispatcherMt::getInternalManifoldPointer();
	}
}
