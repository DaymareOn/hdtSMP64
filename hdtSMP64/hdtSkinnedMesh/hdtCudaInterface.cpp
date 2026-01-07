#ifdef CUDA
#include "hdtCudaInterface.h"

#include <ppl.h>
#include <immintrin.h>
#include <type_traits>

struct cudaStream_t;

#include "hdtCudaCollision.cuh"

namespace hdt
{
	namespace
	{
		template<typename T>
		struct NullDeleter
		{
			void operator()(T*) const {}

			template<typename U>
			void operator()(U*) const {}
		};

		class CudaStream
		{
		public:
			CudaStream()
			{
				cuCreateStream(&m_stream).check(__FUNCTION__);
			}

			~CudaStream()
			{
				cuDestroyStream(m_stream);
			}

			void* get() { return m_stream; }
			operator void* () { return m_stream; }

		private:
			void* m_stream;
		};

		class CudaEvent
		{
		public:
			CudaEvent()
			{
				cuCreateEvent(&m_event).check(__FUNCTION__);
			}

			~CudaEvent()
			{
				cuDestroyEvent(m_event);
			}

			void record(CudaStream& stream)
			{
				cuRecordEvent(m_event, stream);
			}

			void wait()
			{
				cuWaitEvent(m_event);
			}

		private:
			void* m_event;
		};

		// CUDA buffer for long-lived objects
		template <typename CudaT, typename HostT = CudaT>
		class CudaBuffer
		{
		public:

			CudaBuffer(int n)
				: m_size(n * sizeof(CudaT))
			{
				static_assert(sizeof(CudaT) == sizeof(HostT), "Device and host types different sizes");
				cuGetDeviceBuffer(&reinterpret_cast<void*>(m_deviceData), m_size).check(__FUNCTION__);
				cuGetHostBuffer(&reinterpret_cast<void*>(m_hostData), m_size).check(__FUNCTION__);
				m_zeroCopyData = reinterpret_cast<CudaT*>(cuDevicePointer(m_hostData));
			}

			~CudaBuffer()
			{
				cuFreeDevice(m_deviceData);
				cuFreeHost(m_hostData);
			}

			void toDevice(CudaStream& stream)
			{
				cuCopyToDevice(m_deviceData, m_hostData, m_size, stream).check(__FUNCTION__);
			}

			void toHost(CudaStream& stream)
			{
				cuCopyToHost(m_hostData, m_deviceData, m_size, stream).check(__FUNCTION__);
			}

			operator HostT* () { return m_hostData; }
			HostT* get() { return m_hostData; }

			CudaT* getD() { return m_deviceData; }

			CudaT* getZ() { return m_zeroCopyData; }

		private:

			int m_size;
			CudaT* m_deviceData;
			HostT* m_hostData;
			CudaT* m_zeroCopyData;
		};

		template <typename DeviceT, typename... DeviceArgs, typename HostT, typename... HostArgs>
		class CudaBuffer<ArrayType<DeviceT, DeviceArgs...>, ArrayType<HostT, HostArgs...>>
		{
		public:

			CudaBuffer(int n)
				: m_size(n),
				m_allocatedSize(32 * (((n - 1) / 32) + 1)),
				m_buffer(m_allocatedSize)
			{}

			void toDevice(CudaStream& stream)
			{
				m_buffer.toDevice(stream);
			}

			void toHost(CudaStream& stream)
			{
				m_buffer.toHost(stream);
			}

			ArrayType<HostT, HostArgs...> get()
			{
				return { m_buffer.get(), m_allocatedSize };
			}

			ArrayType<DeviceT, DeviceArgs...> getD()
			{
				return { m_buffer.getD(), m_allocatedSize };
			}

			ArrayType<DeviceT, DeviceArgs...> getZ()
			{
				return { m_buffer.getZ(), m_allocatedSize };
			}

		private:

			int m_size;
			int m_allocatedSize;
			CudaBuffer<HostT, DeviceT> m_buffer;
		};

		template <typename CudaT>
		class CudaDeviceBuffer
		{
		public:

			CudaDeviceBuffer(int n)
				: m_size(n * sizeof(CudaT))
			{
				cuGetDeviceBuffer(&reinterpret_cast<void*>(m_deviceData), m_size);
			}

			~CudaDeviceBuffer()
			{
				cuFreeDevice(m_deviceData);
			}

			CudaT* getD() { return m_deviceData; }

		private:

			int m_size;
			CudaT* m_deviceData;
		};

		template <typename T, typename... Ts>
		class CudaDeviceBuffer<ArrayType<T, Ts...>>
		{
		public:

			CudaDeviceBuffer(int n)
				: m_size(32 * (((n - 1) / 32) + 1)),
				m_buffer(m_size)
			{}

			ArrayType<T, Ts...> getD() {
				return ArrayType<T, Ts...>(m_buffer.getD(), m_size);
			}

		private:

			int m_size;
			CudaDeviceBuffer<T> m_buffer;
		};

		// Memory pool for small short-lived objects. This can grow arbitrarily in size, to the maximum required
		// in a single frame. All allocations get cleared at the end of the frame.
		class CudaBufferPool
		{
			using Buffers = std::pair<void*, void*>;
			using Record = std::tuple<size_t, const size_t, Buffers>;

			// Granularity for allocating blocks that won't fit a single page (however, this memory pool is
			// REALLY not designed for large allocations and using them is likely to leak memory badly)
			static constexpr size_t largeBlockSize = 1 << 20;

			// Page size for normal allocations
			static constexpr size_t pageSize = 1 << 24;

			// Granularity of small allocations, should match CUDA memory transaction size
			static constexpr size_t alignment = 128;

		public:

			CudaBufferPool()
			{}

			~CudaBufferPool()
			{
				for (auto record : m_buffers)
				{
					cuFreeDevice(std::get<2>(record).first);
					cuFreeHost(std::get<2>(record).second);
				}
			}

			// FIXME: Not thread safe
			static CudaBufferPool* instance()
			{
				return &s_pools[cuGetDevice()];
			}

			std::pair<void*, void*> getBuffer(size_t size)
			{
				// FIXME: Locking for the whole method is lazy - should do something finer grained
				std::lock_guard l(m_lock);

				auto s = getSize(size);
				std::vector<Record>::iterator it;
				for (it = m_buffers.begin(); it != m_buffers.end(); ++it)
				{
					if (std::get<0>(*it) + s <= std::get<1>(*it))
					{
						break;
					}
				}
				if (it == m_buffers.end())
				{
					size_t newSize = std::max(pageSize, blockSize(size));
					m_buffers.push_back({ 0, newSize, {0,0} });
					cuGetDeviceBuffer(&(std::get<2>(m_buffers.back()).first), newSize).check(__FUNCTION__);
					cuGetHostBuffer(&(std::get<2>(m_buffers.back()).second), newSize).check(__FUNCTION__);
					it = m_buffers.end() - 1;
				}
				Buffers result = {
					static_cast<uint8_t*>(std::get<2>(*it).first) + std::get<0>(*it),
					static_cast<uint8_t*>(std::get<2>(*it).second) + std::get<0>(*it)
				};
				std::get<0>(*it) += s;
				return result;
			}

			void clear()
			{
				for (auto& record : m_buffers)
				{
					std::get<0>(record) = 0;
				}
			}

		private:

			constexpr size_t getSize(size_t size)
			{
				return alignment * ((size - 1) / alignment + 1);
			}

			constexpr size_t blockSize(size_t size)
			{
				return largeBlockSize * ((size - 1) / largeBlockSize + 1);
			}

			std::vector<Record> m_buffers;
			std::mutex m_lock;

			static std::map<int, CudaBufferPool> s_pools;
		};

		std::map<int, CudaBufferPool> CudaBufferPool::s_pools = std::map<int, CudaBufferPool>();

		// CUDA buffer for short-lived per-frame objects. There is no way to deallocate these explicitly - they
		// remain until the buffer pool is cleared manually at the end of the frame, and then all become unsafe.
		template <typename CudaT, typename HostT = CudaT>
		class CudaPooledBuffer
		{
		public:

			CudaPooledBuffer(size_t n)
				: m_size(n * sizeof(CudaT))
			{
				static_assert(sizeof(CudaT) == sizeof(HostT), "Device and host types different sizes");
				auto buffers = CudaBufferPool::instance()->getBuffer(m_size);
				m_deviceData = reinterpret_cast<CudaT*>(buffers.first);
				m_hostData = reinterpret_cast<HostT*>(buffers.second);
				m_zeroCopyData = reinterpret_cast<CudaT*>(cuDevicePointer(m_hostData));
			}

			void toDevice(CudaStream& stream)
			{
				cuCopyToDevice(m_deviceData, m_hostData, m_size, stream).check(__FUNCTION__);
			}

			void toHost(CudaStream& stream)
			{
				cuCopyToHost(m_hostData, m_deviceData, m_size, stream).check(__FUNCTION__);
			}

			void zero(CudaStream& stream)
			{
				cuMemset(m_deviceData, 0, m_size, stream).check(__FUNCTION__);
			}

			operator HostT* () { return m_hostData; }
			HostT* get() { return m_hostData; }

			CudaT* getD() { return m_deviceData; }

			CudaT* getZ() { return m_zeroCopyData; }

		private:

			size_t m_size;
			CudaT* m_deviceData;
			HostT* m_hostData;
			CudaT* m_zeroCopyData;
		};
	}

	class CudaBody::Imp
	{
	public:

		Imp(SkinnedMeshBody* body)
			: m_device(cuGetDevice()),
			m_numVertices(body->m_vertices.size()),
			m_numDynamicBones(0),
			m_bones(body->m_skinnedBones.size()),
			m_boneWeights(body->m_skinnedBones.size()),
			m_boneMap(body->m_skinnedBones.size()),
			m_vertexData(body->m_vertices.size()),
			m_vertexBuffer(body->m_vertices.size())
		{
			// Copy vertex data to the GPU, converting to homogeneous coordinates with w=1
			std::copy(body->m_vertices.begin(), body->m_vertices.end(), m_vertexData.get());
			for (int i = 0; i < m_numVertices; ++i)
			{
				m_vertexData[i].m_skinPos[3] = 1.0f;
			}
			m_vertexData.toDevice(m_stream);

			m_invBoneMap.reserve(body->m_skinnedBones.size());
			for (int i = 0; i < body->m_skinnedBones.size(); ++i)
			{
				m_boneWeights[i] = body->m_skinnedBones[i].weightThreshold;
				if (!body->m_skinnedBones[i].isKinematic)
				{
					m_boneMap[i] = m_numDynamicBones++;
					m_invBoneMap.push_back(i);
				}
				else
				{
					m_boneMap[i] = -1;
				}
			}
			m_boneWeights.toDevice(m_stream);
			m_boneMap.toDevice(m_stream);

			body->m_bones.reset(m_bones.get(), NullDeleter<Bone[]>());
		}

		void synchronize()
		{
			cuSynchronize(m_stream).check(__FUNCTION__);
		}

		int deviceId()
		{
			return m_device;
		}

		operator cuBodyData()
		{
			return { m_vertexData.getD(), m_vertexBuffer.getD(), m_numVertices };
		}

		operator cuCollisionBodyData()
		{
			return { m_vertexData.getD(), m_vertexBuffer.getD(), m_boneWeights.getD(), m_boneMap.getD() };
		}

		int m_device;
		CudaStream m_stream;
		CudaDeviceBuffer<cuVector4> m_vertexBuffer;
		CudaBuffer<cuVertex, Vertex> m_vertexData;
		CudaBuffer<float> m_boneWeights;
		CudaBuffer<int> m_boneMap;
		std::vector<int> m_invBoneMap;
		int m_numVertices;
		int m_numDynamicBones;
		CudaBuffer<cuBone, Bone> m_bones;
	};

	CudaBody::CudaBody(SkinnedMeshBody* body)
		: m_imp(new Imp(body))
	{}

	void CudaBody::synchronize()
	{
		m_imp->synchronize();
	}

	int CudaBody::deviceId()
	{
		return m_imp->deviceId();
	}

	class CudaColliderTree
	{
		using NodePair = std::pair<int, int>;

		ColliderTree* m_tree;

	public:

		CudaColliderTree(ColliderTree* tree, CudaStream& stream)
			: m_tree(tree),
			m_numNodes(nodeCount(*tree)),
			m_nodeData(m_numNodes),
			m_nodeAabbs(m_numNodes)
		{
			unsigned int biggestNode = 0;
			buildNodeData(*tree, m_nodeData.get(), biggestNode);
			m_nodeData.toDevice(stream);
			
			_DMESSAGE("Tree with %d nodes, largest %d, total %d colliders.",
				m_numNodes,
				biggestNode,
				m_nodeData[m_numNodes-1].first + m_nodeData[m_numNodes-1].second);
		}

		void update()
		{
			updateBoundingBoxes(*m_tree, m_nodeAabbs);
		}

		int m_numNodes;
		CudaBuffer<NodePair> m_nodeData;
		CudaBuffer<cuAabb, Aabb> m_nodeAabbs;

	private:

		static int nodeCount(ColliderTree& tree)
		{
			int count = tree.numCollider ? 1 : 0;
			for (auto& child : tree.children)
			{
				count += nodeCount(child);
			}
			return count;
		}

		NodePair* buildNodeData(ColliderTree& tree, NodePair* nodeData, unsigned int& biggestNode)
		{
			if (tree.numCollider)
			{
				*nodeData++ = { tree.aabb - m_tree->aabb, tree.numCollider };
				biggestNode = std::max(biggestNode, tree.numCollider);
			}
			for (auto& child : tree.children)
			{
				nodeData = buildNodeData(child, nodeData, biggestNode);
			}
			return nodeData;
		}

		Aabb* updateBoundingBoxes(ColliderTree& tree, Aabb* boundingBoxes)
		{
			if (tree.numCollider)
			{
				tree.aabbMe = *boundingBoxes++;
			}
			else
			{
				tree.aabbMe.invalidate();
			}
			tree.aabbAll = tree.aabbMe;
			for (auto& child : tree.children)
			{
				boundingBoxes = updateBoundingBoxes(child, boundingBoxes);
				tree.aabbAll.merge(child.aabbAll);
			}
			return boundingBoxes;
		}
	};

	class CudaPerTriangleShape::Imp
	{
	public:

		Imp(PerTriangleShape* shape)
			: m_device(cuGetDevice()),
			m_numColliders(shape->m_colliders.size()),
			m_penetrationType(abs(shape->m_shapeProp.penetration) > FLT_EPSILON ? eInternal : eNone),
			m_body(shape->m_owner->m_cudaObject->m_imp),
			m_input(shape->m_colliders.size()),
			m_output(shape->m_colliders.size()),
			m_tree(&shape->m_tree, m_body->m_stream),
			m_margin(shape->m_shapeProp.margin),
			m_penetration(shape->m_shapeProp.penetration)
		{
			for (int i = 0; i < m_numColliders; ++i)
			{
				if (m_penetration < 0)
				{
					m_input.get()[i] = {
						{	static_cast<int>(shape->m_colliders[i].vertices[1]),
							static_cast<int>(shape->m_colliders[i].vertices[0]),
							static_cast<int>(shape->m_colliders[i].vertices[2]) },
						shape->m_colliders[i].flexible };
				}
				else
				{
					m_input.get()[i] = {
						{ static_cast<int>(shape->m_colliders[i].vertices[0]),
							static_cast<int>(shape->m_colliders[i].vertices[1]),
							static_cast<int>(shape->m_colliders[i].vertices[2]) },
						shape->m_colliders[i].flexible };
				}
			}
			m_input.toDevice(m_body->m_stream);
			m_tree.m_nodeData.toDevice(m_body->m_stream);
		}

		void updateTree()
		{
			m_tree.update();
		}

		int deviceId()
		{
			return m_device;
		}

		operator cuColliderData<CudaPerTriangleShape>()
		{
			return { m_input.getD(), m_output.getD(), m_numColliders, { m_margin, -abs(m_penetration) } };
		}

		int m_device;
		CudaBuffer<TriangleInputArray> m_input;
		CudaDeviceBuffer<BoundingBoxArray> m_output;
		std::shared_ptr<CudaBody::Imp> m_body;
		const cuPenetrationType m_penetrationType;
		int m_numColliders;
		CudaColliderTree m_tree;
		float m_margin;
		float m_penetration;
	};

	CudaPerTriangleShape::CudaPerTriangleShape(PerTriangleShape* shape)
		: m_imp(new Imp(shape))
	{}

	void CudaPerTriangleShape::updateTree()
	{
		m_imp->updateTree();
	}

	int CudaPerTriangleShape::deviceId()
	{
		return m_imp->deviceId();
	}

	class CudaPerVertexShape::Imp
	{
	public:

		Imp(PerVertexShape* shape)
			: m_device(cuGetDevice()),
			m_numColliders(shape->m_colliders.size()),
			m_body(shape->m_owner->m_cudaObject->m_imp),
			m_input(shape->m_colliders.size()),
			m_output(shape->m_colliders.size()),
			m_tree(&shape->m_tree, m_body->m_stream),
			m_margin(shape->m_shapeProp.margin)
		{
			for (int i = 0; i < m_numColliders; ++i)
			{
				m_input.get()[i] = {
					static_cast<int>(shape->m_colliders[i].vertex),
					shape->m_colliders[i].flexible };
			}
			m_input.toDevice(m_body->m_stream);
			m_tree.m_nodeData.toDevice(m_body->m_stream);
		}

		void updateTree()
		{
			m_tree.update();
		}

		int deviceId()
		{
			return m_device;
		}

		operator cuColliderData<CudaPerVertexShape>()
		{
			return { m_input.getD(), m_output.getD(), m_numColliders, { m_margin } };
		}

		int m_device;
		CudaBuffer<VertexInputArray> m_input;
		CudaDeviceBuffer<BoundingBoxArray> m_output;
		std::shared_ptr<CudaBody::Imp> m_body;
		int m_numColliders;
		CudaColliderTree m_tree;
		float m_margin;
	};

	CudaPerVertexShape::CudaPerVertexShape(PerVertexShape* shape)
		: m_imp(new Imp(shape))
	{}

	void CudaPerVertexShape::updateTree()
	{
		m_imp->updateTree();
	}

	int CudaPerVertexShape::deviceId()
	{
		return m_imp->deviceId();
	}

	class CudaMergeBuffer::Imp
	{
	public:

		Imp(SkinnedMeshBody* body0, SkinnedMeshBody* body1)
			: m_x(body0->m_skinnedBones.size()),
			m_y(body1->m_skinnedBones.size()),
			m_dynx(body0->m_cudaObject->m_imp->m_numDynamicBones),
			m_stream(),
			m_buffer(m_dynx * m_y + m_x * body1->m_cudaObject->m_imp->m_numDynamicBones)
		{
			m_buffer.zero(m_stream);
		}

		void launchTransfer()
		{
			m_buffer.toHost(m_stream);
		}

		void addManifold(cuCollisionMerge* c, SkinnedMeshBone* rb0, SkinnedMeshBone* rb1, CollisionDispatcher* dispatcher)
		{
			if (c->weight < FLT_EPSILON) return;

			if (rb0 == rb1) return;

			float invWeight = 1.0f / c->weight;

			auto maniford = dispatcher->getNewManifold(&rb0->m_rig, &rb1->m_rig);
			auto worldA = btVector4(c->posA.val) * invWeight;
			auto worldB = btVector4(c->posB.val) * invWeight;
			auto localA = rb0->m_rig.getWorldTransform().invXform(worldA);
			auto localB = rb1->m_rig.getWorldTransform().invXform(worldB);
			auto normal = btVector4(c->normal.val) * invWeight;
			if (normal.fuzzyZero()) return;
			auto depth = -normal.length();
			normal = -normal.normalized();

			if (depth >= -FLT_EPSILON) return;

			btManifoldPoint newPt(localA, localB, normal, depth);
			newPt.m_positionWorldOnA = worldA;
			newPt.m_positionWorldOnB = worldB;
			newPt.m_combinedFriction = rb0->m_rig.getFriction() * rb1->m_rig.getFriction();
			newPt.m_combinedRestitution = rb0->m_rig.getRestitution() * rb1->m_rig.getRestitution();
			newPt.m_combinedRollingFriction = rb0->m_rig.getRollingFriction() * rb1->m_rig.getRollingFriction();
			maniford->addManifoldPoint(newPt);
		}

		void apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher)
		{
			// Checking can-collide-with and no-collide-with involves a list search, so just do it once for each bone
			std::vector<bool> canCollide0(body0->m_skinnedBones.size());
			for (int i = 0; i < body0->m_skinnedBones.size(); ++i)
			{
				canCollide0[i] = body1->canCollideWith(body0->m_skinnedBones[i].ptr);
			}
			std::vector<bool> canCollide1(body1->m_skinnedBones.size());
			for (int i = 0; i < body1->m_skinnedBones.size(); ++i)
			{
				canCollide1[i] = body0->canCollideWith(body1->m_skinnedBones[i].ptr);
			}

			cuSynchronize(m_stream).check(__FUNCTION__);

			int* map0 = body0->m_cudaObject->m_imp->m_boneMap.get();
			int* map1 = body1->m_cudaObject->m_imp->m_boneMap.get();

			// First check each dynamic bone of body 0 against every bone of body 1
			for (int dyn = 0; dyn < body0->m_cudaObject->m_imp->m_invBoneMap.size(); ++dyn)
			{
				int i = body0->m_cudaObject->m_imp->m_invBoneMap[dyn];
				if (!canCollide0[i])
				{
					continue;
				}

				for (int j = 0; j < body1->m_skinnedBones.size(); ++j)
				{
					if (!canCollide1[j])
					{
						continue;
					}

					cuCollisionMerge* c = m_buffer.get() + dyn * m_y + j;
					auto rb0 = body0->m_skinnedBones[i].ptr;
					auto rb1 = body1->m_skinnedBones[j].ptr;
					addManifold(c, rb0, rb1, dispatcher);

				}
			}

			// Then check each dynamic bone of body 1 against each kinematic bone of body 0
			for (int dyn = 0; dyn < body1->m_cudaObject->m_imp->m_invBoneMap.size(); ++dyn)
			{
				int j = body1->m_cudaObject->m_imp->m_invBoneMap[dyn];
				if (!canCollide1[j])
				{
					continue;
				}

				for (int i = 0; i < body0->m_skinnedBones.size(); ++i)
				{
					if (!body0->m_skinnedBones[i].isKinematic || !canCollide0[i])
					{
						continue;
					}

					cuCollisionMerge* c = m_buffer.get() + m_dynx * m_y + m_x * dyn + i;
					auto rb0 = body0->m_skinnedBones[i].ptr;
					auto rb1 = body1->m_skinnedBones[j].ptr;
					addManifold(c, rb0, rb1, dispatcher);
				}
			}
		}

		operator cuMergeBuffer()
		{
			return { m_buffer.getD(), m_x, m_y, m_dynx };
		}

		CudaStream m_stream;

	private:
		int m_x;
		int m_y;
		int m_dynx;
		CudaPooledBuffer<cuCollisionMerge> m_buffer;
	};

	CudaMergeBuffer::CudaMergeBuffer(SkinnedMeshBody* body0, SkinnedMeshBody* body1)
		: m_imp(new Imp(body0, body1))
	{}

	void CudaMergeBuffer::launchTransfer()
	{
		m_imp->launchTransfer();
	}

	void CudaMergeBuffer::apply(SkinnedMeshBody* body0, SkinnedMeshBody* body1, CollisionDispatcher* dispatcher)
	{
		m_imp->apply(body0, body1, dispatcher);
	}

	template <typename T>
	class CudaCollisionPair<T>::Imp
	{
	public:

		Imp(
			CudaPerVertexShape* shapeA,
			T* shapeB,
			int numCollisionPairs)
			: m_shapeA(shapeA),
			m_shapeB(shapeB),
			m_numCollisionPairs(numCollisionPairs),
			m_nextPair(0),
			m_setupBuffer(numCollisionPairs)
		{}

		void addPair(
			int offsetA,
			int offsetB,
			int sizeA,
			int sizeB,
			const Aabb& aabbA,
			const Aabb& aabbB)
		{
			static_assert(sizeof(cuCollider) == sizeof(Collider));

			m_setupBuffer[m_nextPair++] = {
				sizeA,
				sizeB,
				offsetA,
				offsetB,
				*reinterpret_cast<const cuAabb*>(&aabbA),
				*reinterpret_cast<const cuAabb*>(&aabbB)
			};
		}

		void launch(CudaMergeBuffer* merge, bool swap)
		{
			if (m_nextPair > 0)
			{
				collisionFunc()(
					merge->m_imp->m_stream,
					m_nextPair,
					swap,
					m_setupBuffer.getZ(),
					*m_shapeA->m_imp,
					*m_shapeB->m_imp,
					*m_shapeA->m_imp->m_body,
					*m_shapeB->m_imp->m_body,
					*merge->m_imp).check(__FUNCTION__);
			}
		}

		int numPairs()
		{
			return m_nextPair;
		}

	private:

		CudaPerVertexShape* m_shapeA;
		T* m_shapeB;
		int m_numCollisionPairs;
		int m_nextPair;

		CudaPooledBuffer<cuCollisionSetup> m_setupBuffer;

		template<typename T>
		struct InputType;
		template<>
		struct InputType<CudaPerVertexShape> { using type = VertexInputArray; };
		template<>
		struct InputType<CudaPerTriangleShape> { using type = TriangleInputArray; };

		auto collisionFunc() -> decltype(cuRunCollision<eNone, T>)*;
	};

	template<>
	auto CudaCollisionPair<CudaPerVertexShape>::Imp::collisionFunc()
		-> decltype(cuRunCollision<eNone, CudaPerVertexShape>)*
	{
		return cuRunCollision<eNone, CudaPerVertexShape>;
	}

	template<>
	auto CudaCollisionPair<CudaPerTriangleShape>::Imp::collisionFunc()
		-> decltype(cuRunCollision<eNone, CudaPerTriangleShape>)*
	{
		switch (m_shapeB->m_imp->m_penetrationType)
		{
		case eNone:
			return cuRunCollision<eNone, CudaPerTriangleShape>;
		case eInternal:
		default:
			return cuRunCollision<eInternal, CudaPerTriangleShape>;
		}
	}

	template <typename T>
	CudaCollisionPair<T>::CudaCollisionPair(
		CudaPerVertexShape* shapeA,
		T* shapeB, 
		int numCollisionPairs)
		: m_imp(new Imp(shapeA, shapeB, numCollisionPairs))
	{}

	template <typename T>
	void CudaCollisionPair<T>::addPair(
		int offsetA,
		int offsetB,
		int sizeA,
		int sizeB,
		const Aabb& aabbA,
		const Aabb& aabbB)
	{
		m_imp->addPair(offsetA, offsetB, sizeA, sizeB, aabbA, aabbB);
	}

	template <typename T>
	void CudaCollisionPair<T>::launch(CudaMergeBuffer* merge, bool swap)
	{
		m_imp->launch(merge, swap);
	}

	template <typename T>
	int CudaCollisionPair<T>::numPairs()
	{
		return m_imp->numPairs();
	}

	bool CudaInterface::enableCuda = false;
	int CudaInterface::currentDevice = 0;

	CudaInterface* CudaInterface::instance()
	{
		static CudaInterface s_instance;
		return &s_instance;
	}

	bool CudaInterface::hasCuda()
	{
		return enableCuda && m_enabled;
	}

	void CudaInterface::synchronize()
	{
		cuSynchronize().check(__FUNCTION__);
	}

	void CudaInterface::clearBufferPool()
	{
		CudaBufferPool::instance()->clear();
	}

	int CudaInterface::deviceCount()
	{
		return cuDeviceCount();
	}

	void CudaInterface::setCurrentDevice()
	{
		cuSetDevice(currentDevice);
	}

	void CudaInterface::launchInternalUpdate(
		std::shared_ptr<CudaBody> body,
		std::shared_ptr<CudaPerVertexShape> vertexShape,
		std::shared_ptr<CudaPerTriangleShape> triangleShape)
	{
		static const cuColliderData<CudaPerVertexShape> s_emptyVertexData = {
			VertexInputArray(nullptr, 0),
			BoundingBoxArray(nullptr, 0),
			0,
			{ 0 } };
		static const cuColliderData<CudaPerTriangleShape> s_emptyTriangleData = {
			TriangleInputArray(nullptr, 0),
			BoundingBoxArray(nullptr, 0),
			0,
			{ 0, 0 } };

		body->m_imp->m_bones.toDevice(body->m_imp->m_stream);

		cuInternalUpdate(
			body->m_imp->m_stream,
			*body->m_imp,
			body->m_imp->m_bones.getD(),
			vertexShape ? static_cast<cuColliderData<CudaPerVertexShape>>(*vertexShape->m_imp) : s_emptyVertexData,
			vertexShape ? vertexShape->m_imp->m_tree.m_numNodes : 0,
			vertexShape ? vertexShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
			vertexShape ? vertexShape->m_imp->m_tree.m_nodeAabbs.getZ() : nullptr,
			triangleShape ? static_cast<cuColliderData<CudaPerTriangleShape>>(*triangleShape->m_imp) : s_emptyTriangleData,
			triangleShape ? triangleShape->m_imp->m_tree.m_numNodes : 0,
			triangleShape ? triangleShape->m_imp->m_tree.m_nodeData.getD() : nullptr,
			triangleShape ? triangleShape->m_imp->m_tree.m_nodeAabbs.getZ() : nullptr).check(__FUNCTION__);
	}

	CudaInterface::CudaInterface()
		: m_enabled(cuDeviceCount() > 0)
	{
		if (m_enabled)
		{
			cuInitialize();
		}
	}

	template class CudaCollisionPair<CudaPerVertexShape>;
	template class CudaCollisionPair<CudaPerTriangleShape>;
}
#endif