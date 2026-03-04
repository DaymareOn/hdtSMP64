#pragma once

#include "hdtCollisionAlgorithm.h"
#include "hdtCollider.h"
#include "hdtSkinnedMeshBody.h"

namespace hdt
{
	class PerVertexShape;
	class PerTriangleShape;
#ifdef CUDA
	class CudaPerVertexShape;
	class CudaPerTriangleShape;
#endif // CUDA

	class SkinnedMeshShape : 
		public RE::BSIntrusiveRefCounted
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		SkinnedMeshShape(SkinnedMeshBody* body);
		virtual ~SkinnedMeshShape();

		virtual PerVertexShape* asPerVertexShape() { return nullptr; }
		virtual PerTriangleShape* asPerTriangleShape() { return nullptr; }

		const Aabb& getAabb() const { return m_tree.aabbAll; }

		virtual void clipColliders();
		virtual void finishBuild() = 0;
		virtual void internalUpdate() = 0;
		virtual int getBonePerCollider() = 0;
		virtual void markUsedVertices(bool* flags) = 0;
		virtual void remapVertices(UINT* map) = 0;

		virtual float getColliderBoneWeight(const Collider* c, int boneIdx) = 0;
		virtual int getColliderBoneIndex(const Collider* c, int boneIdx) = 0;
#ifndef CUDA
		virtual btVector3 baryCoord(const Collider* c, const btVector3& p) = 0;
		virtual float baryWeight(const btVector3 & w, int boneIdx) = 0;
#endif // !CUDA

		SkinnedMeshBody* m_owner;
#ifdef CUDA
		std::shared_ptr<Aabb[]> m_aabb;
#else
		vectorA16<Aabb> m_aabb;
#endif // CUDA
		vectorA16<Collider> m_colliders;
		ColliderTree m_tree;
		float m_windEffect = 0.f; //effect from xml m_windEffect

#ifdef ENABLE_CL
		cl::Buffer		m_aabbCL;
		cl::Buffer		m_colliderCL;
		cl::Event		m_eDoneCL;
		virtual void internalUpdateCL() = 0;
#endif
	};

	class PerVertexShape : public SkinnedMeshShape
	{
	public:
#ifdef CUDA
		using CudaType = CudaPerVertexShape;
#endif // CUDA

		PerVertexShape(SkinnedMeshBody* body);
		virtual ~PerVertexShape();

		PerVertexShape* asPerVertexShape() override { return this; }

		void internalUpdate() override;
		int getBonePerCollider() override { return 4; }

		float getColliderBoneWeight(const Collider* c, int boneIdx) override
		{
			return m_owner->m_vertices[c->vertex].m_weight[boneIdx];
		}

		int getColliderBoneIndex(const Collider* c, int boneIdx) override
		{
			return m_owner->m_vertices[c->vertex].getBoneIdx(boneIdx);
		}

#ifndef CUDA
		btVector3 baryCoord([[maybe_unused]] const Collider* c, [[maybe_unused]] const btVector3& p) override { return btVector3(1, 1, 1); }
		float baryWeight([[maybe_unused]] const btVector3 & w, [[maybe_unused]] int boneIdx) override { return 1; }
#endif // !CUDA
		void finishBuild() override;
		void markUsedVertices(bool* flags) override;
		void remapVertices(UINT* map) override;

		void autoGen();

		struct ShapeProp
		{
			float margin = 1.0f;
		} m_shapeProp;

#ifdef CUDA
		std::shared_ptr<CudaPerVertexShape> m_cudaObject;
#endif // CUDA

#ifdef ENABLE_CL
		static hdtCLKernel		m_kernel;
		virtual void internalUpdateCL();
#endif
	};

	class PerTriangleShape : public SkinnedMeshShape
	{
	public:
#ifdef CUDA
		using CudaType = CudaPerTriangleShape;
#endif

		PerTriangleShape(SkinnedMeshBody* body);
		virtual ~PerTriangleShape();

		PerVertexShape* asPerVertexShape() override { return m_verticesCollision.get(); }
		PerTriangleShape* asPerTriangleShape() override { return this; }

		void internalUpdate() override;
		int getBonePerCollider() override { return 12; }

		float getColliderBoneWeight(const Collider* c, int boneIdx) override
		{
			return m_owner->m_vertices[c->vertices[boneIdx / 4]].m_weight[boneIdx % 4];
		}

		int getColliderBoneIndex(const Collider* c, int boneIdx) override
		{
			return m_owner->m_vertices[c->vertices[boneIdx / 4]].getBoneIdx(boneIdx % 4);
		}

#ifndef CUDA
		btVector3 baryCoord(const Collider* c, const btVector3& p) override
		{
			return BaryCoord(
				m_owner->m_vpos[c->vertices[0]].pos(),
				m_owner->m_vpos[c->vertices[1]].pos(),
				m_owner->m_vpos[c->vertices[2]].pos(),
				p);
		}
		float baryWeight(const btVector3 & w, int boneIdx) override { return w[boneIdx / 4]; }
#endif
		void finishBuild() override;
		void markUsedVertices(bool* flags) override;
		void remapVertices(UINT* map) override;

		void addTriangle(int p0, int p1, int p2);

		struct ShapeProp
		{
			float margin = 1.0f;
			float penetration = 1.f;
		} m_shapeProp;

		RE::BSTSmartPointer<PerVertexShape> m_verticesCollision;

#ifdef CUDA
		std::shared_ptr<CudaPerTriangleShape> m_cudaObject;
#endif

#ifdef ENABLE_CL
		static hdtCLKernel		m_kernel;
		virtual void internalUpdateCL();
#endif
	};
}
