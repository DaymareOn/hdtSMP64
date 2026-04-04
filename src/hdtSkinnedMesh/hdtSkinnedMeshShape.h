#pragma once

#include "hdtCollider.h"
#include "hdtCollisionAlgorithm.h"
#include "hdtSkinnedMeshBody.h"

namespace hdt
{
	class PerVertexShape;
	class PerTriangleShape;

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
		virtual void markUsedVertices(bool* flags) = 0;
		virtual void remapVertices(UINT* map) = 0;

		virtual float getColliderBoneWeight(const Collider* c, int boneIdx) = 0;
		virtual int getColliderBoneIndex(const Collider* c, int boneIdx) = 0;
		virtual btVector3 baryCoord(const Collider* c, const btVector3& p) = 0;
		virtual float baryWeight(const btVector3& w, int boneIdx) = 0;
		virtual int getBonePerCollider() = 0;

		SkinnedMeshBody* m_owner;
		vectorA16<Aabb> m_aabb;
		vectorA16<Collider> m_colliders;
		ColliderTree m_tree;
	};

	class PerVertexShape : public SkinnedMeshShape
	{
	public:
		PerVertexShape(SkinnedMeshBody* body);
		virtual ~PerVertexShape();

		PerVertexShape* asPerVertexShape() override { return this; }
		void internalUpdate() override;

		inline int getBonePerCollider() override final { return 4; }
		inline float getColliderBoneWeight(const Collider* c, int boneIdx) override final { return m_owner->m_vertices[c->vertex].m_weight[boneIdx]; }
		inline int getColliderBoneIndex(const Collider* c, int boneIdx) override final { return m_owner->m_vertices[c->vertex].getBoneIdx(boneIdx); }
		inline btVector3 baryCoord([[maybe_unused]] const Collider* c, [[maybe_unused]] const btVector3& p) override final { return btVector3(1, 1, 1); }
		inline float baryWeight([[maybe_unused]] const btVector3& w, [[maybe_unused]] int boneIdx) override final { return 1; }

		void finishBuild() override;
		void markUsedVertices(bool* flags) override;
		void remapVertices(UINT* map) override;
		void autoGen();

		struct ShapeProp
		{
			float margin = 1.0f;
		} m_shapeProp;
	};

	class PerTriangleShape : public SkinnedMeshShape
	{
	public:
		PerTriangleShape(SkinnedMeshBody* body);
		virtual ~PerTriangleShape();

		PerVertexShape* asPerVertexShape() override { return m_verticesCollision.get(); }
		PerTriangleShape* asPerTriangleShape() override { return this; }
		void internalUpdate() override;

		inline int getBonePerCollider() override final { return 12; }
		inline float getColliderBoneWeight(const Collider* c, int boneIdx) override final { return m_owner->m_vertices[c->vertices[boneIdx / 4]].m_weight[boneIdx % 4]; }
		inline int getColliderBoneIndex(const Collider* c, int boneIdx) override final { return m_owner->m_vertices[c->vertices[boneIdx / 4]].getBoneIdx(boneIdx % 4); }
		inline btVector3 baryCoord(const Collider* c, const btVector3& p) override final
		{
			return BaryCoord(
				m_owner->m_vpos[c->vertices[0]].pos(),
				m_owner->m_vpos[c->vertices[1]].pos(),
				m_owner->m_vpos[c->vertices[2]].pos(),
				p);
		}
		inline float baryWeight(const btVector3& w, int boneIdx) override final { return w[boneIdx / 4]; }

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
	};
}
