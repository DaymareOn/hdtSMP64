#include "hdtSkinnedMeshShape.h"

#include <ppl.h>

namespace hdt
{
	SkinnedMeshShape::SkinnedMeshShape(SkinnedMeshBody* body)
	{
		m_owner = body;
		m_owner->m_shape = hdt::make_smart(this);
	}

	SkinnedMeshShape::~SkinnedMeshShape()
	{
		//m_aabbGridLink.discard_data();
		//m_aabbGridBuffer.discard_data();
	}

	void SkinnedMeshShape::clipColliders()
	{
		m_tree.clipCollider([&, this](const Collider& n) -> bool {
			bool flg = false;
			for (int i = 0; i < getBonePerCollider() && !flg; ++i) {
				float weight = getColliderBoneWeight(&n, i);
				if (weight > FLT_EPSILON && weight > m_owner->m_skinnedBones[getColliderBoneIndex(&n, i)].weightThreshold)
					flg = true;
			}
			return !flg;
		});
	}

	PerVertexShape::PerVertexShape(SkinnedMeshBody* body) :
		SkinnedMeshShape(body)
	{
	}

	PerVertexShape::~PerVertexShape()
	{
	}

	void PerVertexShape::finishBuild()
	{
		m_tree.optimize();
		m_tree.updateKinematic([this](const Collider* n) {
			return m_owner->flexible(m_owner->m_vertices[n->vertex]);
		});

		m_owner->setCollisionFlags(m_tree.isKinematic ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);

		m_tree.exportColliders(m_colliders);
		m_aabb.resize(m_colliders.size());
		m_tree.remapColliders(m_colliders.data(), m_aabb.data());
	}

	void PerVertexShape::internalUpdate()
	{
		auto& vertices = m_owner->m_vpos;

		size_t size = m_colliders.size();
		for (size_t i = 0; i < size; ++i) {
			auto c = &m_colliders[i];
			auto p0 = vertices[c->vertex].m_data;
			auto margin = _mm_set_ps1(p0.m128_f32[3] * m_shapeProp.margin);
			m_aabb[i].m_min = p0 - margin;
			m_aabb[i].m_max = p0 + margin;
		}

		m_tree.updateAabb();
	}

	void PerVertexShape::autoGen()
	{
		m_tree.children.clear();
		std::vector<U32> keys;
		for (U32 i = 0; i < m_owner->m_vertices.size(); ++i) {
			keys.clear();
			for (int j = 0; j < 4; ++j) {
				if (m_owner->m_vertices[i].m_weight[j] > FLT_EPSILON)
					keys.push_back(m_owner->m_vertices[i].getBoneIdx(j));
			}
			m_tree.insertCollider(keys, Collider(i));
		}
	}

	void PerVertexShape::markUsedVertices(bool* flags)
	{
		for (auto& i : m_colliders)
			flags[i.vertex] = true;
	}

	void PerVertexShape::remapVertices(UINT* map)
	{
		for (auto& i : m_colliders)
			i.vertex = map[i.vertex];
	}

	PerTriangleShape::PerTriangleShape(SkinnedMeshBody* body) :
		SkinnedMeshShape(body)
	{
	}

	PerTriangleShape::~PerTriangleShape()
	{
	}

	void PerTriangleShape::internalUpdate()
	{
		auto& vertices = m_owner->m_vpos;

		size_t size = m_colliders.size();
		for (size_t i = 0; i < size; ++i) {
			auto c = &m_colliders[i];
			auto p0 = vertices[c->vertices[0]].m_data;
			auto p1 = vertices[c->vertices[1]].m_data;
			auto p2 = vertices[c->vertices[2]].m_data;
			auto margin4 = p0 + p1 + p2;

			auto aabbMin = _mm_min_ps(_mm_min_ps(p0, p1), p2);
			auto aabbMax = _mm_max_ps(_mm_max_ps(p0, p1), p2);
			auto prenetration = _mm_set_ss(m_shapeProp.penetration);
			prenetration = _mm_andnot_ps(_mm_set_ss(-0.0f), prenetration);  // abs
			margin4 = _mm_max_ss(_mm_set_ss(margin4.m128_f32[3] * m_shapeProp.margin / 3), prenetration);
			margin4 = _mm_shuffle_ps(margin4, margin4, 0);
			aabbMin = aabbMin - margin4;
			aabbMax = aabbMax + margin4;

			m_aabb[i].m_min = aabbMin;
			m_aabb[i].m_max = aabbMax;
		}

		m_tree.updateAabb();
	}

	void PerTriangleShape::finishBuild()
	{
		m_tree.optimize();
		m_tree.updateKinematic([=](const Collider* c) {
			float k = m_owner->flexible(m_owner->m_vertices[c->vertices[0]]);
			k += m_owner->flexible(m_owner->m_vertices[c->vertices[1]]);
			k += m_owner->flexible(m_owner->m_vertices[c->vertices[2]]);
			return k / 3;
		});

		m_owner->setCollisionFlags(m_tree.isKinematic ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);

		m_tree.exportColliders(m_colliders);
		m_aabb.resize(m_colliders.size());
		m_tree.remapColliders(m_colliders.data(), m_aabb.data());

		RE::BSTSmartPointer<PerTriangleShape> holder = hdt::make_smart(this);
		m_verticesCollision = RE::make_smart<PerVertexShape>(m_owner);
		m_verticesCollision->m_shapeProp.margin = m_shapeProp.margin;
		m_owner->m_shape = hdt::make_smart(this);

		m_verticesCollision->autoGen();
		m_verticesCollision->clipColliders();
		m_verticesCollision->finishBuild();
	}

	void PerTriangleShape::markUsedVertices(bool* flags)
	{
		for (auto& i : m_colliders) {
			flags[i.vertices[0]] = true;
			flags[i.vertices[1]] = true;
			flags[i.vertices[2]] = true;
		}

		m_verticesCollision->markUsedVertices(flags);
	}

	void PerTriangleShape::remapVertices(UINT* map)
	{
		for (auto& i : m_colliders) {
			i.vertices[0] = map[i.vertices[0]];
			i.vertices[1] = map[i.vertices[1]];
			i.vertices[2] = map[i.vertices[2]];
		}

		m_verticesCollision->remapVertices(map);
	}

	void PerTriangleShape::addTriangle(int a, int b, int c)
	{
		assert(a < m_owner->m_vertices.size());
		assert(b < m_owner->m_vertices.size());
		assert(c < m_owner->m_vertices.size());
		Collider collider(a, b, c);
		std::vector<U32> keys;
		std::vector<float> w;
		for (int i = 0; i < 12; ++i) {
			auto weight = getColliderBoneWeight(&collider, i);
			if (weight < FLT_EPSILON)
				continue;
			auto bone = getColliderBoneIndex(&collider, i);
			auto iter = std::find(keys.begin(), keys.end(), (U32)bone);
			if (iter != keys.end())
				w[iter - keys.begin()] += weight;
			else {
				keys.push_back(bone);
				w.push_back(weight);
			}
		}

		for (int i = 0; i < keys.size(); ++i)
			for (int j = 1; j < keys.size(); ++j) {
				if (w[j - 1] < w[j] || (w[j - 1] == w[j] && keys[j] < keys[j - 1])) {
					std::swap(keys[j], keys[j - 1]);
					std::swap(w[j], w[j - 1]);
				}
			}

		m_tree.insertCollider(keys, collider);
	}
}
