#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

#include <ppl.h>

namespace hdt
{

	SkinnedMeshBody::SkinnedMeshBody()
	{
		m_collisionShape = &m_bulletShape;
	}

	SkinnedMeshBody::~SkinnedMeshBody()
	{
		//m_bonesGPU.discard_data();
		//m_verticesGPU.discard_data();
	}

	__forceinline __m128 calcVertexState(__m128 skinPos, const Bone& bone, __m128 w)
	{
		auto p = bone.m_vertexToWorld * skinPos;
		p = _mm_blend_ps(p.get128(), _mm_load_ps(bone.m_reserved), 0x8);
		return _mm_mul_ps(w, p.get128());
	}

	void SkinnedMeshBody::internalUpdate()
	{
		for (size_t i = 0; i < m_skinnedBones.size(); ++i) {
			auto& v = m_skinnedBones[i];
			auto boneT = v.ptr->m_currentTransform;
			m_bones[i].m_vertexToWorld = btMatrix4x3T(boneT) * v.vertexToBone;
			m_bones[i].m_maginMultipler = v.ptr->m_marginMultipler * boneT.getScale();
		}

		int size = static_cast<int>(m_vpos.size());

		for (int idx = 0; idx < size; ++idx) {
			auto& v = m_vertices[idx];
			auto p = v.m_skinPos.get128();
			auto w = _mm_load_ps(v.m_weight);
			auto flg = _mm_movemask_ps(_mm_cmplt_ps(_mm_set_ps1(FLT_EPSILON), w));
			auto posMargin = calcVertexState(p, m_bones[v.getBoneIdx(0)], setAll0(w));
			if (flg & 0b0010)
				posMargin += calcVertexState(p, m_bones[v.getBoneIdx(1)], setAll1(w));
			if (flg & 0b0100)
				posMargin += calcVertexState(p, m_bones[v.getBoneIdx(2)], setAll2(w));
			if (flg & 0b1000)
				posMargin += calcVertexState(p, m_bones[v.getBoneIdx(3)], setAll3(w));
			m_vpos[idx].set(posMargin);
		}

		// FIXME PROFILING Lots of times is spent here.
		m_shape->internalUpdate();

		m_bulletShape.m_aabb = m_shape->m_tree.aabbAll;
	}

	float SkinnedMeshBody::flexible(const Vertex& v)
	{
		float ret = 0.f;
		for (int i = 0; i < 4; ++i) {
			if (v.m_weight[i] < FLT_EPSILON)
				break;
			int boneIdx = v.getBoneIdx(i);
			if (!m_skinnedBones[boneIdx].isKinematic)
				ret += v.m_weight[i];
		}
		return ret;
	}

	int SkinnedMeshBody::addBone(SkinnedMeshBone* bone, const btQsTransform& verticesToBone,
		const BoundingSphere& boundingSphere)
	{
		m_skinnedBones.push_back(SkinnedBone());
		auto& v = m_skinnedBones.back();
		v.ptr = bone;
		v.vertexToBone = verticesToBone;
		v.localBoundingSphere = boundingSphere;
		v.isKinematic = bone->m_rig.isStaticOrKinematicObject();
		return static_cast<int>(m_skinnedBones.size() - 1);
	}

	void SkinnedMeshBody::finishBuild()
	{
		m_bones.resize(m_skinnedBones.size());
		m_shape->clipColliders();
		m_vpos.resize(m_vertices.size());

		m_isKinematic = true;
		for (auto& i : m_skinnedBones) {
			i.isKinematic = i.ptr->m_rig.isStaticOrKinematicObject();
			if (!i.isKinematic)
				m_isKinematic = false;
		}

		m_shape->finishBuild();

		bool* flags = new bool[m_vertices.size()];
		ZeroMemory(flags, m_vertices.size());
		m_shape->markUsedVertices(flags);

		UINT numUsed = 0;
		std::vector<UINT> map(m_vertices.size());
		for (int i = 0; i < m_vertices.size(); ++i) {
			if (flags[i]) {
				m_vertices[numUsed] = m_vertices[i];
				m_vpos[numUsed] = m_vpos[i];
				map[i] = numUsed++;
			}
		}
		delete[] flags;
		m_shape->remapVertices(map.data());
		m_vertices.resize(numUsed);
		m_vpos.resize(numUsed);

		m_useBoundingSphere = m_shape->m_colliders.size() > 10;
	}

	bool SkinnedMeshBody::canCollideWith(const SkinnedMeshBody* body) const
	{
		if (m_isKinematic && body->m_isKinematic)
			return false;

		if (m_canCollideWithTags.empty()) {
			for (auto& i : body->m_tags)
				if (m_noCollideWithTags.find(i) != m_noCollideWithTags.end())
					return false;
			return true;
		}
		for (auto& i : body->m_tags)
			if (m_canCollideWithTags.find(i) != m_canCollideWithTags.end())
				return true;
		return false;
	}

	void SkinnedMeshBody::updateBoundingSphereAabb()
	{
		m_bulletShape.m_aabb.invalidate();
		for (auto& i : m_skinnedBones) {
			auto sp = i.localBoundingSphere;
			auto tr = i.ptr->m_currentTransform;
			i.worldBoundingSphere = BoundingSphere(tr * sp.center(), tr.getScale() * sp.radius());
			m_bulletShape.m_aabb.merge(i.worldBoundingSphere.getAabb());
		}

		if (!m_useBoundingSphere)
			internalUpdate();
	}

	bool SkinnedMeshBody::isBoundingSphereCollided(SkinnedMeshBody* rhs)
	{
		if (canCollideWith(rhs) && rhs->canCollideWith(this)) {
			return true;
		}
		return false;
	}
}
