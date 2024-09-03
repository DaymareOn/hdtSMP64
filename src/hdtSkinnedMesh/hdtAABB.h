#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	struct Aabb
	{
		Aabb() { invalidate(); }

		Aabb(__m128 mmin, __m128 mmax) : m_min(mmin), m_max(mmax)
		{
		}

		__m128 m_min;
		__m128 m_max;

		void extendMargin(float margin)
		{
			auto margin3 = _mm_set_ps1(margin);
			m_min -= margin3;
			m_max += margin3;
		}

		Aabb extended(float margin) const
		{
			auto margin3 = _mm_set_ps1(margin);
			return Aabb(m_min - margin3, m_max + margin3);
		}

		bool collideWith(const btVector3& rhs) const
		{
			auto flag0 = _mm_cmplt_ps(m_min, rhs.get128());
			auto flag1 = _mm_cmplt_ps(rhs.get128(), m_max);
			auto flag = _mm_movemask_ps(_mm_and_ps(flag0, flag1));
			return (flag & 0x7) == 7;
		}

		bool collideWith(const Aabb& rhs) const
		{
			auto flag0 = _mm_cmplt_ps(rhs.m_max, m_min);
			auto flag1 = _mm_cmplt_ps(m_max, rhs.m_min);
			auto flag = _mm_movemask_ps(_mm_or_ps(flag0, flag1));
			return !(flag & 0x7);
			/*
			bool overlap = true;
			overlap = (m_min[0] >= rhs.m_max[0] || m_max[0] <= rhs.m_min[0]) ? false : overlap;
			overlap = (m_min[1] >= rhs.m_max[1] || m_max[1] <= rhs.m_min[1]) ? false : overlap;
			overlap = (m_min[2] >= rhs.m_max[2] || m_max[2] <= rhs.m_min[2]) ? false : overlap;
			return overlap;*/
		}

		bool collideWithSphere(const btVector3& p, float radius) const
		{
			return extended(radius).collideWith(p);
		}

		void invalidate()
		{
			m_min = setAll(FLT_MAX);
			m_max = setAll(-FLT_MAX);
		}

		void merge(const btVector3& p)
		{
			m_min = _mm_min_ps(m_min, p.get128());
			m_max = _mm_max_ps(m_max, p.get128());
		}

		void merge(const Aabb& rhs)
		{
			m_min = _mm_min_ps(m_min, rhs.m_min);
			m_max = _mm_max_ps(m_max, rhs.m_max);
		}

		void mergeAdd(const btVector3& p)
		{
			m_min = _mm_min_ps(m_min, _mm_add_ps(m_min, p.get128()));
			m_max = _mm_max_ps(m_max, _mm_add_ps(m_max, p.get128()));
		}

		void mergeSub(const btVector3& p)
		{
			m_min = _mm_min_ps(m_min, _mm_sub_ps(m_min, p.get128()));
			m_max = _mm_max_ps(m_max, _mm_sub_ps(m_max, p.get128()));
		}
	};

	struct BoundingSphere
	{
		BoundingSphere()
		{
		}

		BoundingSphere(const btVector3& center, float radius)
			: m_centerRadius(center)
		{
			m_centerRadius[3] = radius;
		}

		bool isCollide(const BoundingSphere& rhs) const
		{
			btVector3 ca = m_centerRadius;
			btVector3 cb = rhs.m_centerRadius;
			float ra = m_centerRadius.w();
			float rb = rhs.m_centerRadius.w();
			return (ca - cb).length2() < (ra + rb) * (ra + rb);
		}

		btVector3 center() const { return m_centerRadius; }
		float radius() const { return m_centerRadius.w(); }
		Aabb getAabb() const { return Aabb(center().get128(), center().get128()).extended(radius()); }

		btVector4 m_centerRadius;
	};
}
