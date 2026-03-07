#pragma once

#include "FrameworkUtils.h"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <cassert>
#include <cfloat>
#include <intrin.h>

#undef min
#undef max

#define HDT_LOCK_GUARD(name, lock) std::lock_guard<decltype(lock)> name(lock)

namespace hdt
{
	typedef int8_t I8;
	typedef int16_t I16;
	typedef int32_t I32;
	typedef int64_t I64;

	typedef uint8_t U8;
	typedef uint16_t U16;
	typedef uint32_t U32;
	typedef uint64_t U64;

	template <int imm>
	__m128 pshufd(__m128 m)
	{
		return _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(m), imm));
	}

	inline __m128 setAll(float f) { return pshufd<0>(_mm_load_ss(&f)); }
	inline __m128 setAll0(__m128 m) { return pshufd<0>(m); }
	inline __m128 setAll1(__m128 m) { return pshufd<0x55>(m); }
	inline __m128 setAll2(__m128 m) { return pshufd<0xAA>(m); }
	inline __m128 setAll3(__m128 m) { return pshufd<0xFF>(m); }

	inline __m128& operator+=(__m128& l, __m128 r)
	{
		l = _mm_add_ps(l, r);
		return l;
	}

	inline __m128& operator-=(__m128& l, __m128 r)
	{
		l = _mm_sub_ps(l, r);
		return l;
	}

	inline __m128& operator*=(__m128& l, __m128 r)
	{
		l = _mm_mul_ps(l, r);
		return l;
	}

	inline __m128& operator+=(__m128& l, float r)
	{
		l = _mm_add_ps(l, setAll(r));
		return l;
	}

	inline __m128& operator-=(__m128& l, float r)
	{
		l = _mm_sub_ps(l, setAll(r));
		return l;
	}

	inline __m128& operator*=(__m128& l, float r)
	{
		l = _mm_mul_ps(l, setAll(r));
		return l;
	}

	inline __m128 cross(__m128 a, __m128 b)
	{
		__m128 T, V;

		T = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(a);
		V = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(b);

		V = _mm_mul_ps(V, a);
		T = _mm_mul_ps(T, b);
		V = _mm_sub_ps(V, T);

		V = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(V);
		return V;
	}

	inline float rsqrt(float number)
	{
		__m128 v = _mm_set_ss(number);
		__m128 est = _mm_rsqrt_ss(v);
		__m128 half = _mm_set_ss(0.5f);
		__m128 three = _mm_set_ss(3.0f);
		__m128 muls = _mm_mul_ss(_mm_mul_ss(v, est), est);
		est = _mm_mul_ss(_mm_mul_ss(half, est), _mm_sub_ss(three, muls));
		return _mm_cvtss_f32(est);
	}

	template <class T>
	T abs(T rhs)
	{
		return rhs < 0 ? -rhs : rhs;
	}

	template <>
	inline float abs(float rhs)
	{
		return _mm_cvtss_f32(_mm_andnot_ps(_mm_set_ss(-0.f), _mm_set_ss(rhs)));
	}

	template <class T>
	T min(const T& a, const T& b) restrict(cpu, amp)
	{
		return a < b ? a : b;
	}

	template <class T>
	T max(const T& a, const T& b) restrict(cpu, amp)
	{
		return a < b ? b : a;
	}

	inline int aligned(int x, int a) { return (x + a - 1) & -a; }

	template <int a>
	int aligned(int x)
	{
		return (x + a - 1) & -a;
	}

	inline U32 aligned2Pow(U32 lim)
	{
		if (lim == 0)
			return 0;
		unsigned long idx;
		_BitScanReverse(&idx, lim);
		U32 size = 1u << idx;
		return (size < lim) ? size << 1 : size;
	}

	ATTRIBUTE_ALIGNED16(class)
	btQsTransform
	{
		btQuaternion m_basis;
		btVector4 m_originScale;

	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		btQsTransform() :
			m_basis(btQuaternion::getIdentity()), m_originScale(0, 0, 0, 1)
		{
		}

		btQsTransform(const btQuaternion& r, const btVector3& t, float s = 1.0f) :
			m_basis(r), m_originScale(t.get128())
		{
			setScale(s);
		}

		btQsTransform(const btTransform& t, float s = 1.0f)
		{
			m_basis = t.getRotation();
			m_originScale = t.getOrigin();
			setScale(s);
		}

		btQsTransform(const btQsTransform& rhs) :
			m_basis(rhs.m_basis), m_originScale(rhs.m_originScale.get128())
		{
			assert(getScale() > 0);
		}

		btQuaternion getBasis() const { return m_basis; }
		btQuaternion& getBasis() { return m_basis; }
		void setBasis(const btQuaternion& q) { m_basis = q; }
		void setBasis(const btMatrix3x3& m) { m.getRotation(m_basis); }

		float getScale() const { return m_originScale[3]; }
		float& getScale() { return m_originScale[3]; }
		float getScaleReg() const { return _mm_cvtss_f32(setAll3(m_originScale.get128())); }

		void setScale(float s)
		{
			assert(s > 0);
			m_originScale[3] = s;
		}

		btVector3 getOrigin() const { return m_originScale; }

		void setOrigin(const btVector3& vec)
		{
			float s = getScale();
			m_originScale = vec.mVec128;
			setScale(s);
		}

		void setOrigin(float x, float y, float z)
		{
			m_originScale[0] = x;
			m_originScale[1] = y;
			m_originScale[2] = z;
		}

		btQsTransform& operator=(const btQsTransform& rhs)
		{
			m_basis = rhs.m_basis;
			m_originScale = rhs.m_originScale;
			assert(getScale() > 0);
			return *this;
		}

		btQsTransform operator*(const btQsTransform& rhs) const
		{
			return btQsTransform(m_basis * rhs.m_basis, *this * rhs.getOrigin(), getScale() * rhs.getScale());
		}

		btVector3 operator*(const btVector3& rhs) const
		{
			return getOrigin() + quatRotate(m_basis, rhs * getScale());
		}

		void operator*=(const btQsTransform& rhs)
		{
			float s = getScale();
			setOrigin(getOrigin() + quatRotate(m_basis, rhs.getOrigin() * s));
			m_basis *= rhs.m_basis;
			setScale(s * rhs.getScale());
		}

		btQsTransform inverse() const
		{
			auto r = m_basis.inverse();
			auto s = 1 / getScale();
			return btQsTransform(r, quatRotate(r, -getOrigin() * s), s);
		}

		btTransform asTransform() const
		{
			return btTransform(m_basis, m_originScale);
		}
	};

	ATTRIBUTE_ALIGNED16(class)
	btMatrix4x3
	{
	public:
		btMatrix4x3()
		{
		}

		btMatrix4x3(const btQsTransform& t)
		{
			((btMatrix3x3*)this)->setRotation(t.getBasis());
			__m128 scale = pshufd<0xFF>(t.getOrigin().get128());
			m_row[0] = _mm_mul_ps(m_row[0], scale);
			m_row[1] = _mm_mul_ps(m_row[1], scale);
			m_row[2] = _mm_mul_ps(m_row[2], scale);
			m_row[0].m128_f32[3] = t.getOrigin()[0];
			m_row[1].m128_f32[3] = t.getOrigin()[1];
			m_row[2].m128_f32[3] = t.getOrigin()[2];
		}

		btVector3 operator*(const btVector3& rhs) const
		{
#ifdef BT_ALLOW_SSE4
			auto v = _mm_blend_ps(rhs.get128(), _mm_set_ps1(1), 0x8);
			__m128 xmm0 = _mm_dp_ps(m_row[0], v, 0xF1);
			__m128 xmm1 = _mm_dp_ps(m_row[1], v, 0xF2);
			__m128 xmm2 = _mm_dp_ps(m_row[2], v, 0xF4);
			xmm0 = _mm_or_ps(xmm0, xmm1);
			xmm0 = _mm_or_ps(xmm0, xmm2);
#else
			auto v = rhs.get128();
			v.m128_f32[3] = 1;

			__m128 xmm0 = _mm_mul_ps(m_row[0], v);
			__m128 xmm1 = _mm_mul_ps(m_row[1], v);
			__m128 xmm2 = _mm_mul_ps(m_row[2], v);

			xmm0 = _mm_hadd_ps(xmm0, xmm1);
			xmm2 = _mm_hadd_ps(xmm2, xmm2);
			xmm0 = _mm_hadd_ps(xmm0, xmm2);
#endif
			return xmm0;
		}

		__m128 mulPack(const btVector3& rhs, float packW) const
		{
#ifdef BT_ALLOW_SSE4
			auto v = _mm_blend_ps(rhs.get128(), _mm_set_ps1(1), 0x8);
			__m128 xmm0 = _mm_dp_ps(m_row[0], v, 0xF1);       // x, 0, 0, 0
			__m128 xmm1 = _mm_dp_ps(m_row[1], v, 0xF2);       // 0, y, 0, 0
			xmm0 = _mm_or_ps(xmm0, xmm1);                     // x, y, 0, 0
			xmm1 = _mm_dp_ps(m_row[2], v, 0xF1);              // z, 0, 0, 0
			xmm1 = _mm_unpacklo_ps(xmm1, _mm_set_ss(packW));  // z, w, 0, 0
			xmm0 = _mm_movelh_ps(xmm0, xmm1);                 // x, y, z, w
#else
			auto v = rhs.get128();
			v.m128_f32[3] = 1;
			auto w = _mm_load_ss(&packW);

			__m128 xmm0 = _mm_mul_ps(m_row[0], v);
			__m128 xmm1 = _mm_mul_ps(m_row[1], v);
			__m128 xmm2 = _mm_mul_ps(m_row[2], v);

			xmm0 = _mm_hadd_ps(xmm0, xmm1);
			xmm2 = _mm_hadd_ps(xmm2, w);
			xmm0 = _mm_hadd_ps(xmm0, xmm2);
#endif
			return xmm0;
		}
		__m128 m_row[3];
	};

	ATTRIBUTE_ALIGNED16(class)
	btMatrix4x3T
	{
	public:
		btMatrix4x3T()
		{
		}

		btMatrix4x3T(const btQsTransform& t)
		{
			btMatrix3x3 rot;
			rot.setRotation(t.getBasis());
			rot = rot.transpose();
			__m128 scale = pshufd<0xFF>(t.getOrigin().get128());
			m_col[0] = rot[0].get128() * scale;
			m_col[1] = rot[1].get128() * scale;
			m_col[2] = rot[2].get128() * scale;
			m_col[3] = t.getOrigin().get128();
		}

		btVector3 operator*(const btVector3& rhs) const
		{
			return m_col[0] * rhs[0] + m_col[1] * rhs[1] + m_col[2] * rhs[2] + m_col[3];
		}

		btMatrix4x3T operator*(const btMatrix4x3T& r)
		{
			btMatrix4x3T ret;
			ret.m_col[0] = m_col[0] * r.m_col[0][0] + m_col[1] * r.m_col[0][1] + m_col[2] * r.m_col[0][2];
			ret.m_col[1] = m_col[0] * r.m_col[1][0] + m_col[1] * r.m_col[1][1] + m_col[2] * r.m_col[1][2];
			ret.m_col[2] = m_col[0] * r.m_col[2][0] + m_col[1] * r.m_col[2][1] + m_col[2] * r.m_col[2][2];
			ret.m_col[3] = *this * r.m_col[3];
			return ret;
		}

		btMatrix3x3 basis() const { return ((btMatrix3x3*)this)->transpose(); }
		btTransform toTransform() const { return btTransform(((btMatrix3x3*)this)->transpose(), m_col[3]); }
		btVector3 m_col[4];
	};

	template <>
	inline btVector3 abs(btVector3 rhs)
	{
		return _mm_andnot_ps(_mm_set_ps1(-0.f), rhs.get128());
	}

	template <class T>
	using vectorA16 = std::vector<T>;

	struct SpinLock
	{
	public:
		void lock()
		{
			for (int spin = 0; m_flag.test_and_set(std::memory_order_acquire); ++spin) {
				if (spin < 16)
					_mm_pause();
				else if (spin < 128) {
					for (int i = 0; i < spin; ++i)
						_mm_pause();
				} else
					SwitchToThread();
			}
		}

		void unlock() { m_flag.clear(std::memory_order_release); }

	protected:
		std::atomic_flag m_flag = ATOMIC_FLAG_INIT;
	};
}
