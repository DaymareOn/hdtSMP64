#include "hdtCollisionAlgorithm.h"

namespace hdt
{
	CheckTriangle::CheckTriangle(const btVector3& p0, const btVector3& p1, const btVector3& p2, float margin,
		float prenetration) :
		p0(p0), p1(p1), p2(p2), margin(margin), prenetration(prenetration)
	{
		edge0 = p1 - p0;
		edge1 = p2 - p0;
		edge2 = p2 - p1;

		normal = edge0.cross(edge1);
		__m128 len2 = _mm_dp_ps(normal.get128(), normal.get128(), 0x71);
		if (_mm_cvtss_f32(len2) < FLT_EPSILON * FLT_EPSILON) {
			valid = false;
		} else {
			valid = true;
			normal.set128(_mm_div_ps(normal.get128(), setAll0(_mm_sqrt_ss(len2))));

			d00 = edge0.dot(edge0);
			d01 = edge0.dot(edge1);
			d11 = edge1.dot(edge1);
			invDenom = 1.0f / (d00 * d11 - d01 * d01);

			if (prenetration > -FLT_EPSILON && prenetration < FLT_EPSILON)
				prenetration = 0;

			if (prenetration < 0) {
				//triangle facing the other way
				normal = -normal;
				prenetration = -prenetration;
			}
			this->prenetration = prenetration;
		}
	}

	btVector3 closestPointOnTriangle(const btVector3& p, const btVector3& a, const btVector3& b, const btVector3& c)
	{
		btVector3 ab = b - a;
		btVector3 ac = c - a;
		btVector3 ap = p - a;

		float d1 = ab.dot(ap);
		float d2 = ac.dot(ap);
		if (d1 <= 0.0f && d2 <= 0.0f)
			return a;  //1,0,0

		btVector3 bp = p - b;
		float d3 = ab.dot(bp);
		float d4 = ac.dot(bp);
		if (d3 >= 0.0f && d4 <= d3)
			return b;  //0,1,0

		float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
			float v = d1 / (d1 - d3);
			return a + ab * v;  //u,v,0
		}

		btVector3 cp = p - c;
		float d5 = ab.dot(cp);
		float d6 = ac.dot(cp);
		if (d6 >= 0.0f && d5 <= d6)
			return c;  //0,0,1

		float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
			float w = d2 / (d2 - d6);
			return a + ac * w;  //u,0,w
		}

		float va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
			float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return b + (c - b) * w;  //0,v,w
		}

		float denom = 1.0f / (va + vb + vc);
		float v = vb * denom;
		float w = vc * denom;
		return a + ab * v + ac * w;
	}

	bool checkSphereSphere(const btVector3& a, const btVector3& b, float ra, float rb, CollisionResult& res)
	{
		btVector3 diff = a - b;
		float dist2 = diff.length2();
		float radiusSum = ra + rb;

		if (dist2 > radiusSum * radiusSum)
			return false;

		float len = btSqrt(dist2);

		res.normOnB = btVector3(1, 0, 0);
		if (len > FLT_EPSILON)
			res.normOnB = diff / len;

		res.depth = len - radiusSum;
		res.posA = a - res.normOnB * ra;
		res.posB = b + res.normOnB * rb;

		return true;
	}

	bool checkSphereTriangle(const btVector3& s, float r, const CheckTriangle& tri, CollisionResult& res)
	{
		if (!tri.valid)
			return false;

		float radiusWithMargin = r + tri.margin;

		btVector3 closest = closestPointOnTriangle(s, tri.p0, tri.p1, tri.p2);

		btVector3 delta = s - closest;
		float dist2 = delta.length2();

		// Early out if sphere is too far from closest point
		if (dist2 > radiusWithMargin * radiusWithMargin)
			return false;

		btVector3 p1ToCentre = s - tri.p0;
		float distanceFromPlane = p1ToCentre.dot(tri.normal);

		auto normal = tri.normal;

		if (tri.prenetration >= FLT_EPSILON) {
			if (distanceFromPlane < -tri.prenetration || distanceFromPlane > radiusWithMargin)
				return false;
		}

		if (dist2 < FLT_EPSILON * FLT_EPSILON) {
			res.normOnB = normal;
			res.depth = -radiusWithMargin;
			res.posA = s - normal * r;
			res.posB = closest;
			return true;
		}

		float dist = btSqrt(dist2);

		// use face normal for face contacts, delta direction for edge/vertex contacts
		btVector3 contactNormal;
		if (distanceFromPlane > 0 && dist2 <= (distanceFromPlane * distanceFromPlane + FLT_EPSILON)) {
			contactNormal = normal;
		} else {
			contactNormal = delta / dist;
			if (contactNormal.dot(normal) < 0) {
				if (tri.prenetration < FLT_EPSILON)
					contactNormal = -contactNormal;
				else
					return false;
			}
		}

		res.normOnB = contactNormal;
		res.depth = dist - radiusWithMargin;
		res.posA = s - contactNormal * r;
		res.posB = closest;

		return res.depth < -FLT_EPSILON;
	}

	[[maybe_unused]] static bool linePlaneIntersection(btVector3& contact, const btVector3& p0, const btVector3& p1, const btVector3& normal, const btVector3& coord, float radius)
	{
		float d = normal.dot(coord);
		auto dir = p1 - p0;
		if (normal.dot(dir) < FLT_EPSILON)
			return false;  // No intersection, the line is parallel to the plane

		// Compute the X value for the directed line ray intersecting the plane
		float e = radius / dir.length();
		float x = (d - normal.dot(p0)) / normal.dot(dir);
		if (x <= 0.f || x >= 1.f + e)
			return false;

		x = btClamped(x - e, 0.f, 1.f);
		// output contact point
		contact = p0 + dir * x;
		return true;
	}
}
