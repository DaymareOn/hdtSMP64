#include "hdtCollisionAlgorithm.h"

namespace hdt
{

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
}
