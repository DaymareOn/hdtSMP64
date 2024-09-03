#include "hdtConvertNi.h"

namespace hdt
{
	btQuaternion convertNi(const RE::NiMatrix3& rhs)
	{
		//using namespace DirectX;
		//auto mat = XMLoadFloat3x3((const XMFLOAT3X3*)&rhs);
		//auto q = XMQuaternionRotationMatrix(XMMatrixTranspose(mat));
		//if (XMQuaternionIsInfinite(q) || XMQuaternionIsNaN(q))
		//	return XMQuaternionIdentity();
		//return q;
		btMatrix3x3 mat;
		mat[0][0] = rhs.entry[0][0];
		mat[0][1] = rhs.entry[0][1];
		mat[0][2] = rhs.entry[0][2];
		mat[1][0] = rhs.entry[1][0];
		mat[1][1] = rhs.entry[1][1];
		mat[1][2] = rhs.entry[1][2];
		mat[2][0] = rhs.entry[2][0];
		mat[2][1] = rhs.entry[2][1];
		mat[2][2] = rhs.entry[2][2];
		btQuaternion q;
		mat.getRotation(q);
		return q;
	}

	RE::NiTransform convertBt(const btQsTransform& rhs)
	{
		RE::NiTransform ret;

		ret.rotate = convertBt(btMatrix3x3(rhs.getBasis()));
		ret.translate = convertBt(rhs.getOrigin());
		ret.scale = rhs.getScale();

		return ret;
	}

	RE::NiMatrix3 convertBt(const btQuaternion& rhs)
	{
		btMatrix3x3 mat(rhs.normalized());
		return convertBt(mat);
	}

	RE::NiPoint3 convertBt(const btVector3& rhs)
	{
		RE::NiPoint3 ret;
		ret.x = rhs[0];
		ret.y = rhs[1];
		ret.z = rhs[2];
		return ret;
	}

	RE::NiMatrix3 convertBt(const btMatrix3x3& rhs)
	{
		RE::NiMatrix3 ret;
		ret.entry[0][0] = rhs[0][0];
		ret.entry[0][1] = rhs[0][1];
		ret.entry[0][2] = rhs[0][2];
		ret.entry[1][0] = rhs[1][0];
		ret.entry[1][1] = rhs[1][1];
		ret.entry[1][2] = rhs[1][2];
		ret.entry[2][0] = rhs[2][0];
		ret.entry[2][1] = rhs[2][1];
		ret.entry[2][2] = rhs[2][2];
		return ret;
	}
}
