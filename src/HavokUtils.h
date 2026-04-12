#pragma once

namespace hdt::havok
{
	inline const RE::hkaSkeleton* getAnimationSkeleton(RE::Actor* a_actor)
	{
		RE::BSAnimationGraphManagerPtr graphManager;
		if (!a_actor->GetAnimationGraphManager(graphManager) || !graphManager || graphManager->graphs.empty())
			return nullptr;

		const auto& graph = graphManager->graphs[0];
		if (!graph || !graph->characterInstance.setup)
			return nullptr;

		return reinterpret_cast<const RE::hkaSkeleton*>(graph->characterInstance.setup->animationSkeleton.get());
	}

	inline bool getReferencePoseByIndex(const RE::hkaSkeleton* a_skeleton, std::int32_t a_boneIndex, RE::hkQsTransform& a_outTransform)
	{
		if (a_boneIndex < 0 || a_boneIndex >= static_cast<std::int32_t>(a_skeleton->referencePose.size()))
			return false;

		a_outTransform = a_skeleton->referencePose[a_boneIndex];
		return true;
	}

	inline RE::NiTransform hKQsTransformToNiTransform(const RE::hkQsTransform& a_hkTransform)
	{
		const auto* r = a_hkTransform.rotation.vec.quad.m128_f32;
		const float qx = r[0], qy = r[1], qz = r[2], qw = r[3];
		const float sqx = qx * qx, sqy = qy * qy, sqz = qz * qz, sqw = qw * qw;
		const float invs = 1.0f / (sqx + sqy + sqz + sqw);

		RE::NiTransform result;

		result.rotate.entry[0][0] = (sqx - sqy - sqz + sqw) * invs;
		result.rotate.entry[1][1] = (-sqx + sqy - sqz + sqw) * invs;
		result.rotate.entry[2][2] = (-sqx - sqy + sqz + sqw) * invs;

		auto cross = [&](float a, float b, float c, float d, int i, int j) {
			result.rotate.entry[i][j] = 2.0f * (a * b + c * d) * invs;
			result.rotate.entry[j][i] = 2.0f * (a * b - c * d) * invs;
		};

		cross(qx, qy, qz, qw, 1, 0);
		cross(qx, qz, qy, qw, 0, 2);
		cross(qy, qz, qx, qw, 2, 1);

		const auto* t = a_hkTransform.translation.quad.m128_f32;
		result.translate.x = t[0];
		result.translate.y = t[1];
		result.translate.z = t[2];
		result.scale = a_hkTransform.scale.quad.m128_f32[0];

		return result;
	}
}
