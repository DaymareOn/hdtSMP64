#include "hdtNIFValidator.h"

#include "NetImmerseUtils.h"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace hdt
{
	namespace
	{
		constexpr float kMinSaneScale = 1e-6f;
		constexpr float kMaxSaneScale = 1000.f;
		constexpr float kRootScaleDeviationThreshold = 0.5f;

		/// Validates a single node transform for numeric sanity and plausible scale range.
		/// Appends descriptive errors for NaN/inf values or extreme scale values.
		/// Returns true when all checks pass, false otherwise.
		bool validateNodeTransform(const RE::NiTransform& xfm, const std::string& boneName,
			std::vector<std::string>& errors)
		{
			bool ok = true;
			const auto& t = xfm.translate;
			if (std::isnan(t.x) || std::isnan(t.y) || std::isnan(t.z) ||
				std::isinf(t.x) || std::isinf(t.y) || std::isinf(t.z)) {
				errors.push_back("Bone '" + boneName + "': NaN/inf translation");
				ok = false;
			}
			if (std::isnan(xfm.scale) || std::isinf(xfm.scale)) {
				errors.push_back("Bone '" + boneName + "': NaN/inf scale");
				ok = false;
			}
			if (xfm.scale < kMinSaneScale) {
				errors.push_back(
					"Bone '" + boneName + "': scale is zero or near-zero (" +
					std::to_string(xfm.scale) + ")");
				ok = false;
			}
			if (xfm.scale > kMaxSaneScale) {
				errors.push_back(
					"Bone '" + boneName + "': scale is extreme (" +
					std::to_string(xfm.scale) + ")");
				ok = false;
			}
			return ok;
		}

		/// Recursively validates mesh skinning data in a scene subtree.
		/// Records warnings for missing skin instances and errors for malformed
		/// skinData/bone references, while updating aggregate skinning counters.
		void validateSkinningSubtree(RE::NiAVObject* obj, NIFStructuralResult& result)
		{
			if (!obj)
				return;

			RE::BSTriShape* triShape = castBSTriShape(obj);
			if (triShape) {
				auto& runtimeData = triShape->GetGeometryRuntimeData();
				if (!runtimeData.skinInstance) {
					result.warnings.push_back(
						std::string("Mesh '") + triShape->name.c_str() +
						"' has no NiSkinInstance");
				} else {
					result.hasSkinningData = true;
					RE::NiSkinInstance* skinInst = runtimeData.skinInstance.get();

					if (!skinInst->skinData) {
						result.errors.push_back(
							std::string("Mesh '") + triShape->name.c_str() +
							"' has null skinData");
					} else {
						RE::NiSkinData* skinData = skinInst->skinData.get();
						result.boneCount += skinData->GetBoneCount();

						if (skinData->GetBoneCount() == 0) {
							result.errors.push_back(
								std::string("Mesh '") + triShape->name.c_str() +
								"' has zero bones in skinData");
						}

						if (!skinInst->bones) {
							result.errors.push_back(
								std::string("Mesh '") + triShape->name.c_str() +
								"' has null bones array in skinInstance");
						} else {
							for (uint32_t i = 0; i < skinData->GetBoneCount(); ++i) {
								if (!skinInst->bones[i]) {
									result.errors.push_back(
										std::string("Mesh '") + triShape->name.c_str() +
										"' has null bone[" + std::to_string(i) + "]");
								}
							}
						}
					}
				}
			}

			RE::NiNode* node = castNiNode(obj);
			if (node) {
				for (auto& child : node->GetChildren()) {
					if (child) {
						validateSkinningSubtree(child.get(), result);
					}
				}
			}
		}
		/// Recursively validates node transforms in a skeleton subtree.
		/// Calls validateNodeTransform on every NiNode; errors accumulate in result.errors.
		void validateNodeTransformsSubtree(RE::NiNode* node, std::vector<std::string>& errors)
		{
			if (!node)
				return;
			std::string name = node->name.empty() ? "<unnamed>" : node->name.c_str();
			validateNodeTransform(node->world, name, errors);
			for (auto& child : node->GetChildren()) {
				RE::NiNode* childNode = castNiNode(child.get());
				if (childNode)
					validateNodeTransformsSubtree(childNode, errors);
			}
		}

	}  // namespace

	void CollectNamedSkeletonNodes(RE::NiNode* root, std::vector<std::string>& outNames)
	{
		if (!root)
			return;

		const char* name = root->name.c_str();
		if (name && name[0] != '\0') {
			outNames.push_back(name);
		}

		for (auto& child : root->GetChildren()) {
			if (!child)
				continue;
			RE::NiNode* childNode = castNiNode(child.get());
			if (childNode) {
				CollectNamedSkeletonNodes(childNode, outNames);
			}
		}
	}

	/// Validates runtime NIF structure rooted at a NiNode.
	/// Performs bone discovery, transform sanity checks, and skinning integrity checks,
	/// then returns a consolidated structural validation result with errors/warnings.
	NIFStructuralResult ValidateNIFStructure(RE::NiNode* root, const std::string& nifPath)
	{
		NIFStructuralResult result;

		if (!root) {
			result.isValid = false;
			result.errors.push_back(nifPath + ": root NiNode is null");
			return result;
		}

		CollectNamedSkeletonNodes(root, result.boneNames);
		result.boneCount = static_cast<uint32_t>(result.boneNames.size());

		if (result.boneCount == 0) {
			result.errors.push_back(nifPath + ": no bones found in skeleton");
		}

		validateNodeTransformsSubtree(root, result.errors);

		validateSkinningSubtree(root, result);

		if (!result.hasSkinningData) {
			result.warnings.push_back(nifPath + ": no skinned meshes found");
		}

		if (root->world.scale > kMinSaneScale && std::abs(root->world.scale - 1.0f) > kRootScaleDeviationThreshold) {
			result.warnings.push_back(nifPath + ": root scale " +
									  std::to_string(root->world.scale) + " deviates significantly from 1.0");
		}

		result.isValid = result.errors.empty();
		return result;
	}

}  // namespace hdt
