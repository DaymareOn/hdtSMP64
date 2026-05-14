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
		/// Recursively collects non-empty NiNode names from a skeleton subtree.
		/// The collected names are used as a simple bone inventory for diagnostics.
		void collectNamedSkeletonNodes(RE::NiNode* node, std::vector<std::string>& boneNames)
		{
			if (!node)
				return;

			const char* name = node->name.c_str();
			if (name && name[0] != '\0') {
				boneNames.push_back(name);
			}

			for (auto& child : node->GetChildren()) {
				if (!child)
					continue;
				RE::NiNode* childNode = castNiNode(child.get());
				if (childNode) {
					collectNamedSkeletonNodes(childNode, boneNames);
				}
			}
		}

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
			if (xfm.scale < 1e-6f) {
				errors.push_back(
					"Bone '" + boneName + "': scale is zero or near-zero (" +
					std::to_string(xfm.scale) + ")");
				ok = false;
			}
			if (xfm.scale > 1000.f) {
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
						result.boneCount += skinData->bones;

						if (skinData->bones == 0) {
							result.errors.push_back(
								std::string("Mesh '") + triShape->name.c_str() +
								"' has zero bones in skinData");
						}

						if (!skinInst->bones) {
							result.errors.push_back(
								std::string("Mesh '") + triShape->name.c_str() +
								"' has null bones array in skinInstance");
						} else {
							for (uint32_t i = 0; i < skinData->bones; ++i) {
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
	}  // namespace

	/// Validates runtime NIF structure rooted at a NiNode.
	/// Performs bone discovery, transform sanity checks, and skinning integrity checks,
	/// then returns a consolidated structural validation result with errors/warnings.
	NIFStructuralResult validateNIFStructure(RE::NiNode* root, const std::string& nifPath)
	{
		NIFStructuralResult result;

		if (!root) {
			result.isValid = false;
			result.errors.push_back(nifPath + ": root NiNode is null");
			return result;
		}

		collectNamedSkeletonNodes(root, result.boneNames);
		result.boneCount = static_cast<uint32_t>(result.boneNames.size());

		if (result.boneCount == 0) {
			result.isValid = false;
			result.errors.push_back(nifPath + ": no bones found in skeleton");
		}

		std::function<void(RE::NiNode*)> checkTransforms = [&](RE::NiNode* node) {
			if (!node)
				return;
			std::string name = node->name.empty() ? "<unnamed>" : node->name.c_str();
			validateNodeTransform(node->world, name, result.errors);
			for (auto& child : node->GetChildren()) {
				RE::NiNode* childNode = castNiNode(child.get());
				if (childNode) {
					checkTransforms(childNode);
				}
			}
		};
		checkTransforms(root);

		validateSkinningSubtree(root, result);

		if (!result.hasSkinningData) {
			result.warnings.push_back(nifPath + ": no skinned meshes found");
		}

		if (root->world.scale > 1e-6f && std::abs(root->world.scale - 1.0f) > 0.5f) {
			result.warnings.push_back(nifPath + ": root scale " +
				std::to_string(root->world.scale) + " deviates significantly from 1.0");
		}

		result.isValid = result.errors.empty();
		return result;
	}

}  // namespace hdt
