#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	struct XSDViolation
	{
		std::string xmlPath;
		uint64_t line;
		uint64_t column;
		std::string elementPath;
		std::string message;
	};

	struct XSDValidationResult
	{
		bool isValid = true;
		bool hasWeightThreshold = false;
		std::vector<XSDViolation> violations;
		std::unordered_set<std::string> definedBones;
		std::unordered_set<std::string> definedBodies;
		std::vector<std::string> weightThresholdBones;
	};

	// Validate an FSMP physics XML file against the hdtSMP64 XSD constraints.
	// Checks required attributes, valid enum values, and cross-references within the XML.
	XSDValidationResult ValidatePhysicsXML(const std::string& xmlPath);

}  // namespace hdt
