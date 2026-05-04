#pragma once

#include <cstdint>
#include <string>
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
		std::vector<XSDViolation> violations;
		// Advisory warnings (not errors): missing recommended elements, performance hints, etc.
		std::vector<XSDViolation> warnings;
	};

	// Validate an FSMP physics XML file against the hdtSMP64 XSD constraints.
	// Checks required attributes, valid enum values, and cross-references within the XML.
	XSDValidationResult ValidatePhysicsXML(const std::string& xmlPath);

}  // namespace hdt
