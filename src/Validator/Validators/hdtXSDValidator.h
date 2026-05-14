#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
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
		bool schemaSkipped = false;  // true when the XSD schema was unavailable; isValid is set to true but no real validation occurred
		std::vector<XSDViolation> violations;
	};

	// Validate an FSMP physics XML file against the hdtSMP64 XSD constraints.
	// Checks required attributes, valid enum values, and cross-references within the XML.
	XSDValidationResult ValidatePhysicsXMLWithXSD(const std::string& xmlPath);

	// Access to the parsed XSD schema, for use by hdtXMLImprover.
	// The schema is loaded on first call to ValidatePhysicsXML and cached thereafter.
	// Safe to call from any thread once the schema is loaded.
	const std::unordered_map<std::string, std::unordered_set<std::string>>& GetSchemaAllowedChildren();
	const std::unordered_set<std::string>& GetSchemaKnownElements();

}  // namespace hdt
