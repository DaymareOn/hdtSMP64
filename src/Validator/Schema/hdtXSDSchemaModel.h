#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// Generic value type constraint derived from xs:restriction facets in the XSD.
	// All fields are populated from the XSD at load time -- no type names are hardcoded.
	struct TypeConstraint
	{
		enum class Base
		{
			Any,
			Float,
			Boolean,
			Integer
		} base = Base::Any;
		bool hasMin = false, hasMax = false;
		double minInclusive = 0.0, maxInclusive = 0.0;
	};

	// Parsed schema enumerations loaded once from hdtSMP64.xsd.
	struct PhysicsSchema
	{
		// Maps element tag names to their allowed values (inline anonymous simpleType enumerations).
		std::unordered_map<std::string, std::unordered_set<std::string>> elementEnums;
		// Root element tag for physics XML files.
		std::string rootTag;
		// Required attributes per element -- driven entirely by xs:attribute use="required".
		std::unordered_map<std::string, std::vector<std::string>> requiredAttrs;
		// Key/unique and keyref definitions parsed from xsd:key, xsd:unique, xsd:keyref.
		struct KeyDef
		{
			std::unordered_set<std::string> elems;
			std::string fieldAttr;
		};
		struct KeyRefDef
		{
			std::string refer;
			std::unordered_set<std::string> elems;
			std::string fieldAttr;
		};
		std::unordered_map<std::string, KeyDef> keyDefs;        // key/unique name -> declared value sets
		std::unordered_map<std::string, KeyRefDef> keyRefDefs;  // keyref name -> reference definitions
		// Allowed children per element from XSD content model (xs:choice/xs:all/xs:sequence).
		std::unordered_map<std::string, std::unordered_set<std::string>> allowedChildren;
		// All element tag names mentioned anywhere in the schema.
		std::unordered_set<std::string> knownElements;
		// Per element: type constraint on text content for non-enum typed elements.
		std::unordered_map<std::string, TypeConstraint> elementTextConstraints;
		// Per element: per-attribute type constraint on attribute values.
		std::unordered_map<std::string, std::unordered_map<std::string, TypeConstraint>> elementAttrConstraints;
		bool loaded = false;  // true only when the XSD was successfully parsed
	};

}  // namespace hdt
