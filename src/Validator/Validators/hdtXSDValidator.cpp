#include "hdtXSDValidator.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"
#include "../Config/hdtValidatorPaths.h"
#include "../hdtXSDSchemaModel.h"
#include "../Parser/hdtXSDSchemaParser.h"
#include "../Utils/hdtStringUtils.h"

#include <pugixml.hpp>

#include <charconv>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdt
{

	// XSD schema parsing helpers live in hdtXSDSchemaParser.cpp.

	static PhysicsSchema g_physicsSchema;
	static std::once_flag g_schemaOnce;

	/// Returns the cached physics schema, loading and parsing it on first access.
	/// Loading is thread-safe via std::call_once and failures are logged once.
	static const PhysicsSchema& getOrLoadPhysicsSchema()
	{
		std::call_once(g_schemaOnce, []() {
			// Use load_file (direct filesystem) only: schema files are always on disk
			// and readAllFile (BSA VFS) is unsafe before BSAs are mounted during SKSEPlugin_Load.
			pugi::xml_document doc;
			auto res = doc.load_file(kPhysicsXSDPath);

			if (!res) {
				logger::error(
					"[XSDValidator] Could not load physics schema from '{}': {}; "
					"physics XML validation will be skipped.",
					kPhysicsXSDPath, res.description());
				return;
			}

			try {
				g_physicsSchema = PhysicsSchema{};
				ParsePhysicsSchemaFromXSD(doc, g_physicsSchema);
				g_physicsSchema.loaded = true;

				logger::info(
					"[XSDValidator] Loaded physics schema: root '{}', "
					"{} enumerated element type(s), {} elements with required attr(s), "
					"{} elements with allowed-children constraint(s), "
					"{} with text type constraint(s), {} with attr type constraint(s).",
					g_physicsSchema.rootTag,
					g_physicsSchema.elementEnums.size(),
					g_physicsSchema.requiredAttrs.size(),
					g_physicsSchema.allowedChildren.size(),
					g_physicsSchema.elementTextConstraints.size(),
					g_physicsSchema.elementAttrConstraints.size());
			} catch (const std::exception& e) {
				logger::warn("[XSDValidator] Failed to parse physics schema '{}': {}",
					kPhysicsXSDPath, e.what());
			} catch (...) {
				logger::warn("[XSDValidator] Failed to parse physics schema '{}'.",
					kPhysicsXSDPath);
			}
		});

		return g_physicsSchema;
	}

	// ---- public schema accessors ----

	/// Exposes the schema-derived allowed-children map used by XML improvers/validators.
	const std::unordered_map<std::string, std::unordered_set<std::string>>& GetSchemaAllowedChildren()
	{
		return getOrLoadPhysicsSchema().allowedChildren;
	}

	/// Exposes the set of element names known to the parsed XSD schema.
	const std::unordered_set<std::string>& GetSchemaKnownElements()
	{
		return getOrLoadPhysicsSchema().knownElements;
	}

	// ---- validation context ----

	struct ValidationContext
	{
		std::string xmlPath;
		std::vector<XSDViolation>& violations;
		std::vector<std::string> elementStack;
		// Generic referential integrity tracking — driven by XSD key/unique/keyref.
		// keyValues: key/unique name — set of declared values encountered during parsing.
		// keyRefPending: keyref name — list of (line, value) pairs to verify after parsing.
		std::unordered_map<std::string, std::unordered_set<std::string>> keyValues;
		std::unordered_map<std::string, std::vector<std::pair<uint64_t, std::string>>> keyRefPending;

		/// Adds a validation violation with the current stack-derived element path.
		void addViolationWithCurrentPath(uint64_t line, uint64_t col, const std::string& msg)
		{
			std::string path;
			for (const auto& e : elementStack) {
				path += "/" + e;
			}
			violations.push_back({ xmlPath, line, col, path, msg });
		}
	};

	// ---- element validators ----

	/// Validates a raw text value against an XSD-derived type constraint.
	/// Trims surrounding whitespace and reports parse/range violations via ValidationContext.
	static void validateValueAgainstTypeConstraint(
		const std::string& tag, const std::string& attrName,
		const std::string& rawVal, const TypeConstraint& tc,
		uint64_t row, uint64_t col, ValidationContext& ctx)
	{
		const auto b = rawVal.find_first_not_of(" \t\r\n");
		const auto e = rawVal.find_last_not_of(" \t\r\n");
		const std::string val = (b == std::string::npos) ? "" : rawVal.substr(b, e - b + 1);

		const std::string where = attrName.empty() ? "<" + tag + ">" : "<" + tag + "> attribute '" + attrName + "'";

		if (tc.base == TypeConstraint::Base::Boolean) {
			if (val != "true" && val != "false" && val != "1" && val != "0")
				ctx.addViolationWithCurrentPath(row, col, where + " has invalid boolean value '" + val + "'");
			return;
		}

		if (tc.base == TypeConstraint::Base::Float || tc.base == TypeConstraint::Base::Integer) {
			double dval = 0.0;
			const auto [ptr, ec] = std::from_chars(val.data(), val.data() + val.size(), dval);
			if (ec != std::errc{} || ptr != val.data() + val.size()) {
				const std::string typeName = (tc.base == TypeConstraint::Base::Float) ? "float" : "integer";
				ctx.addViolationWithCurrentPath(row, col, where + " has invalid " + typeName + " value '" + val + "'");
				return;
			}
			if (tc.hasMin && dval < tc.minInclusive)
				ctx.addViolationWithCurrentPath(row, col, where + " value " + val + " is below minimum " + std::to_string(tc.minInclusive));
			if (tc.hasMax && dval > tc.maxInclusive)
				ctx.addViolationWithCurrentPath(row, col, where + " value " + val + " is above maximum " + std::to_string(tc.maxInclusive));
		}
	}

	/// Recursively validates descendant elements under a parent tag.
	/// Enforces allowed children, required attributes, key/keyref tracking,
	/// and enum/type-constrained text values based on the parsed schema.
	static void validateElementSubtree(
		const std::string& parentTag, XMLReader& reader,
		ValidationContext& ctx, const PhysicsSchema& schema)
	{
		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag)
				return;
			if (reader.GetInspected() != XMLReader::Inspected::StartTag)
				continue;

			const std::string tag = reader.GetLocalName();

			// Check that this element is allowed inside parentTag (XSD content model).
			const auto parentIt = schema.allowedChildren.find(parentTag);
			if (parentIt != schema.allowedChildren.end() &&
				!parentIt->second.count(tag)) {
				ctx.addViolationWithCurrentPath(reader.GetRow(), reader.GetColumn(),
					"<" + tag + "> is not allowed inside <" + parentTag + ">");
				reader.skipCurrentElement();
				continue;
			}

			// Generic key/keyref tracking (XSD-driven referential integrity).
			for (const auto& [kn, kd] : schema.keyDefs) {
				if (kd.elems.count(tag) && reader.hasAttribute(kd.fieldAttr))
					ctx.keyValues[kn].insert(reader.getAttribute(kd.fieldAttr));
			}
			for (const auto& [rn, rd] : schema.keyRefDefs) {
				if (rd.elems.count(tag) && reader.hasAttribute(rd.fieldAttr))
					ctx.keyRefPending[rn].emplace_back(reader.GetRow(), reader.getAttribute(rd.fieldAttr));
			}

			ctx.elementStack.push_back(tag);

			// Generic required attribute check (XSD-driven, no hardcoded names).
			if (schema.requiredAttrs.count(tag)) {
				for (const auto& attr : schema.requiredAttrs.at(tag)) {
					if (!reader.hasAttribute(attr))
						ctx.addViolationWithCurrentPath(reader.GetRow(), reader.GetColumn(),
							"<" + tag + "> is missing required attribute '" + attr + "'");
				}
			}

			// Attribute value type validation (XSD-driven, no hardcoded names).
			const auto attrConstraintIt = schema.elementAttrConstraints.find(tag);
			if (attrConstraintIt != schema.elementAttrConstraints.end()) {
				const uint64_t row = reader.GetRow(), col = reader.GetColumn();
				for (const auto& [attrName, tc] : attrConstraintIt->second) {
					if (reader.hasAttribute(attrName))
						validateValueAgainstTypeConstraint(tag, attrName, reader.getAttribute(attrName), tc, row, col, ctx);
				}
			}

			// Enum-constrained text content — read and validate; otherwise recurse into children.
			if (schema.elementEnums.count(tag)) {
				const uint64_t row = reader.GetRow(), col = reader.GetColumn();
				const std::string val = reader.readText();
				const auto& allowed = schema.elementEnums.at(tag);
				if (!allowed.count(val))
					ctx.addViolationWithCurrentPath(row, col,
						"<" + tag + "> has invalid value '" + val +
							"' (see hdtSMP64.xsd for valid values: " + JoinSortedSet(allowed) + ")");
			} else if (schema.elementTextConstraints.count(tag)) {
				const uint64_t row = reader.GetRow(), col = reader.GetColumn();
				const std::string val = reader.readText();
				validateValueAgainstTypeConstraint(tag, "", val, schema.elementTextConstraints.at(tag), row, col, ctx);
			} else {
				validateElementSubtree(tag, reader, ctx, schema);
			}

			ctx.elementStack.pop_back();
		}
	}

	/// Validates the <system> subtree using the currently loaded schema root tag.
	static void validateSystemRootSubtree(XMLReader& reader, ValidationContext& ctx)
	{
		const PhysicsSchema& schema = getOrLoadPhysicsSchema();
		ctx.elementStack.push_back(schema.rootTag);
		validateElementSubtree(schema.rootTag, reader, ctx, schema);
		ctx.elementStack.pop_back();
	}

	// ---- public API ----

	/// Validates a physics XML file against the parsed hdtSMP64 XSD constraints.
	/// Returns pass/fail status and a detailed list of violations with line/path context.
	XSDValidationResult ValidatePhysicsXMLWithXSD(const std::string& xmlPath)
	{
		XSDValidationResult result;

		// Skip validation entirely if the physics schema could not be loaded.
		// The error was already logged once by getOrLoadPhysicsSchema().
		if (!getOrLoadPhysicsSchema().loaded) {
			result.isValid = true;
			result.schemaSkipped = true;
			return result;
		}

		// Use readAllFile2 (direct filesystem) only: physics XML files are on disk
		// and readAllFile (BSA VFS) is unsafe before BSAs are mounted during SKSEPlugin_Load.
		auto bytes = readAllFile2(xmlPath.c_str());

		if (bytes.empty()) {
			result.isValid = false;
			result.violations.push_back({ xmlPath, 0, 0, "", "File not found or empty" });
			return result;
		}

		const PhysicsSchema& schema = getOrLoadPhysicsSchema();
		ValidationContext ctx{
			xmlPath,
			result.violations,
			{}
		};

		try {
			XMLReader reader((uint8_t*)bytes.data(), bytes.size());
			bool foundSystem = false;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string tag = reader.GetLocalName();
					if (tag == schema.rootTag) {
						foundSystem = true;
						validateSystemRootSubtree(reader, ctx);
						// Referential integrity — check all pending keyref values against declared keys.
						const std::unordered_set<std::string> emptySet;
						for (const auto& [refName, refDef] : schema.keyRefDefs) {
							const auto pendIt = ctx.keyRefPending.find(refName);
							if (pendIt == ctx.keyRefPending.end())
								continue;
							const auto keyIt = ctx.keyValues.find(refDef.refer);
							const auto& declared =
								(keyIt != ctx.keyValues.end()) ? keyIt->second : emptySet;
							for (const auto& [line, val] : pendIt->second) {
								if (!val.empty() && !declared.count(val)) {
									result.violations.push_back({ xmlPath, line, 0,
										"/" + schema.rootTag,
										"Undeclared reference '" + val + "' (must be declared in '" +
											refDef.refer + "')" });
								}
							}
						}
					} else {
						ctx.addViolationWithCurrentPath(reader.GetRow(), reader.GetColumn(),
							"Root element must be <" + schema.rootTag + ">, found <" + tag + ">");
						reader.skipCurrentElement();
					}
				}
			}

			if (!foundSystem) {
				result.violations.push_back({ xmlPath, 0, 0, "/", "No <" + schema.rootTag + "> root element found" });
			}
		} catch (const std::exception& e) {
			result.violations.push_back(
				{ xmlPath, 0, 0, "", std::string("XML parse error: ") + e.what() });
		} catch (...) {
			result.violations.push_back({ xmlPath, 0, 0, "", "Unknown XML parse error" });
		}

		result.isValid = result.violations.empty();
		return result;
	}

}  // namespace hdt
