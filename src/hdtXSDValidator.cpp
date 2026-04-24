#include "hdtXSDValidator.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"

#include <string>
#include <unordered_set>
#include <vector>

namespace hdt
{
	static const std::unordered_set<std::string> kValidSharedValues = {
		"public", "private", "internal", "external"
	};

	static const std::unordered_set<std::string> kValidShapeTypes = {
		"box", "sphere", "capsule", "hull", "cylinder", "compound", "ref"
	};

	static const std::unordered_set<std::string> kConstraintTags = {
		"generic-constraint", "stiffspring-constraint", "conetwist-constraint"
	};

	// ---- validation context ----

	struct ValidationContext
	{
		std::string xmlPath;
		std::vector<XSDViolation>& violations;
		std::vector<std::string> elementStack;
		std::unordered_set<std::string>& definedBones;
		std::unordered_set<std::string>& definedBodies;
		std::vector<std::string>& weightThresholdBones;
		bool& hasWeightThreshold;

		void addViolation(uint64_t line, uint64_t col, const std::string& msg)
		{
			std::string path;
			for (const auto& e : elementStack) {
				path += "/" + e;
			}
			violations.push_back({ xmlPath, line, col, path, msg });
		}
	};

	// ---- forward declarations ----

	static void validateSystem(XMLReader& reader, ValidationContext& ctx);
	static void validateBone(XMLReader& reader, ValidationContext& ctx);
	static void validateConstraint(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx);
	static void validateShape(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx);
	static void validateWeightThreshold(XMLReader& reader, ValidationContext& ctx);

	// ---- element validators ----

	static void validateSystem(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("system");

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				ctx.elementStack.pop_back();
				return;
			}
			if (reader.GetInspected() != XMLReader::Inspected::StartTag) {
				continue;
			}

			const std::string tag = reader.GetLocalName();

			if (tag == "bone") {
				validateBone(reader, ctx);
			} else if (kConstraintTags.count(tag)) {
				validateConstraint(reader, tag, ctx);
			} else if (kValidShapeTypes.count(tag) || tag == "per-triangle-shape" ||
				tag == "per-vertex-shape") {
				validateShape(reader, tag, ctx);
			} else if (tag == "weight-threshold") {
				validateWeightThreshold(reader, ctx);
			} else {
				reader.skipCurrentElement();
			}
		}

		if (!ctx.elementStack.empty() && ctx.elementStack.back() == "system") {
			ctx.elementStack.pop_back();
		}
	}

	static void validateBone(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("bone");

		if (!reader.hasAttribute("name")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<bone> is missing required attribute 'name'");
		} else {
			ctx.definedBones.insert(reader.getAttribute("name"));
		}

		while (reader.Inspect()) {
			if (reader.GetInspected() == XMLReader::Inspected::EndTag) {
				ctx.elementStack.pop_back();
				return;
			}
			if (reader.GetInspected() != XMLReader::Inspected::StartTag) {
				continue;
			}

			const std::string tag = reader.GetLocalName();
			ctx.elementStack.push_back(tag);

			if (tag == "shared") {
				// Read text and validate against allowed values
				const std::string val = reader.readText();
				if (!kValidSharedValues.count(val)) {
					ctx.addViolation(reader.GetRow(), reader.GetColumn(),
						"<shared> has invalid value '" + val +
							"' (expected: public, private, internal, external)");
				}
			} else if (kValidShapeTypes.count(tag) || tag == "per-triangle-shape" ||
				tag == "per-vertex-shape") {
				ctx.elementStack.pop_back();
				validateShape(reader, tag, ctx);
				continue;  // validateShape manages its own stack entry
			} else {
				reader.skipCurrentElement();
			}

			ctx.elementStack.pop_back();
		}

		if (!ctx.elementStack.empty() && ctx.elementStack.back() == "bone") {
			ctx.elementStack.pop_back();
		}
	}

	static void validateConstraint(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx)
	{
		ctx.elementStack.push_back(tag);

		if (!reader.hasAttribute("bodyA")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<" + tag + "> is missing required attribute 'bodyA'");
		} else {
			ctx.definedBodies.insert(reader.getAttribute("bodyA"));
		}

		if (!reader.hasAttribute("bodyB")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<" + tag + "> is missing required attribute 'bodyB'");
		} else {
			ctx.definedBodies.insert(reader.getAttribute("bodyB"));
		}

		reader.skipCurrentElement();
		ctx.elementStack.pop_back();
	}

	static void validateShape(XMLReader& reader, const std::string& tag,
		ValidationContext& ctx)
	{
		ctx.elementStack.push_back(tag);

		if (!kValidShapeTypes.count(tag) && tag != "per-triangle-shape" &&
			tag != "per-vertex-shape") {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"Unknown shape type <" + tag +
					"> (expected: box, sphere, capsule, hull, cylinder, compound, ref)");
		}

		reader.skipCurrentElement();
		ctx.elementStack.pop_back();
	}

	static void validateWeightThreshold(XMLReader& reader, ValidationContext& ctx)
	{
		ctx.elementStack.push_back("weight-threshold");
		ctx.hasWeightThreshold = true;

		if (!reader.hasAttribute("bone")) {
			ctx.addViolation(reader.GetRow(), reader.GetColumn(),
				"<weight-threshold> is missing required attribute 'bone'");
		} else {
			ctx.weightThresholdBones.push_back(reader.getAttribute("bone"));
		}

		reader.skipCurrentElement();
		ctx.elementStack.pop_back();
	}

	// ---- public API ----

	XSDValidationResult ValidatePhysicsXML(const std::string& xmlPath)
	{
		XSDValidationResult result;

		// Try VFS first (works for mod-managed files), then fall back to direct filesystem
		auto bytes = readAllFile(xmlPath.c_str());
		if (bytes.empty()) {
			bytes = readAllFile2(xmlPath.c_str());
		}

		if (bytes.empty()) {
			result.isValid = false;
			result.violations.push_back({ xmlPath, 0, 0, "", "File not found or empty" });
			return result;
		}

		// Store original locale and switch to en_US for numeric parsing
		char saved_locale[64];
		strcpy_s(saved_locale, std::setlocale(LC_NUMERIC, nullptr));
		std::setlocale(LC_NUMERIC, "en_US");

		ValidationContext ctx{
			xmlPath,
			result.violations,
			{},
			result.definedBones,
			result.definedBodies,
			result.weightThresholdBones,
			result.hasWeightThreshold
		};

		try {
			XMLReader reader((uint8_t*)bytes.data(), bytes.size());
			bool foundSystem = false;

			while (reader.Inspect()) {
				if (reader.GetInspected() == XMLReader::Inspected::StartTag) {
					const std::string tag = reader.GetLocalName();
					if (tag == "system") {
						foundSystem = true;
						validateSystem(reader, ctx);
					} else {
						ctx.addViolation(reader.GetRow(), reader.GetColumn(),
							"Root element must be <system>, found <" + tag + ">");
						reader.skipCurrentElement();
					}
				}
			}

			if (!foundSystem) {
				result.violations.push_back({ xmlPath, 0, 0, "/", "No <system> root element found" });
			}
		} catch (const std::exception& e) {
			result.violations.push_back(
				{ xmlPath, 0, 0, "", std::string("XML parse error: ") + e.what() });
		} catch (...) {
			result.violations.push_back({ xmlPath, 0, 0, "", "Unknown XML parse error" });
		}

		std::setlocale(LC_NUMERIC, saved_locale);

		result.isValid = result.violations.empty();
		return result;
	}

}  // namespace hdt

