#include "hdtSCHValidator.h"

#include "NetImmerseUtils.h"
#include "../hdtSCHSchemaModel.h"
#include "../Parser/hdtSCHSchemaParser.h"
#include "../Config/hdtValidatorPaths.h"
#include "../Utils/hdtStringUtils.h"
#include "../Utils/hdtXMLUtils.h"

#include <pugixml.hpp>

#include <mutex>

namespace hdt
{
	namespace
	{
		static void replaceAllInPlace(std::string& s, const std::string& from, const std::string& to)
		{
			if (from.empty())
				return;

			size_t pos = 0;
			while ((pos = s.find(from, pos)) != std::string::npos) {
				s.replace(pos, from.size(), to);
				pos += to.size();
			}
		}

		/// Expands parser placeholders using the currently matched XML node.
		static std::string resolveMessageTemplate(std::string msgTemplate, const pugi::xml_node& node)
		{
			replaceAllInPlace(msgTemplate, "{name}", std::string(XmlLocalName(node.name())));
			replaceAllInPlace(msgTemplate, "{value}", TrimAsciiWhitespace(node.text().as_string()));
			return msgTemplate;
		}
	}  // namespace

	// ---- schema loading ----

	static CompiledSchema g_compiledSchema;
	static std::once_flag g_schemaOnce;

	/// Returns the cached compiled Schematron schema, loading and compiling it
	/// from disk on first use. Initialization is thread-safe and runs only once.
	static const CompiledSchema& getOrLoadCompiledSchema()
	{
		std::call_once(g_schemaOnce, []() {
			// Use readAllFile2 (direct filesystem) only: schema files are always on disk
			// and readAllFile (BSA VFS) is unsafe before BSAs are mounted during SKSEPlugin_Load.
			std::string bytes = readAllFile2(kPhysicsSCHPath);

			if (bytes.empty()) {
				logger::warn(
					"[SCHValidator] Could not load Schematron schema from '{}'; "
					"Schematron validation will be skipped.",
					kPhysicsSCHPath);
				return;
			}

			pugi::xml_document schDoc;
			auto parseResult = schDoc.load_buffer(bytes.data(), bytes.size());
			if (!parseResult) {
				logger::warn("[SCHValidator] Failed to parse Schematron schema '{}': {}",
					kPhysicsSCHPath, parseResult.description());
				return;
			}

			if (!ParseCompiledSchemaFromSCH(schDoc, g_compiledSchema)) {
				logger::warn("[SCHValidator] Failed to compile Schematron schema '{}'.",
					kPhysicsSCHPath);
				return;
			}

			logger::info("[SCHValidator] Loaded Schematron schema: {} rule(s).",
				g_compiledSchema.rules.size());
		});

		return g_compiledSchema;
	}

	// ---- public API ----

	/// Validates a physics XML file against compiled Schematron rules.
	/// Returns all matched violations with location and line metadata, and sets
	/// hasErrors/hasWarnings flags according to each matched rule role.
	SCHValidationResult ValidatePhysicsXMLWithSchematron(const std::string& xmlPath)
	{
		SCHValidationResult result;

		const CompiledSchema& schema = getOrLoadCompiledSchema();
		if (!schema.loaded)
			return result;

		std::string bytes = readAllFile2(xmlPath.c_str());
		if (bytes.empty())
			return result;  // File-not-found is reported by the XSD validator

		pugi::xml_document doc;
		auto parseResult = doc.load_buffer(bytes.data(), bytes.size());
		if (!parseResult) {
			SCHViolation v;
			v.xmlPath = xmlPath;
			v.location = "/";
			v.message = std::string("XML parse error: ") + parseResult.description();
			v.role = SCHRole::Error;
			result.violations.push_back(std::move(v));
			result.hasErrors = true;
			return result;
		}

		for (const auto& rule : schema.rules) {
			pugi::xpath_node_set matches;
			try {
				matches = doc.select_nodes(rule.xpathExpr.c_str());
			} catch (const pugi::xpath_exception& e) {
				logger::warn("[SCHValidator] XPath error evaluating '{}': {}",
					rule.xpathExpr, e.what());
				continue;
			}

			for (const auto& xnode : matches) {
				SCHViolation v;
				v.xmlPath = xmlPath;
				v.location = BuildNodeLocationPath(xnode.node());
				v.line = OffsetToLineNumber(bytes, xnode.node().offset_debug());
				v.message = resolveMessageTemplate(rule.message, xnode.node());
				v.role = rule.role;

				if (rule.role == SCHRole::Error)
					result.hasErrors = true;
				else
					result.hasWarnings = true;

				result.violations.push_back(std::move(v));
			}
		}

		return result;
	}

}  // namespace hdt
