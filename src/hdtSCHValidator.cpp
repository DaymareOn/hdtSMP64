#include "hdtSCHValidator.h"

#include "hdtValidatorPaths.h"
#include "NetImmerseUtils.h"

#include <pugixml.hpp>

namespace hdt
{
	// A single compiled Schematron rule: an absolute XPath expression (namespace-stripped
	// and document-rooted) plus the assertion message and role.
	struct CompiledRule
	{
		std::string xpathExpr;  // e.g. "//inertia[parent::bone-default[mass = '0'] or ...]"
		std::string message;
		SCHRole role = SCHRole::Warning;
	};

	struct CompiledSchema
	{
		std::vector<CompiledRule> rules;
		bool loaded = false;
	};

	// ---- XPath expression transformation ----

	// Strip the "f:" namespace prefix from an XPath expression.
	// Only strips "f:" that is preceded by an XPath boundary character to avoid
	// false-positives inside string literals (none exist in this .sch file).
	static std::string stripFPrefix(const std::string& expr)
	{
		std::string result;
		result.reserve(expr.size());
		for (size_t i = 0; i < expr.size();) {
			if (i + 1 < expr.size() && expr[i] == 'f' && expr[i + 1] == ':') {
				bool atBoundary = (i == 0);
				if (!atBoundary && !result.empty()) {
					char prev = result.back();
					atBoundary = (prev == '/' || prev == '[' || prev == '(' ||
								  prev == '|' || prev == ' ' || prev == '\t' ||
								  prev == ':');
				}
				if (atBoundary) {
					i += 2;  // skip "f:"
					continue;
				}
			}
			result += expr[i++];
		}
		return result;
	}

	static std::string trim(const std::string& s)
	{
		auto start = s.find_first_not_of(" \t\r\n");
		if (start == std::string::npos)
			return "";
		auto end = s.find_last_not_of(" \t\r\n");
		return s.substr(start, end - start + 1);
	}

	// Convert a Schematron context match pattern (after namespace stripping) to an
	// absolute XPath expression for pugixml select_nodes().
	//
	// Handles the union-at-root form "(elem1 | elem2)/child[pred]" by expanding it to
	// "//elem1/child[pred] | //elem2/child[pred]".  All other patterns are prefixed with "//".
	static std::string contextToAbsoluteXPath(const std::string& rawContext)
	{
		std::string ctx = trim(stripFPrefix(rawContext));
		if (ctx.empty())
			return "";

		if (ctx.front() == '(') {
			size_t closePos = ctx.find(')');
			if (closePos != std::string::npos) {
				std::string unionPart = ctx.substr(1, closePos - 1);
				std::string rest = ctx.substr(closePos + 1);  // "/child[pred]" or ""

				std::vector<std::string> parts;
				size_t start = 0;
				while (true) {
					size_t sep = unionPart.find(" | ", start);
					if (sep == std::string::npos) {
						parts.push_back(trim(unionPart.substr(start)));
						break;
					}
					parts.push_back(trim(unionPart.substr(start, sep - start)));
					start = sep + 3;
				}

				std::string result;
				for (const auto& p : parts) {
					if (!result.empty())
						result += " | ";
					result += "//" + p + rest;
				}
				return result;
			}
		}

		return "//" + ctx;
	}

	// ---- schema loading ----

	static CompiledSchema g_compiledSchema;
	static std::once_flag g_schemaOnce;

	static const CompiledSchema& getCompiledSchema()
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

			// Strip the namespace prefix from a node name and return the local name.
			// pugixml is not namespace-aware, so we do this manually.
			auto localName = [](const char* name) -> std::string_view {
				std::string_view sv(name);
				auto colon = sv.rfind(':');
				return (colon != std::string_view::npos) ? sv.substr(colon + 1) : sv;
			};

			// Walk sch:schema/sch:pattern/sch:rule/sch:assert using local-name matching.
			auto schemaRoot = schDoc.first_child();  // <sch:schema>
			for (auto pattern : schemaRoot.children()) {
				if (localName(pattern.name()) != "pattern")
					continue;

				for (auto rule : pattern.children()) {
					if (localName(rule.name()) != "rule")
						continue;

					std::string contextAttr = rule.attribute("context").as_string();
					if (contextAttr.empty())
						continue;

					std::string xpathExpr = contextToAbsoluteXPath(contextAttr);
					if (xpathExpr.empty())
						continue;

					for (auto assertNode : rule.children()) {
						if (localName(assertNode.name()) != "assert")
							continue;

						std::string_view roleStr = assertNode.attribute("role").as_string("warning");
						SCHRole role = (roleStr == "error") ? SCHRole::Error : SCHRole::Warning;
						std::string message = trim(assertNode.text().as_string());

						g_compiledSchema.rules.push_back({ xpathExpr, message, role });
					}
				}
			}

			g_compiledSchema.loaded = true;
			logger::info("[SCHValidator] Loaded Schematron schema: {} rule(s).",
				g_compiledSchema.rules.size());
		});

		return g_compiledSchema;
	}

	// ---- XPath location helper ----

	// Build a human-readable location path for a pugixml node, e.g.
	// "/system[1]/bone[2]/linearDamping[1]".
	static std::string nodeLocation(const pugi::xml_node& node)
	{
		if (!node)
			return "/";

		std::string path;
		pugi::xml_node cur = node;
		while (cur && cur.type() != pugi::node_document) {
			std::string name = cur.name();

			int pos = 1;
			for (auto sib = cur.previous_sibling(); sib; sib = sib.previous_sibling()) {
				if (std::string(sib.name()) == name)
					++pos;
			}

			path = "/" + name + "[" + std::to_string(pos) + "]" + path;
			cur = cur.parent();
		}
		return path.empty() ? "/" : path;
	}

	// Convert a byte offset from pugi::xml_node::offset_debug() to a 1-based line number
	// by counting newlines in the source buffer up to (but not including) the offset.
	static int offsetToLine(const std::string& src, ptrdiff_t offset)
	{
		if (offset <= 0 || offset > static_cast<ptrdiff_t>(src.size()))
			return 1;
		int line = 1;
		for (ptrdiff_t i = 0; i < offset; ++i) {
			if (src[i] == '\n')
				++line;
		}
		return line;
	}

	// ---- public API ----

	SCHValidationResult ValidatePhysicsXMLWithSCH(const std::string& xmlPath)
	{
		SCHValidationResult result;

		const CompiledSchema& schema = getCompiledSchema();
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
				v.location = nodeLocation(xnode.node());
				v.line = offsetToLine(bytes, xnode.node().offset_debug());
				v.message = rule.message;
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
