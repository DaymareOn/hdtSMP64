#include "hdtSCHSchemaParser.h"

#include "../Utils/hdtStringUtils.h"
#include "../Utils/hdtXMLUtils.h"

#include <string>
#include <string_view>
#include <vector>

namespace hdt
{
	namespace
	{
		/// Converts a Schematron rule context into an absolute XPath used for node selection.
		/// Expands root-level union forms like "(a | b)/c" into explicit XPath unions.
		std::string contextToAbsoluteXPath(const std::string& rawContext, const std::string& schemaPrefix)
		{
			std::string ctx = TrimAsciiWhitespace(StripNamespacePrefix(rawContext, schemaPrefix));
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
							parts.push_back(TrimAsciiWhitespace(unionPart.substr(start)));
							break;
						}
						parts.push_back(TrimAsciiWhitespace(unionPart.substr(start, sep - start)));
						start = sep + 3;
					}

					std::string result;
					for (const auto& part : parts) {
						if (!result.empty())
							result += " | ";
						result += "//" + part + rest;
					}
					return result;
				}
			}

			return "//" + ctx;
		}
	}  // namespace

	/// Parses a Schematron document into compiled XPath/message/role rules.
	/// Returns false only when the root is missing; otherwise schema.loaded is set true.
	bool ParseCompiledSchemaFromSCH(const pugi::xml_document& schDoc, CompiledSchema& schema)
	{
		schema = CompiledSchema{};

		auto schemaRoot = schDoc.first_child();
		if (!schemaRoot) {
			return false;
		}

		std::string schemaPrefix = "f";
		for (auto nsNode : schemaRoot.children()) {
			if (XmlLocalName(nsNode.name()) != "ns")
				continue;

			std::string_view uri = nsNode.attribute("uri").as_string();
			if (uri == "FSMP-Validator") {
				schemaPrefix = nsNode.attribute("prefix").as_string("f");
				break;
			}
		}

		for (auto pattern : schemaRoot.children()) {
			if (XmlLocalName(pattern.name()) != "pattern")
				continue;

			for (auto rule : pattern.children()) {
				if (XmlLocalName(rule.name()) != "rule")
					continue;

				std::string contextAttr = rule.attribute("context").as_string();
				if (contextAttr.empty())
					continue;

				std::string xpathExpr = contextToAbsoluteXPath(contextAttr, schemaPrefix);
				if (xpathExpr.empty())
					continue;

				for (auto assertNode : rule.children()) {
					if (XmlLocalName(assertNode.name()) != "assert")
						continue;

					std::string_view roleStr = assertNode.attribute("role").as_string("warning");
					SCHRole role = (roleStr == "error") ? SCHRole::Error : SCHRole::Warning;
					std::string message = TrimAsciiWhitespace(assertNode.text().as_string());
					schema.rules.push_back({ xpathExpr, message, role });
				}
			}
		}

		schema.loaded = true;
		return true;
	}

}  // namespace hdt
