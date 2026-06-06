#pragma once

#include <pugixml.hpp>

#include <cstddef>
#include <string>
#include <string_view>

namespace hdt
{
	// Strip an XML namespace prefix from a node/tag name and return the local name.
	inline std::string_view XmlLocalName(const char* name)
	{
		std::string_view sv(name ? name : "");
		auto colon = sv.rfind(':');
		return (colon != std::string_view::npos) ? sv.substr(colon + 1) : sv;
	}

	// Build a human-readable location path for a pugixml node.
	inline std::string BuildNodeLocationPath(const pugi::xml_node& node)
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

	// Convert a pugixml byte offset to a 1-based source line number.
	inline int OffsetToLineNumber(const std::string& src, ptrdiff_t offset)
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

	// Extract the local element name from a Schematron report location string.
	// e.g. "/system[1]/bone[2]/angularDamping[1]" -> "angularDamping"
	inline std::string ExtractElementNameFromSchLocation(const std::string& location)
	{
		auto slash = location.rfind('/');
		if (slash == std::string::npos)
			return {};

		auto start = slash + 1;
		auto bracket = location.find('[', start);
		if (bracket == std::string::npos)
			bracket = location.size();
		if (bracket <= start)
			return {};

		return location.substr(start, bracket - start);
	}

}  // namespace hdt
