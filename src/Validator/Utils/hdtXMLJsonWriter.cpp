#include "hdtXMLJsonWriter.h"

#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace hdt
{
		static bool isTextOnlyElement(const pugi::xml_node& node, std::string* outText)
		{
			if (node.type() != pugi::node_element)
				return false;
			if (node.first_attribute())
				return false;

			bool hasElementChild = false;
			std::string text;
			for (auto child = node.first_child(); child; child = child.next_sibling()) {
				if (child.type() == pugi::node_element) {
					hasElementChild = true;
					break;
				}
				if (child.type() == pugi::node_pcdata)
					text += child.value();
			}

			if (hasElementChild)
				return false;

			if (outText)
				*outText = text;
			return true;
		}

	static void jsonEscapeString(std::ostream& out, const char* s)
	{
		out << '"';
		for (; *s; ++s) {
			switch (*s) {
			case '"': out << "\\\""; break;
			case '\\': out << "\\\\"; break;
			case '\n': out << "\\n"; break;
			case '\r': out << "\\r"; break;
			case '\t': out << "\\t"; break;
			default: out << *s; break;
			}
		}
		out << '"';
	}

	// Returns true if the node has exactly x/y/z attributes and no child elements.
	// Such nodes are serialized as a JSON float array [x, y, z] instead of an object.
	static bool isXYZVectorNode(const pugi::xml_node& node)
	{
		auto a = node.first_attribute();
		if (!a || std::string(a.name()) != "x") return false;
		a = a.next_attribute();
		if (!a || std::string(a.name()) != "y") return false;
		a = a.next_attribute();
		if (!a || std::string(a.name()) != "z") return false;
		if (a.next_attribute()) return false;
		for (auto child = node.first_child(); child; child = child.next_sibling())
			if (child.type() == pugi::node_element) return false;
		return true;
	}

	static void xmlNodeToJsonStream(std::ostream& out, const pugi::xml_node& node, int indent)
	{
		std::string pad(indent * 2, ' ');
		std::string pad2((indent + 1) * 2, ' ');

		out << "{";
		bool needComma = false;

		for (auto attr = node.first_attribute(); attr; attr = attr.next_attribute()) {
			if (needComma)
				out << ",";
			out << "\n" << pad2;
			jsonEscapeString(out, attr.name());
			out << ": ";
			jsonEscapeString(out, attr.value());
			needComma = true;
		}

		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() == pugi::node_pcdata) {
				std::string text = child.value();
				if (!text.empty()) {
					if (needComma)
						out << ",";
					out << "\n" << pad2 << "\"#text\": ";
					jsonEscapeString(out, text.c_str());
					needComma = true;
				}
			}
		}

		std::vector<std::string> order;
		std::unordered_map<std::string, std::vector<pugi::xml_node>> childrenByName;
		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;
			std::string name = child.name();
			if (childrenByName.find(name) == childrenByName.end())
				order.push_back(name);
			childrenByName[name].push_back(child);
		}

		for (const auto& name : order) {
			const auto& arr = childrenByName[name];
			if (needComma)
				out << ",";
			out << "\n" << pad2;
			jsonEscapeString(out, name.c_str());
			out << ": ";
			if (arr.size() == 1) {
				std::string textValue;
				if (isTextOnlyElement(arr[0], &textValue))
					jsonEscapeString(out, textValue.c_str());
				else if (isXYZVectorNode(arr[0]))
					out << "[" << arr[0].attribute("x").value()
					    << ", " << arr[0].attribute("y").value()
					    << ", " << arr[0].attribute("z").value() << "]";
				else
					xmlNodeToJsonStream(out, arr[0], indent + 1);
			} else {
				out << "[";
				for (size_t i = 0; i < arr.size(); ++i) {
					if (i > 0)
						out << ",";
					out << "\n" << pad2 << "  ";
					std::string textValue;
					if (isTextOnlyElement(arr[i], &textValue))
						jsonEscapeString(out, textValue.c_str());
					else
						xmlNodeToJsonStream(out, arr[i], indent + 2);
				}
				out << "\n" << pad2 << "]";
			}
			needComma = true;
		}

		if (needComma)
			out << "\n" << pad;
		out << "}";
	}

	bool WriteXmlDocumentAsJsonFile(const pugi::xml_document& doc, const std::filesystem::path& outPath)
	{
		pugi::xml_node root;
		for (auto child = doc.first_child(); child; child = child.next_sibling()) {
			if (child.type() == pugi::node_element) {
				root = child;
				break;
			}
		}

		if (root.type() != pugi::node_element)
			return false;

		std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
		if (!out.is_open())
			return false;

		out << "{\n  ";
		jsonEscapeString(out, root.name());
		out << ": ";
		xmlNodeToJsonStream(out, root, 1);
		out << "\n}\n";

		return out.good();
	}
}
