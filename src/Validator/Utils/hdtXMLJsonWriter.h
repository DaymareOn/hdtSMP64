#pragma once

#include <pugixml.hpp>

#include <filesystem>

namespace hdt
{
	// Serialize an XML document into a structure-preserving JSON file.
	// Returns true when JSON was written successfully.
	bool WriteXmlDocumentAsJsonFile(const pugi::xml_document& doc, const std::filesystem::path& outPath);
}
