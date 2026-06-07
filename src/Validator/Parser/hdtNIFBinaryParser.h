#pragma once

#include "../Improvers/hdtNIFBinaryIO.h"

#include <string>
#include <vector>

namespace hdt
{
	namespace nif
	{
		// Extracts all XML string references from NiStringExtraData blocks named with the
		// physics marker ("HDT Skinned Mesh Physics Object") in a fully-parsed NIF.
		// Reads directly from parsed.blocks — no raw-byte offset arithmetic needed.
		std::vector<std::string> FindXmlPathsInNif(const ParsedNif& parsed);

	}  // namespace nif

}  // namespace hdt
