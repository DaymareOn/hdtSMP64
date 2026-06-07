#pragma once

#include "../Schema/hdtSCHSchemaModel.h"

#include <pugixml.hpp>

namespace hdt
{
	// Populate a compiled Schematron schema from an already loaded .sch document.
	// Returns true on successful parse.
	bool ParseCompiledSchemaFromSCH(const pugi::xml_document& schDoc, CompiledSchema& schema);

}  // namespace hdt
