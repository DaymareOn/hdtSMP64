#pragma once

#include "../Schema/hdtXSDSchemaModel.h"

#include <pugixml.hpp>

namespace hdt
{
	// Populate a PhysicsSchema from an already loaded XSD document.
	// Returns true when parsing completes without throwing.
	bool ParsePhysicsSchemaFromXSD(const pugi::xml_document& doc, PhysicsSchema& schema);

}  // namespace hdt
