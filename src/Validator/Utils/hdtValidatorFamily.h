#pragma once

#include <string>

namespace hdt
{
	// Constraint/element family — shared by the template analyser and the XML improver.
	enum class Family
	{
		None,
		Bone,
		PerTriangle,
		PerVertex,
		Generic,
		StiffSpring,
		ConeTwist
	};

	inline Family familyForNode(const std::string& localName)
	{
		if (localName == "bone" || localName == "bone-default")
			return Family::Bone;
		if (localName == "per-triangle-shape" || localName == "per-triangle-shape-default")
			return Family::PerTriangle;
		if (localName == "per-vertex-shape" || localName == "per-vertex-shape-default")
			return Family::PerVertex;
		if (localName == "generic-constraint" || localName == "generic-constraint-default")
			return Family::Generic;
		if (localName == "stiffspring-constraint" || localName == "stiffspring-constraint-default")
			return Family::StiffSpring;
		if (localName == "conetwist-constraint" || localName == "conetwist-constraint-default")
			return Family::ConeTwist;
		return Family::None;
	}

}  // namespace hdt
