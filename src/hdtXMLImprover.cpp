#include "hdtXMLImprover.h"

#include <pugixml.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdt
{
	// ---- Schema-derived allowance tables ----
	// Derived from hdtSMP64.xsd.  Each entry maps a parent element name to the
	// set of child element names that the XSD permits inside it.
	// Elements that appear in this map but have an empty allowed-children set are
	// considered leaf nodes; any element child found inside them is invalid.
	// Elements that do NOT appear in this map at all are also leaf nodes (simple
	// types, or inline-defined elements with no element children).

	using ChildSet = std::unordered_set<std::string>;
	using ChildMap = std::unordered_map<std::string, ChildSet>;

	static const ChildMap& allowedChildren()
	{
		// clang-format off
		static const ChildMap map = {
			// ---- document root ----
			{"system", {
				"bone", "bone-default",
				"conetwist-constraint", "conetwist-constraint-default",
				"constraint-group",
				"generic-constraint", "generic-constraint-default",
				"per-triangle-shape", "per-vertex-shape", "shape",
				"stiffspring-constraint", "stiffspring-constraint-default",
			}},

			// ---- bone / bone-default ----
			{"bone", {
				"mass", "inertia", "centerOfMassTransform",
				"linearDamping", "angularDamping",
				"gravity-factor", "wind-factor",
				"friction", "rollingFriction", "restitution",
				"margin-multiplier", "shape", "collision-filter",
				"can-collide-with-bone", "no-collide-with-bone",
			}},
			{"bone-default", {
				"mass", "inertia", "centerOfMassTransform",
				"linearDamping", "angularDamping",
				"gravity-factor", "wind-factor",
				"friction", "rollingFriction", "restitution",
				"margin-multiplier", "shape", "collision-filter",
				"can-collide-with-bone", "no-collide-with-bone",
			}},

			// ---- compound shape child ----
			{"child", {"transform", "shape"}},

			// ---- conetwist constraints ----
			{"conetwist-constraint", {
				"frameInA", "frameInB", "frameInLerp", "angularOnly",
				"swingSpan1", "swingSpan2", "twistSpan",
				"coneLimit", "planeLimit", "twistLimit",
				"limitX", "limitY", "limitZ",
				"limitSoftness", "biasFactor", "relaxationFactor",
			}},
			{"conetwist-constraint-default", {
				"frameInA", "frameInB", "frameInLerp", "angularOnly",
				"swingSpan1", "swingSpan2", "twistSpan",
				"coneLimit", "planeLimit", "twistLimit",
				"limitX", "limitY", "limitZ",
				"limitSoftness", "biasFactor", "relaxationFactor",
			}},

			// ---- constraint-group ----
			{"constraint-group", {
				"generic-constraint", "generic-constraint-default",
				"stiffspring-constraint", "stiffspring-constraint-default",
				"conetwist-constraint", "conetwist-constraint-default",
			}},

			// ---- generic constraints ----
			{"generic-constraint", {
				"angularBounce", "angularEquilibrium",
				"angularLowerLimit", "angularUpperLimit", "angularStiffness",
				"angularSpringFrequency", "angularStiffnessLimited",
				"angularMotors", "angularServoMotors",
				"angularTargetVelocity", "angularMaxMotorForce",
				"angularNonHookeanDamping", "angularNonHookeanStiffness",
				"linearBounce", "linearEquilibrium",
				"linearLowerLimit", "linearUpperLimit", "linearStiffness",
				"linearSpringFrequency", "linearStiffnessLimited",
				"linearMotors", "linearServoMotors",
				"linearTargetVelocity", "linearMaxMotorForce",
				"linearNonHookeanDamping", "linearNonHookeanStiffness",
				"frameInA", "frameInB", "frameInLerp",
				"enableLinearSprings", "enableAngularSprings",
				"useLinearReferenceFrameA",
				"stopERP", "stopCFM", "motorERP", "motorCFM",
				"springDampingLimited",
				// vector3 versions of damping (distinct from the factor versions on bone)
				"linearDamping", "angularDamping",
			}},
			{"generic-constraint-default", {
				"angularBounce", "angularEquilibrium",
				"angularLowerLimit", "angularUpperLimit", "angularStiffness",
				"angularSpringFrequency", "angularStiffnessLimited",
				"angularMotors", "angularServoMotors",
				"angularTargetVelocity", "angularMaxMotorForce",
				"angularNonHookeanDamping", "angularNonHookeanStiffness",
				"linearBounce", "linearEquilibrium",
				"linearLowerLimit", "linearUpperLimit", "linearStiffness",
				"linearSpringFrequency", "linearStiffnessLimited",
				"linearMotors", "linearServoMotors",
				"linearTargetVelocity", "linearMaxMotorForce",
				"linearNonHookeanDamping", "linearNonHookeanStiffness",
				"frameInA", "frameInB", "frameInLerp",
				"enableLinearSprings", "enableAngularSprings",
				"useLinearReferenceFrameA",
				"stopERP", "stopCFM", "motorERP", "motorCFM",
				"springDampingLimited",
				"linearDamping", "angularDamping",
			}},

			// ---- per-triangle-shape ----
			{"per-triangle-shape", {
				"margin", "prenetration", "penetration",
				"tag", "shared",
				"no-collide-with-tag", "no-collide-with-bone",
				"can-collide-with-tag", "can-collide-with-bone",
				"weight-threshold", "disable-tag", "disable-priority",
			}},

			// ---- per-vertex-shape (no penetration/prenetration) ----
			{"per-vertex-shape", {
				"margin",
				"tag", "shared",
				"no-collide-with-tag", "can-collide-with-tag",
				"no-collide-with-bone", "can-collide-with-bone",
				"weight-threshold", "disable-tag", "disable-priority",
			}},

			// ---- shape ----
			{"shape", {"half-extend", "margin", "radius", "height", "point", "child"}},

			// ---- stiffspring constraints ----
			{"stiffspring-constraint", {
				"minDistanceFactor", "maxDistanceFactor",
				"stiffness", "damping", "equilibrium",
			}},
			{"stiffspring-constraint-default", {
				"minDistanceFactor", "maxDistanceFactor",
				"stiffness", "damping", "equilibrium",
			}},

			// ---- transform-typed elements (basis, basis-axis-angle, origin) ----
			{"centerOfMassTransform", {"basis", "basis-axis-angle", "origin"}},
			{"frameInA",             {"basis", "basis-axis-angle", "origin"}},
			{"frameInB",             {"basis", "basis-axis-angle", "origin"}},
			{"transform",            {"basis", "basis-axis-angle", "origin"}},

			// ---- lerp-typed element ----
			{"frameInLerp", {"translationLerp", "rotationLerp"}},
		};
		// clang-format on
		return map;
	}

	// All element names that appear anywhere in hdtSMP64.xsd.
	// An element whose name is NOT in this set is completely unknown and must
	// be removed regardless of its position in the document.
	static const ChildSet& knownElements()
	{
		// clang-format off
		static const ChildSet set = {
			// root
			"system",
			// complex elements
			"bone", "bone-default", "child", "constraint-group",
			"conetwist-constraint", "conetwist-constraint-default",
			"generic-constraint", "generic-constraint-default",
			"per-triangle-shape", "per-vertex-shape", "shape",
			"stiffspring-constraint", "stiffspring-constraint-default",
			// bone / bone-default properties
			"mass", "inertia", "centerOfMassTransform",
			"linearDamping", "angularDamping",
			"gravity-factor", "wind-factor",
			"friction", "rollingFriction", "restitution",
			"margin-multiplier", "collision-filter",
			"can-collide-with-bone", "no-collide-with-bone",
			"can-collide-with-tag", "no-collide-with-tag",
			// constraint frames
			"frameInA", "frameInB", "frameInLerp", "transform",
			// transform internals
			"basis", "basis-axis-angle", "origin",
			// lerp internals
			"translationLerp", "rotationLerp",
			// conetwist params
			"angularOnly",
			"swingSpan1", "swingSpan2", "twistSpan",
			"coneLimit", "planeLimit", "twistLimit",
			"limitX", "limitY", "limitZ",
			"limitSoftness", "biasFactor", "relaxationFactor",
			// generic-constraint params
			"angularBounce", "angularEquilibrium",
			"angularLowerLimit", "angularUpperLimit", "angularStiffness",
			"angularSpringFrequency", "angularStiffnessLimited",
			"angularMotors", "angularServoMotors",
			"angularTargetVelocity", "angularMaxMotorForce",
			"angularNonHookeanDamping", "angularNonHookeanStiffness",
			"linearBounce", "linearEquilibrium",
			"linearLowerLimit", "linearUpperLimit", "linearStiffness",
			"linearSpringFrequency", "linearStiffnessLimited",
			"linearMotors", "linearServoMotors",
			"linearTargetVelocity", "linearMaxMotorForce",
			"linearNonHookeanDamping", "linearNonHookeanStiffness",
			"enableLinearSprings", "enableAngularSprings",
			"useLinearReferenceFrameA",
			"stopERP", "stopCFM", "motorERP", "motorCFM",
			"springDampingLimited",
			// shape params
			"half-extend", "margin", "radius", "height", "point",
			// per-triangle / per-vertex params
			"prenetration", "penetration", "tag", "shared",
			"weight-threshold", "disable-tag", "disable-priority",
			// stiffspring params
			"minDistanceFactor", "maxDistanceFactor",
			"stiffness", "damping", "equilibrium",
		};
		// clang-format on
		return set;
	}

	// ---- DOM walker ----

	// Recursively removes child elements that are either:
	//   (a) not in knownElements() — completely unknown tag, or
	//   (b) not in the allowed children set for their parent.
	// Returns true if any element was removed at this level or below.
	static bool cleanNode(pugi::xml_node node, const std::string& parentName)
	{
		const auto& allowed = allowedChildren();
		const auto& known   = knownElements();

		std::vector<pugi::xml_node> toRemove;

		for (auto child = node.first_child(); child; child = child.next_sibling()) {
			if (child.type() != pugi::node_element)
				continue;

			std::string childName = child.name();

			// (a) completely unknown tag
			if (!known.count(childName)) {
				toRemove.push_back(child);
				continue;
			}

			// (b) not allowed in this parent
			if (!parentName.empty()) {
				auto it = allowed.find(parentName);
				if (it != allowed.end()) {
					// parent is a known container — child must be in its list
					if (!it->second.count(childName)) {
						toRemove.push_back(child);
						continue;
					}
				} else {
					// parent is not in the map → it is a leaf element; no children allowed
					toRemove.push_back(child);
					continue;
				}
			}

			// Valid child — recurse
			cleanNode(child, childName);
		}

		for (auto& n : toRemove)
			node.remove_child(n);

		return !toRemove.empty();
	}

	// ---- Path helpers ----

	// Strip the leading "data/" prefix (case-insensitive, forward slashes).
	static std::string stripDataPrefix(const std::string& path)
	{
		std::string norm = path;
		std::replace(norm.begin(), norm.end(), '\\', '/');

		std::string lower = norm;
		std::transform(lower.begin(), lower.end(), lower.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });

		if (lower.size() >= 5 && lower.substr(0, 5) == "data/")
			return norm.substr(5);
		return norm;
	}

	// ---- Public entry point ----

	bool GenerateImprovedXML(const std::string& srcXMLPath, const std::string& outputDir)
	{
		namespace fs = std::filesystem;

		pugi::xml_document doc;
		auto loadResult = doc.load_file(srcXMLPath.c_str());
		if (!loadResult)
			return false;

		// Locate the root <system> element
		pugi::xml_node sysNode;
		for (auto child = doc.first_child(); child; child = child.next_sibling()) {
			if (child.type() == pugi::node_element) {
				if (std::string(child.name()) != "system")
					return false; // unexpected root — skip
				sysNode = child;
				break;
			}
		}
		if (!sysNode)
			return false;

		// Clean the tree
		bool changed = cleanNode(sysNode, "system");
		if (!changed)
			return false;

		// Compute output path: <outputDir>/<relative-from-data>
		std::string relative = stripDataPrefix(srcXMLPath);
		fs::path outPath = fs::path(outputDir) / relative;

		std::error_code ec;
		fs::create_directories(outPath.parent_path(), ec);
		if (ec)
			return false;

		return doc.save_file(outPath.string().c_str(), "  ", pugi::format_default);
	}

} // namespace hdt
