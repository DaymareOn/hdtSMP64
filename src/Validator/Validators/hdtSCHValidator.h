#pragma once

#include <string>
#include <vector>

namespace hdt
{
	enum class SCHRole
	{
		Warning,
		Error,
	};

	struct SCHViolation
	{
		std::string xmlPath;
		std::string location;  // e.g. "/system[1]/bone[1]/linearDamping[1]"
		std::string message;
		SCHRole role = SCHRole::Warning;
		int line = 0;  // 1-based source line number, 0 if unknown
	};

	struct SCHValidationResult
	{
		std::vector<SCHViolation> violations;
		bool hasErrors = false;
		bool hasWarnings = false;
	};

	// Validate an FSMP physics XML file against the Schematron rules in hdtSMP64.sch.
	// Rules are loaded once from disk at first call and cached for subsequent calls.
	SCHValidationResult ValidatePhysicsXMLWithSchematron(const std::string& xmlPath);

}  // namespace hdt
