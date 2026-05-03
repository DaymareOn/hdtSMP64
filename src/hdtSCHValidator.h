#pragma once

#include <string>
#include <vector>

namespace hdt
{
    struct SCHViolation
    {
        std::string xmlPath;
        std::string location;  // e.g. "/system[1]/bone[1]/linearDamping[1]"
        std::string message;
        std::string role;  // "error" or "warning"
    };

    struct SCHValidationResult
    {
        std::vector<SCHViolation> violations;
        bool hasErrors = false;
        bool hasWarnings = false;
    };

    // Validate an FSMP physics XML file against the Schematron rules in hdtSMP64.sch.
    // Rules are loaded once from disk at first call and cached for subsequent calls.
    SCHValidationResult ValidatePhysicsXMLWithSCH(const std::string& xmlPath);

}  // namespace hdt
