#pragma once

#include "hdtNIFImprover.h"

#include <string>
#include <utility>
#include <vector>

namespace hdt
{
	struct ParsedNif;

	struct DecimationBridgeDiagnostics
	{
		int candidatesDiscovered = 0;
		int candidatesAttempted = 0;
		int candidatesApplied = 0;
		int candidatesSkippedNoChange = 0;
		int candidatesSkippedUnsafe = 0;
		std::vector<std::pair<std::string, int>> skipReasons;
	};

	DecimationBridgeDiagnostics runOfflineCollisionDecimationBridge(
		ParsedNif& parsed,
		const NIFDecimationOptions& options,
		bool& changed);
}