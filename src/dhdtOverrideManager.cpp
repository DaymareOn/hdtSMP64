#include "dhdtOverrideManager.h"
#include <dhdtPapyrusFunctions.h>

using namespace hdt::Override;

bool g_hasPapyrusExtension = false;

OverrideManager* hdt::Override::OverrideManager::GetSingleton()
{
	static OverrideManager g_overrideManager;
	return &g_overrideManager;
}

bool checkPapyrusExtension()
{
	std::ofstream ifs("Data/Scripts/DynamicHDT.pex", std::ios::in | std::ios::_Nocreate);
	if (!ifs || !ifs.is_open()) {
		g_hasPapyrusExtension = false;
		return false;
	}
	ifs.close();
	g_hasPapyrusExtension = true;
	return true;
}

std::string hdt::Override::OverrideManager::queryOverrideData()
{
	std::string console_print("[DynamicHDT] -- Querying existing override data...\n");

	for (auto i : m_ActorPhysicsFileSwapList) {
		console_print += "Actor formID: " + util::UInt32toString(i.first) + "\t" + std::to_string(i.second.size()) + "\n";
		for (auto j : i.second) {
			console_print += "\tOriginal file: " + j.first + "\n\t\t| Override: " + j.second + "\n";
		}
	}

	console_print += "[DynamicHDT] -- Query finished...\n";
	return console_print;
}

bool hdt::Override::OverrideManager::registerOverride(uint32_t actor_formID, std::string old_file_path, std::string new_file_path)
{
	if (old_file_path.empty())
		return false;
	for (auto& e : m_ActorPhysicsFileSwapList[actor_formID]) {
		if (e.second == old_file_path) {
			old_file_path = e.first;
		}
	}
	m_ActorPhysicsFileSwapList[actor_formID][old_file_path] = new_file_path;
	return true;
}

std::string hdt::Override::OverrideManager::checkOverride(uint32_t actor_formID, std::string old_file_path)
{
	auto iter1 = m_ActorPhysicsFileSwapList.find(actor_formID);
	if (iter1 != m_ActorPhysicsFileSwapList.end()) {
		auto iter2 = iter1->second.find(old_file_path);
		if (iter2 != iter1->second.end()) {
			return iter2->second;
		}
	}
	return std::string();
}

std::stringstream hdt::Override::OverrideManager::Serialize()
{
	std::stringstream data_stream;
	if (!checkPapyrusExtension())
		return data_stream;

	for (const auto& [formID, overrides] : m_ActorPhysicsFileSwapList) {
		auto count = std::ranges::count_if(overrides, [](const auto& p) { return !p.second.empty(); });

		if (count == 0)
			continue;

		data_stream << std::format("{:08X} {}\n", formID, count);

		for (const auto& [orig, override_path] : overrides) {
			if (override_path.empty())
				continue;

			data_stream << orig << "\t" << override_path << std::endl;
		}
	}

	return data_stream;
}

void hdt::Override::OverrideManager::Deserialize(std::stringstream& data_stream)
{
	if (!checkPapyrusExtension())
		return;

	m_ActorPhysicsFileSwapList.clear();
	try {
		std::string line;
		while (std::getline(data_stream, line)) {
			if (line.empty())
				continue;

			std::istringstream header(line);
			uint32_t actor_formID = 0, override_size = 0;
			header >> std::hex >> actor_formID >> std::dec >> override_size;
			if (header.fail())
				return;

			for (auto i = 0u; i < override_size; ++i) {
				std::string pathLine;
				if (!std::getline(data_stream, pathLine))
					return;

				auto tabPos = pathLine.find('\t');
				if (tabPos == std::string::npos)
					return;

				hdt::papyrus::impl::SwapPhysicsFileImpl(actor_formID, pathLine.substr(0, tabPos), pathLine.substr(tabPos + 1), true, false);
			}
		}
	} catch (std::exception& e) {
		RE::ConsoleLog::GetSingleton()->Print("[DynamicHDT] ERROR! -- Failed parsing override data.");
		RE::ConsoleLog::GetSingleton()->Print("[DynamicHDT] Error(): {}\nWhat():\n\t{}", typeid(e).name(), e.what());
	}
}
