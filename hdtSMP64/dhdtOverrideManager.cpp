#include "dhdtOverrideManager.h"

using namespace hdt::Override;

bool g_hasPapyrusExtension = false;

OverrideManager* hdt::Override::OverrideManager::GetSingleton() 
{
	static OverrideManager g_overrideManager;
	return &g_overrideManager;
}

bool checkPapyrusExtension() {
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

bool hdt::Override::OverrideManager::registerOverride(UInt32 actor_formID, std::string old_file_path, std::string new_file_path)
{
	if (old_file_path.empty())return false;
	for (auto& e : m_ActorPhysicsFileSwapList[actor_formID]) {
		if (e.second == old_file_path) {
			old_file_path = e.first;
		}
	}
	m_ActorPhysicsFileSwapList[actor_formID][old_file_path] = new_file_path;
	return true;
}

std::string hdt::Override::OverrideManager::checkOverride(UInt32 actor_formID, std::string old_file_path)
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
	if (!checkPapyrusExtension())return data_stream;

	for (auto& e : m_ActorPhysicsFileSwapList) {
		char buff[16];
		sprintf_s(buff, "%08X", e.first);
		data_stream << std::hex << buff << " " << std::dec << e.second.size() << std::endl;
		for (auto& e1 : e.second) {
			if (e1.second.empty())continue;
			data_stream << e1.first << "\t" << e1.second << std::endl;
		}
	}
	
	return data_stream;
}

void hdt::Override::OverrideManager::Deserialize(std::stringstream& data_stream)
{
	if (!checkPapyrusExtension())return;
	m_ActorPhysicsFileSwapList.clear();
	try {
		while (!data_stream.eof()) {
			UInt32 actor_formID, override_size = 0;
			data_stream >> std::hex >> actor_formID >> override_size;
			for (int i = 0; i < override_size; ++i) {
				std::string orig_physics_file, override_physics_file;
				data_stream >> orig_physics_file >> override_physics_file;
				//this->registerOverride(actor_formID, orig_physics_file, override_physics_file);
				ActorManager::instance()->swapPhysicsFile(actor_formID, orig_physics_file, override_physics_file);
			}
		}
	}
	catch (std::exception& e) {

		Console_Print("[DynamicHDT] ERROR! -- Failed parsing override data.");

		Console_Print("[DynamicHDT] Error(): %s\nWhat():\n\t%s", typeid(e).name(), e.what());

		return;
	}
}
