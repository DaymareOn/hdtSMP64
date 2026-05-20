#include "hdtForceUpdateList.h"

static std::unordered_set<RE::BSFixedString, RE::BSCRC32_<RE::BSFixedString>> nodes = {
	"WeaponAxe",
	"WeaponMace",
	"WeaponSword",
	"WeaponDagger",
	"WeaponBack",
	"WeaponBow",
	"QUIVER",
	"WeaponAxeLeft",
	"WeaponMaceLeft",
	"WeaponSwordLeft",
	"WeaponDaggerLeft",
	"ShieldBack",
	"WeaponStaff",
	"WeaponStaffLeft"
};

static std::unordered_set<RE::BSFixedString, RE::BSCRC32_<RE::BSFixedString>> nodes_mov = {
	"MOV WeaponAxeDefault",
	"MOV WeaponAxeLeftDefault",
	"MOV WeaponAxeReverse",
	"MOV WeaponAxeLeftReverse",
	"MOV WeaponAxeOnBack",
	"MOV WeaponAxeLeftOnBack",
	"MOV WeaponMaceDefault",
	"MOV WeaponMaceLeftDefault",
	"MOV WeaponSwordDefault",
	"MOV WeaponSwordLeftDefault",
	"MOV WeaponSwordOnBack",
	"MOV WeaponSwordLeftOnBack",
	"MOV WeaponSwordSWP",
	"MOV WeaponSwordLeftSWP",
	"MOV WeaponSwordFSM",
	"MOV WeaponSwordLeftFSM",
	"MOV WeaponSwordLeftHip",
	"MOV WeaponSwordLeftLeftHip",
	"MOV WeaponSwordNMD",
	"MOV WeaponSwordLeftNMD",
	"MOV WeaponDaggerDefault",
	"MOV WeaponDaggerLeftDefault",
	"MOV WeaponDaggerBackHip",
	"MOV WeaponDaggerLeftBackHip",
	"MOV WeaponDaggerAnkle",
	"MOV WeaponDaggerLeftAnkle",
	"MOV WeaponBackDefault",
	"MOV WeaponBackSWP",
	"MOV WeaponBackFSM",
	"MOV WeaponBackAxeMaceDefault",
	"MOV WeaponBackAxeMaceSWP",
	"MOV WeaponBackAxeMaceFSM",
	"MOV WeaponStaffDefault",
	"MOV WeaponStaffLeftDefault",
	"MOV WeaponBowDefault",
	"MOV WeaponBowChesko",
	"MOV WeaponBowBetter",
	"MOV WeaponBowFSM",
	"MOV WeaponCrossbowDefault",
	"MOV WeaponCrossbowChesko",
	"MOV QUIVERDefault",
	"MOV QUIVERChesko",
	"MOV QUIVERLeftHipBolt",
	"MOV BOLTDefault",
	"MOV BOLTChesko",
	"MOV BOLTLeftHipBolt",
	"MOV BOLTABQ",
	"MOV ShieldBackDefault"
};

namespace hdt
{
	int GetForceUpdateTypeFromName(const RE::BSFixedString& a_node_name)
	{
		const std::string node_name = a_node_name.c_str();

		// MOV-prefixed nodes → type 2
		if (node_name.rfind("MOV", 0) == 0) {
			if (nodes_mov.find(node_name) != nodes_mov.end()) {
				return 2;
			}

			return 0;
		}

		// All other nodes → type 1
		if (nodes.find(node_name) != nodes.end()) {
			return 1;
		}

		return 0;
	}
}
