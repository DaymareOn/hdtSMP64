#include "hdtForceUpdateList.h"

hdt::ForceUpdateList* hdt::ForceUpdateList::GetSingleton()
{
	static ForceUpdateList g_forceUpdateNode;
	return &g_forceUpdateNode;
}

int hdt::ForceUpdateList::isAmong(const RE::BSFixedString& node_name)
{
	if (!node_name.contains("MOV")) {
		if (m_list.nodes.find(node_name) != m_list.nodes.end()) {
			return 1;
		}
	} else {
		if (m_list.nodes_mov.find(node_name) != m_list.nodes_mov.end()) {
			return 2;
		}
	}
	return 0;
}

hdt::ForceUpdateList::ForceUpdateList()
{
	m_list.nodes = {
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

	m_list.nodes_mov = {
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
}
