#pragma once

namespace hdt
{
	class ForceUpdateList
	{
		typedef struct
		{
			std::unordered_set<RE::BSFixedString> nodes;
			std::unordered_set<RE::BSFixedString> nodes_mov;
		} nodeList_t;

	public:
		static ForceUpdateList* GetSingleton();
		int isAmong(const RE::BSFixedString& node_name);

	private:
		ForceUpdateList();
		nodeList_t m_list;
	};
}
