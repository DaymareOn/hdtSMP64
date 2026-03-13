#pragma once

#include "DynamicHDT.h"
#include <hdtSerialization.h>

extern bool g_hasPapyrusExtension;

namespace hdt
{
	namespace Override
	{

		//The formID of the armoraddon in ArmorAttachEvent cannot be acquired, which makes it impossible to check override by the formID upon attaching armoraddon.
		class OverrideManager : public Serializer<void>
		{
		public:
			~OverrideManager() {};

			//Override virtual methods inherited from Serializer
			uint32_t FormatVersion() override { return 1; };

			uint32_t StorageName() override { return 'APFW'; };

			std::stringstream Serialize() override;

			void Deserialize(std::stringstream&) override;
			//Inherit End

			static OverrideManager* GetSingleton();

			std::string queryOverrideData();

			bool registerOverride(uint32_t actor_formID, std::string old_file_path, std::string new_file_path);

			std::string checkOverride(uint32_t actor_formID, std::string old_file_path);

		protected:
			OverrideManager() = default;
			std::unordered_map<uint32_t, std::unordered_map<std::string, std::string>> m_ActorPhysicsFileSwapList;
		};
	}
}
