#pragma once

namespace hdt
{
	class SerializerBase;

	extern SKSE::PluginHandle g_PluginHandle;

	extern std::vector<SerializerBase*> g_SerializerList;

	struct _uint32_to_str_t
	{
		union _uin32_cstr
		{
			uint32_t _number;
			char _buffer[4];
		};

		std::string operator()(uint32_t number)
		{
			_uin32_cstr _union{};
			_union._number = number;
			auto str = std::string(_union._buffer, 4);
			return std::string(str.rbegin(), str.rend());
		}
	};

	extern _uint32_to_str_t UInt32toStr;

	class SerializerBase
	{
	public:
		SerializerBase() {};
		~SerializerBase() {};
		virtual uint32_t StorageName() = 0;
		virtual uint32_t FormatVersion() = 0;

		virtual void SaveData(SKSE::SerializationInterface*) = 0;
		virtual void ReadData(SKSE::SerializationInterface*, uint32_t) = 0;

		static inline std::vector<SerializerBase*>& GetSerializerList() { return g_SerializerList; };

		static void Save(SKSE::SerializationInterface* intfc)
		{
			for (auto data_block : g_SerializerList) {
				// Console_Print("[HDT-SMP] Saving data, type: %s version: %08X", UInt32toStr(data_block->StorageName()).c_str(), data_block->FormatVersion());
				data_block->SaveData(intfc);
			}
		};

		static void Load(SKSE::SerializationInterface* intfc)
		{
			uint32_t type, version, length;
			//auto load_begin = clock();
			while (intfc->GetNextRecordInfo(type, version, length)) {
				auto record = std::find_if(g_SerializerList.begin(), g_SerializerList.end(), [type, version](SerializerBase* a_srlzr) {
					return type == a_srlzr->StorageName() && version == a_srlzr->FormatVersion();
				});

				if (record == g_SerializerList.end()) {
					continue;
				}

				//_MESSAGE("[HDT-SMP] Reading data, type: %s version: %08X length: %d", UInt32toStr(type).c_str(), version, length);
				(*record)->ReadData(intfc, length);
			}
			//Less than a microsecond
			//Console_Print("[HDT-SMP] Serializer loading cost: %.3f sec.", (clock() - load_begin) / 1000.0f);
		};
	};

	template <class _Storage_t = void, class _Stream_t = std::stringstream>
	class Serializer : public SerializerBase
	{
	public:
		Serializer()
		{
			g_SerializerList.push_back(this);
		};

		~Serializer() {};

		virtual _Stream_t Serialize() = 0;
		virtual _Storage_t Deserialize(_Stream_t&) = 0;

		void SaveData(SKSE::SerializationInterface*) override;
		void ReadData(SKSE::SerializationInterface*, uint32_t) override;

	protected:
		static inline std::string _toString(_Stream_t& _stream)
		{
			return _stream.rdbuf()->str();
		};
	};

	template <class _Storage_t, class _Stream_t>
	inline void Serializer<_Storage_t, _Stream_t>::SaveData(SKSE::SerializationInterface* intfc)
	{
		_Stream_t s_data_block = this->Serialize();
		if (intfc->OpenRecord(this->StorageName(), this->FormatVersion())) {
			intfc->WriteRecordData(_toString(s_data_block).c_str(), static_cast<uint32_t>(_toString(s_data_block).length()));
		}
		// Console_Print("Writing Data: \"%s\" \nStatus: %s", _toString(s_data_block).c_str(), success?"Succeeded":"Failed");
	}

	template <class _Storage_t, class _Stream_t>
	inline void Serializer<_Storage_t, _Stream_t>::ReadData(SKSE::SerializationInterface* intfc, uint32_t length)
	{
		std::string s_data;
		s_data.resize(length);

		intfc->ReadRecordData(s_data.data(), length);

		_Stream_t _stream;
		_stream << s_data;
		this->Deserialize(_stream);
	}
}
