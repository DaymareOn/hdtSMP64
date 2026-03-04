#pragma once

namespace hdt
{
	static inline void setNiNodeName(RE::NiNode* node, const char* name)
	{
		node->name = name;
	}

	static inline RE::NiNode* castNiNode(RE::NiAVObject* obj) 
	{ 
		return obj ? obj->AsNode() : nullptr; 
	}

	inline RE::BSTriShape* castBSTriShape(RE::NiAVObject* obj) 
	{
		return obj ? obj->AsTriShape() : nullptr; 
	}

	static inline RE::BSDynamicTriShape* castBSDynamicTriShape(RE::NiAVObject* obj)
	{
		return obj ? obj->AsDynamicTriShape() : nullptr; 
	}

	static inline RE::NiAVObject* findObject(RE::NiAVObject* obj, const RE::BSFixedString& name)
	{
		return obj->GetObjectByName(name);
	}

	static inline RE::NiNode* findNode(RE::NiNode* obj, const RE::BSFixedString& name)
	{
		auto ret = obj->GetObjectByName(name);
		return ret ? ret->AsNode() : nullptr;
	}

	static inline std::string readAllFile(const char* path)
	{
		RE::BSResourceNiBinaryStream stream(path);
		if (!stream.good())
		{
			return "";
		}

		//
		std::string file = "";

		//
		size_t required = stream.stream->totalSize;
		
		//
		file.resize(required);

		//
		stream.read((char*)file.data(), (uint32_t)required);

		//
		return file;
	}

	static inline std::string readAllFile2(const char* path)
	{
		std::ifstream stream(path, std::ios::binary);
		if (!stream.is_open())
		{
			return "";
		}

		stream.seekg(0, std::ios::end);
		auto size = stream.tellg();
		stream.seekg(0, std::ios::beg);
		std::string ret;
		ret.resize(size);
		stream.read(&ret[0], size);
		return ret;
	}

	static inline void updateTransformUpDown(RE::NiAVObject* obj, bool dirty)
	{
		if (!obj)
		{
			return;
		}

		RE::NiUpdateData ctx =
		{
			0.f,
			dirty ? RE::NiUpdateData::Flag::kDirty : RE::NiUpdateData::Flag::kNone
		};

		//
		obj->UpdateWorldData(&ctx);

		//
		RE::NiNode* node = castNiNode(obj);
		if (node)
		{
			for (auto& child : node->GetChildren())
			{
				if (child)
				{
					updateTransformUpDown(child.get(), dirty);
				}
			}
		}
	}
}
