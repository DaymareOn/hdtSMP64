#pragma once

namespace hdt
{

	// Returns true if obj looks like a valid NiObject safe to call virtuals on.
	// VR's NiStream can leave bones[] slots as null or as raw char* pointers for
	// unresolved bone references. A char* used as a pointer has its first 8 bytes
	// interpreted as a vtable — those bytes are ASCII text, giving a non-canonical
	// address (> 0x7FFFFFFFFFFF) that faults on access. We also guard against null
	// vtable slots (stub objects from unknown NIF block types).
	// NiObject vtable layout: [0]=~NiRefObject, [1]=DeleteThis, [2]=GetRTTI, [3]=AsNode
	static constexpr uintptr_t kCanonicalUserSpaceMax = 0x00007FFFFFFFFFFFull;
	static inline bool isValidNiObject(const RE::NiAVObject* obj)
	{
		if (!obj)
			return false;
		if (reinterpret_cast<uintptr_t>(obj) > kCanonicalUserSpaceMax)
			return false;
		auto vtbl = *reinterpret_cast<void* const* const*>(obj);
		if (!vtbl || reinterpret_cast<uintptr_t>(vtbl) > kCanonicalUserSpaceMax)
			return false;
		return vtbl[3] != nullptr;  // slot 3 = AsNode
	}

	// Returns true if obj is a VR NiStream stub: passes isValidNiObject but has a
	// null or non-canonical function pointer at vtable slot 0x2B (GetObjectByName in VR,
	// offset 0x158). VR uses slot 0x2B because ApplyLocalTransformToWorld is inserted at
	// slot 26, shifting all SKYRIM_REL_VR_VIRTUAL entries by +1 relative to SE's 0x2A.
	// VR creates these stubs for SE-specific NIF block types it can't fully instantiate.
	static inline bool isVRNiStreamStub(const RE::NiAVObject* obj)
	{
		if (!isValidNiObject(obj))
			return false;
		auto vtbl = *reinterpret_cast<const uintptr_t* const*>(obj);
		auto slotGetObjectByName = vtbl[0x158 / sizeof(uintptr_t)];  // 0x158 = slot 0x2B (VR)
		return slotGetObjectByName == 0 || slotGetObjectByName > kCanonicalUserSpaceMax;
	}

	static inline void setNiNodeName(RE::NiNode* node, const char* name)
	{
		node->name = name;
	}

	static inline RE::NiNode* castNiNode(RE::NiAVObject* obj)
	{
		if (!isValidNiObject(obj)) {
			if (obj)
				logger::warn("castNiNode: skipping object at {:p} with invalid vtable (VR NiStream stub or unresolved bone ref)", static_cast<const void*>(obj));
			return nullptr;
		}
		return obj->AsNode();
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
		if (!stream.good()) {
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
		if (!stream.is_open()) {
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
		if (!obj) {
			return;
		}

		RE::NiUpdateData ctx = {
			0.f,
			dirty ? RE::NiUpdateData::Flag::kDirty : RE::NiUpdateData::Flag::kNone
		};

		//
		obj->UpdateWorldData(&ctx);

		//
		RE::NiNode* node = castNiNode(obj);
		if (node) {
			for (auto& child : node->GetChildren()) {
				if (child) {
					updateTransformUpDown(child.get(), dirty);
				}
			}
		}
	}

	static inline RE::NiPoint3 rotate(const RE::NiPoint3& v, const RE::NiPoint3& axis, float theta)
	{
		const float cosTheta = std::cos(theta);
		return (v * cosTheta) + (axis.Cross(v) * std::sin(theta)) + (axis * axis.Dot(v)) * (1.0f - cosTheta);
	}
}
