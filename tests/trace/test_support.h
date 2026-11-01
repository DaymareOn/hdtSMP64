#pragma once

#include <doctest/doctest.h>

#include "hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMeshBone.h"
#include "hdtTraceRecorder.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace repro
{
	inline const std::filesystem::path traceDirectory = "Data/SKSE/Plugins/hdtSMP64/trace";
	inline const std::filesystem::path requestFile = traceDirectory / "request.json";

	constexpr int kPollInterval = 30;
	constexpr float kStep = 1.0f / 60.0f;

	using BonePtr = std::shared_ptr<hdt::SkinnedMeshBone>;

	inline btSphereShape& sphere()
	{
		static btSphereShape shape(1.0f);
		return shape;
	}

	inline BonePtr makeBone(const char* a_name, float a_mass, const btTransform& a_transform = btTransform::getIdentity())
	{
		btVector3 inertia(0, 0, 0);
		if (a_mass > 0)
			sphere().calculateLocalInertia(a_mass, inertia);
		btRigidBody::btRigidBodyConstructionInfo info(a_mass, nullptr, &sphere(), inertia);
		info.m_startWorldTransform = a_transform;
		auto bone = std::make_shared<hdt::SkinnedMeshBone>(a_name, info);
		if (a_mass == 0)
			bone->m_rig.setCollisionFlags(bone->m_rig.getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		bone->m_rig.setGravity(btVector3(0, 0, -9.8f));
		return bone;
	}

	struct Scene
	{
		std::vector<BonePtr> bones;
		btAlignedObjectArray<btTypedConstraint*> constraints;
		btContactSolverInfo solverInfo;
		btVector3 gravity{ 0, 0, -9.8f };

		bool begin(hdt::TraceRecorder& a_recorder, const char* a_name, const char* a_filter, uint32_t a_steps)
		{
			hdt::TraceRequest request;
			request.name = a_name;
			request.filter = a_filter;
			request.steps = a_steps;
			return a_recorder.begin(request, bones, constraints, solverInfo, gravity);
		}
	};

	inline uint64_t stepRecordBytes(size_t a_bodies) { return 24 + a_bodies * (26 + 13) * sizeof(float); }

	inline void step(hdt::TraceRecorder& a_recorder)
	{
		a_recorder.beginFrame(btVector3(0, 0, 0));
		a_recorder.recordPre(kStep);
		a_recorder.recordPost();
	}

	inline std::filesystem::path headerPath(const char* a_name) { return traceDirectory / (std::string(a_name) + ".json"); }
	inline std::filesystem::path binPath(const char* a_name) { return traceDirectory / (std::string(a_name) + ".bin"); }

	inline rapidjson::Document readHeader(const char* a_name)
	{
		rapidjson::Document document;
		std::ifstream stream(headerPath(a_name));
		if (!stream) {
			document.SetNull();
			return document;
		}
		rapidjson::IStreamWrapper wrapper(stream);
		document.ParseStream(wrapper);
		return document;
	}

	inline std::vector<char> readBin(const char* a_name)
	{
		std::ifstream stream(binPath(a_name), std::ios::binary);
		return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
	}

	template <class T>
	T readAt(const std::vector<char>& a_bytes, size_t a_offset)
	{
		T value;
		std::memcpy(&value, a_bytes.data() + a_offset, sizeof(T));
		return value;
	}

	inline void writeRequest(const std::string& a_json)
	{
		std::filesystem::create_directories(traceDirectory);
		std::ofstream(requestFile, std::ios::trunc) << a_json;
	}

	inline bool pollOnce(hdt::TraceRecorder& a_recorder, hdt::TraceRequest& a_request)
	{
		bool found = false;
		for (int i = 0; i < kPollInterval; ++i)
			found = a_recorder.pollRequest(a_request) || found;
		return found;
	}
}
