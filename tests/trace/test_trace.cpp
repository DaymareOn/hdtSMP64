
#include "test_support.h"

#include <cmath>
#include <new>

namespace
{
	const btTransform identity = btTransform::getIdentity();
	constexpr uint32_t kMaxSteps = 36000;

	void clearRequest()
	{
		std::error_code error;
		std::filesystem::remove(repro::requestFile, error);
	}
}

TEST_CASE("request: a missing steps field is refused and the request file is consumed")
{
	using namespace repro;
	hdt::TraceRecorder recorder;
	hdt::TraceRequest request;
	writeRequest(R"({"name":"t_nosteps"})");
	CHECK_FALSE(pollOnce(recorder, request));
	CHECK_FALSE(std::filesystem::exists(requestFile));
}

TEST_CASE("request: steps is capped")
{
	using namespace repro;
	hdt::TraceRecorder recorder;
	hdt::TraceRequest request;
	writeRequest(R"({"name":"t_huge","steps":4294967295})");
	REQUIRE(pollOnce(recorder, request));
	CHECK_FALSE(request.stop);
	CHECK(request.steps == kMaxSteps);
}

TEST_CASE("request: steps 0 and stop true are stop requests")
{
	using namespace repro;
	hdt::TraceRecorder recorder;
	hdt::TraceRequest request;
	writeRequest(R"({"steps":0})");
	REQUIRE(pollOnce(recorder, request));
	CHECK(request.stop);
	writeRequest(R"({"stop":true})");
	REQUIRE(pollOnce(recorder, request));
	CHECK(request.stop);
}

TEST_CASE("request: a start request waits while a trace runs and a stop request ends it with a valid header")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("A", 1.0f) };
	hdt::TraceRecorder recorder;
	REQUIRE(scene.begin(recorder, "t_stop", "", 1000));
	step(recorder);
	step(recorder);

	hdt::TraceRequest request;
	writeRequest(R"({"name":"t_other","steps":1})");
	CHECK_FALSE(pollOnce(recorder, request));
	CHECK(recorder.active());
	CHECK(std::filesystem::exists(requestFile));

	writeRequest(R"({"stop":true})");
	REQUIRE(pollOnce(recorder, request));
	REQUIRE(request.stop);
	recorder.stop();
	CHECK_FALSE(recorder.active());

	const auto header = readHeader("t_stop");
	REQUIRE(header.IsObject());
	CHECK(header["steps"].GetUint() == 1000);
	CHECK(header["stepsRecorded"].GetUint() == 2);
	CHECK(std::string(header["ended"].GetString()) == "stopped");
	clearRequest();
}

TEST_CASE("begin: a trace projected over the size limit is refused")
{
	using namespace repro;
	Scene scene;
	for (int i = 0; i < 500; ++i)
		scene.bones.push_back(makeBone("B", 1.0f));
	hdt::TraceRecorder recorder;
	CHECK_FALSE(scene.begin(recorder, "t_big", "", kMaxSteps));
	CHECK_FALSE(recorder.active());
	CHECK_FALSE(std::filesystem::exists(binPath("t_big")));
}

TEST_CASE("record size: a step is the header plus 26 pre and 13 post floats per body")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("A", 1.0f), makeBone("B", 1.0f) };
	hdt::TraceRecorder recorder;
	REQUIRE(scene.begin(recorder, "t_size", "", 3));
	for (int i = 0; i < 3; ++i)
		step(recorder);
	CHECK(std::filesystem::file_size(binPath("t_size")) == 3 * stepRecordBytes(2));
}

TEST_CASE("header: written when the trace starts, so an interrupted trace can be decoded")
{
	using namespace repro;
	{
		Scene scene;
		scene.bones = { makeBone("A", 1.0f) };
		hdt::TraceRecorder recorder;
		REQUIRE(scene.begin(recorder, "t_interrupted", "", 5));
		step(recorder);
		step(recorder);
	}
	CHECK(std::filesystem::file_size(binPath("t_interrupted")) == 2 * stepRecordBytes(1));
	const auto header = readHeader("t_interrupted");
	REQUIRE(header.IsObject());
	CHECK(header["version"].GetUint() == 2);
	CHECK(header["steps"].GetUint() == 5);
	CHECK(std::string(header["ended"].GetString()) == "running");
}

TEST_CASE("header: a finished trace reports what was recorded")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("A", 1.0f) };
	hdt::TraceRecorder recorder;
	REQUIRE(scene.begin(recorder, "t_done", "", 2));
	step(recorder);
	step(recorder);
	CHECK_FALSE(recorder.active());
	const auto header = readHeader("t_done");
	REQUIRE(header.IsObject());
	CHECK(header["stepsRecorded"].GetUint() == 2);
	CHECK(std::string(header["ended"].GetString()) == "complete");
}

TEST_CASE("constraints: a constraint destroyed while the trace runs is not read again")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("NPC L Breast", 1.0f), makeBone("NPC Spine2 [Spn2]", 0.0f) };
	alignas(hdt::Generic6DofConstraint) unsigned char storage[sizeof(hdt::Generic6DofConstraint)];
	auto* constraint = new (storage) hdt::Generic6DofConstraint(scene.bones[0]->m_rig, scene.bones[1]->m_rig, identity, identity);
	scene.constraints.push_back(constraint);

	hdt::TraceRecorder recorder;
	REQUIRE(scene.begin(recorder, "t_freed", "", 2));
	step(recorder);

	constraint->~Generic6DofConstraint();
	std::memset(storage, 0xDD, sizeof(storage));

	CHECK_NOTHROW(step(recorder));
	const auto header = readHeader("t_freed");
	REQUIRE(header.IsObject());
	REQUIRE(header["constraints"].Size() == 1);
	CHECK(std::string(header["constraints"][0]["type"].GetString()) == "generic6dofSpring2");
}

TEST_CASE("filter: the constraint holding a selected bone and its anchor are kept, the anchor marked boundary")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("NPC L Breast", 1.0f), makeBone("NPC Spine2 [Spn2]", 0.0f), makeBone("Unrelated", 1.0f) };
	hdt::Generic6DofConstraint anchor(scene.bones[0]->m_rig, scene.bones[1]->m_rig, identity, identity);
	scene.constraints.push_back(&anchor);

	hdt::TraceRecorder recorder;
	REQUIRE(scene.begin(recorder, "t_filter", "Breast", 1));
	step(recorder);

	const auto header = readHeader("t_filter");
	REQUIRE(header.IsObject());
	REQUIRE(header["bodies"].Size() == 2);
	CHECK(std::string(header["bodies"][0]["name"].GetString()) == "NPC L Breast");
	CHECK_FALSE(header["bodies"][0]["boundary"].GetBool());
	CHECK(std::string(header["bodies"][1]["name"].GetString()) == "NPC Spine2 [Spn2]");
	CHECK(header["bodies"][1]["boundary"].GetBool());
	REQUIRE(header["constraints"].Size() == 1);
	CHECK(header["constraints"][0]["bodyA"].GetInt() == 0);
	CHECK(header["constraints"][0]["bodyB"].GetInt() == 1);
}

TEST_CASE("filter: no match refuses the trace")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("A", 1.0f) };
	hdt::TraceRecorder recorder;
	CHECK_FALSE(scene.begin(recorder, "t_nomatch", "zzz", 1));
}

TEST_CASE("bodies: gravity is the effective acceleration, kinematic bodies carry none and no field double-scales it")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("Dynamic", 1.0f), makeBone("Kinematic", 0.0f) };
	scene.bones[0]->m_rig.setGravity(btVector3(0, 0, -4.9f));
	hdt::TraceRecorder recorder;
	REQUIRE(scene.begin(recorder, "t_bodies", "", 1));
	step(recorder);

	const auto header = readHeader("t_bodies");
	REQUIRE(header.IsObject());
	const auto& dynamic = header["bodies"][0];
	const auto& kinematic = header["bodies"][1];
	CHECK(dynamic["gravity"][2].GetDouble() == doctest::Approx(-4.9));
	CHECK_FALSE(dynamic.HasMember("gravityFactor"));
	CHECK_FALSE(dynamic.HasMember("centerOfMassOffset"));
	CHECK(kinematic["kinematic"].GetBool());
	CHECK_FALSE(kinematic.HasMember("gravity"));
}

TEST_CASE("step record: forces, the kinematic flag and the wind flag are recorded")
{
	using namespace repro;
	Scene scene;
	scene.bones = { makeBone("Dynamic", 1.0f), makeBone("Kinematic", 0.0f) };
	scene.bones[0]->m_rig.applyCentralForce(btVector3(1, 2, 3));
	scene.bones[0]->m_rig.applyTorque(btVector3(4, 5, 6));

	hdt::TraceRecorder recorder;
	recorder.setWorldState(true, btVector3(0, 100, 0), 1.5f);
	REQUIRE(scene.begin(recorder, "t_record", "", 2));
	step(recorder);
	recorder.setWorldState(true, btVector3(0, 0, 0), 1.5f);
	step(recorder);

	const auto bin = readBin("t_record");
	REQUIRE(bin.size() == 2 * stepRecordBytes(2));
	CHECK(readAt<uint32_t>(bin, 20) == 1);
	const size_t body0 = 24;
	const size_t body1 = body0 + 26 * sizeof(float);
	CHECK(readAt<float>(bin, body0 + 19 * sizeof(float)) == doctest::Approx(1.0f));
	CHECK(readAt<float>(bin, body0 + 24 * sizeof(float)) == doctest::Approx(6.0f));
	CHECK(readAt<float>(bin, body0 + 25 * sizeof(float)) == doctest::Approx(0.0f));
	CHECK(readAt<float>(bin, body1 + 25 * sizeof(float)) == doctest::Approx(1.0f));
	CHECK(readAt<uint32_t>(bin, stepRecordBytes(2) + 20) == 0);

	const auto header = readHeader("t_record");
	REQUIRE(header.IsObject());
	CHECK(header["wind"]["enabled"].GetBool());
	CHECK(header["wind"]["speed"][1].GetDouble() == doctest::Approx(100.0));
	CHECK(header["wind"]["time"].GetDouble() == doctest::Approx(1.5));
}
