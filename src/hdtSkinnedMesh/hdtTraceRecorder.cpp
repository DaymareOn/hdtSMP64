#include "hdtTraceRecorder.h"
#include "hdtConeTwistConstraint.h"
#include "hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMeshBone.h"
#include "hdtStiffSpringConstraint.h"
#include <filesystem>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

namespace hdt
{
	namespace
	{
		const std::filesystem::path kTraceDirectory = "Data/SKSE/Plugins/hdtSMP64/trace";
		const std::filesystem::path kRequestFile = kTraceDirectory / "request.json";
		constexpr uint32_t kPollInterval = 30;
		constexpr uint32_t kFormatVersion = 2;
		constexpr uint32_t kMaxTraceMinutes = 10;
		constexpr uint32_t kStepsPerSecond = 60;
		constexpr uint32_t kMaxSteps = kMaxTraceMinutes * 60 * kStepsPerSecond;
		constexpr uint64_t kMaxTraceBytes = 2ull << 30;
		constexpr uint32_t kPreFloatsPerBody = 26;
		constexpr uint32_t kPostFloatsPerBody = 13;
		constexpr uint32_t kStepHeaderBytes = 4 + 4 + 12 + 4;
		constexpr uint32_t kStepFlagWind = 1;

		using Writer = rapidjson::PrettyWriter<rapidjson::StringBuffer>;

		void writeVector(Writer& a_writer, const btVector3& a_value)
		{
			a_writer.StartArray();
			a_writer.Double(a_value.x());
			a_writer.Double(a_value.y());
			a_writer.Double(a_value.z());
			a_writer.EndArray();
		}

		void writeTransform(Writer& a_writer, const btTransform& a_value)
		{
			btQuaternion rotation;
			a_value.getBasis().getRotation(rotation);
			a_writer.StartObject();
			a_writer.Key("origin");
			writeVector(a_writer, a_value.getOrigin());
			a_writer.Key("rotation");
			a_writer.StartArray();
			a_writer.Double(rotation.x());
			a_writer.Double(rotation.y());
			a_writer.Double(rotation.z());
			a_writer.Double(rotation.w());
			a_writer.EndArray();
			a_writer.EndObject();
		}

		void writeBody(Writer& a_writer, const SkinnedMeshBone& a_bone, bool a_boundary)
		{
			const btRigidBody& body = a_bone.m_rig;
			const bool kinematic = body.isStaticOrKinematicObject();
			a_writer.StartObject();
			a_writer.Key("name");
			a_writer.String(a_bone.m_name.c_str());
			a_writer.Key("kinematic");
			a_writer.Bool(kinematic);
			a_writer.Key("boundary");
			a_writer.Bool(a_boundary);
			a_writer.Key("inverseMass");
			a_writer.Double(body.getInvMass());
			a_writer.Key("inverseInertiaLocal");
			writeVector(a_writer, body.getInvInertiaDiagLocal());
			a_writer.Key("linearDamping");
			a_writer.Double(body.getLinearDamping());
			a_writer.Key("angularDamping");
			a_writer.Double(body.getAngularDamping());
			if (!kinematic) {
				a_writer.Key("gravity");
				writeVector(a_writer, (body.getFlags() & BT_DISABLE_WORLD_GRAVITY) ? btVector3(0, 0, 0) : body.getGravity());
			}
			a_writer.EndObject();
		}

		void writeRotationalMotor(Writer& a_writer, const btRotationalLimitMotor2& a_motor)
		{
			a_writer.StartObject();
			a_writer.Key("lower");
			a_writer.Double(a_motor.m_loLimit);
			a_writer.Key("upper");
			a_writer.Double(a_motor.m_hiLimit);
			a_writer.Key("bounce");
			a_writer.Double(a_motor.m_bounce);
			a_writer.Key("stopERP");
			a_writer.Double(a_motor.m_stopERP);
			a_writer.Key("stopCFM");
			a_writer.Double(a_motor.m_stopCFM);
			a_writer.Key("springEnabled");
			a_writer.Bool(a_motor.m_enableSpring);
			a_writer.Key("springStiffness");
			a_writer.Double(a_motor.m_springStiffness);
			a_writer.Key("springStiffnessLimited");
			a_writer.Bool(a_motor.m_springStiffnessLimited);
			a_writer.Key("springDamping");
			a_writer.Double(a_motor.m_springDamping);
			a_writer.Key("springDampingLimited");
			a_writer.Bool(a_motor.m_springDampingLimited);
			a_writer.Key("equilibrium");
			a_writer.Double(a_motor.m_equilibriumPoint);
			a_writer.Key("nonHookeanStiffness");
			a_writer.Double(a_motor.m_nonHookeanStiffness);
			a_writer.Key("nonHookeanDamping");
			a_writer.Double(a_motor.m_nonHookeanDamping);
			a_writer.Key("motorEnabled");
			a_writer.Bool(a_motor.m_enableMotor);
			a_writer.Key("servoEnabled");
			a_writer.Bool(a_motor.m_servoMotor);
			a_writer.EndObject();
		}

		void writeGeneric6Dof(Writer& a_writer, Generic6DofConstraint& a_constraint)
		{
			a_writer.Key("type");
			a_writer.String("generic6dofSpring2");
			a_writer.Key("frameA");
			writeTransform(a_writer, a_constraint.getFrameOffsetA());
			a_writer.Key("frameB");
			writeTransform(a_writer, a_constraint.getFrameOffsetB());
			a_writer.Key("rotationOrder");
			a_writer.Int(static_cast<int>(a_constraint.getRotationOrder()));

			btTranslationalLimitMotor2* linear = a_constraint.getTranslationalLimitMotor();
			a_writer.Key("linear");
			a_writer.StartObject();
			a_writer.Key("lower");
			writeVector(a_writer, linear->m_lowerLimit);
			a_writer.Key("upper");
			writeVector(a_writer, linear->m_upperLimit);
			a_writer.Key("stopERP");
			writeVector(a_writer, linear->m_stopERP);
			a_writer.Key("stopCFM");
			writeVector(a_writer, linear->m_stopCFM);
			a_writer.Key("springEnabled");
			a_writer.StartArray();
			for (const bool enabled : linear->m_enableSpring)
				a_writer.Bool(enabled);
			a_writer.EndArray();
			a_writer.Key("springStiffness");
			writeVector(a_writer, linear->m_springStiffness);
			a_writer.Key("springDamping");
			writeVector(a_writer, linear->m_springDamping);
			a_writer.Key("equilibrium");
			writeVector(a_writer, linear->m_equilibriumPoint);
			a_writer.Key("nonHookeanStiffness");
			writeVector(a_writer, linear->m_nonHookeanStiffness);
			a_writer.Key("nonHookeanDamping");
			writeVector(a_writer, linear->m_nonHookeanDamping);
			a_writer.EndObject();

			a_writer.Key("angular");
			a_writer.StartArray();
			for (int axis = 0; axis < 3; ++axis)
				writeRotationalMotor(a_writer, *a_constraint.getRotationalLimitMotor(axis));
			a_writer.EndArray();
		}

		void writeConeTwist(Writer& a_writer, ConeTwistConstraint& a_constraint)
		{
			a_writer.Key("type");
			a_writer.String("coneTwist");
			a_writer.Key("frameA");
			writeTransform(a_writer, a_constraint.getFrameOffsetA());
			a_writer.Key("frameB");
			writeTransform(a_writer, a_constraint.getFrameOffsetB());
			a_writer.Key("swingSpan1");
			a_writer.Double(a_constraint.getSwingSpan1());
			a_writer.Key("swingSpan2");
			a_writer.Double(a_constraint.getSwingSpan2());
			a_writer.Key("twistSpan");
			a_writer.Double(a_constraint.getTwistSpan());
			a_writer.Key("limitSoftness");
			a_writer.Double(a_constraint.getLimitSoftness());
			a_writer.Key("biasFactor");
			a_writer.Double(a_constraint.getBiasFactor());
			a_writer.Key("relaxationFactor");
			a_writer.Double(a_constraint.getRelaxationFactor());
			a_writer.Key("damping");
			a_writer.Double(a_constraint.getDamping());
		}

		void writeStiffSpring(Writer& a_writer, const StiffSpringConstraint& a_constraint)
		{
			a_writer.Key("type");
			a_writer.String("stiffSpring");
			a_writer.Key("minDistance");
			a_writer.Double(a_constraint.m_minDistance);
			a_writer.Key("maxDistance");
			a_writer.Double(a_constraint.m_maxDistance);
			a_writer.Key("stiffness");
			a_writer.Double(a_constraint.m_stiffness);
			a_writer.Key("damping");
			a_writer.Double(a_constraint.m_damping);
			a_writer.Key("equilibrium");
			a_writer.Double(a_constraint.m_equilibriumPoint);
		}

		void writeSolverInfo(Writer& a_writer, const btContactSolverInfo& a_info)
		{
			a_writer.StartObject();
			a_writer.Key("iterations");
			a_writer.Int(a_info.m_numIterations);
			a_writer.Key("tau");
			a_writer.Double(a_info.m_tau);
			a_writer.Key("damping");
			a_writer.Double(a_info.m_damping);
			a_writer.Key("friction");
			a_writer.Double(a_info.m_friction);
			a_writer.Key("restitution");
			a_writer.Double(a_info.m_restitution);
			a_writer.Key("erp");
			a_writer.Double(a_info.m_erp);
			a_writer.Key("erp2");
			a_writer.Double(a_info.m_erp2);
			a_writer.Key("globalCfm");
			a_writer.Double(a_info.m_globalCfm);
			a_writer.Key("warmstartingFactor");
			a_writer.Double(a_info.m_warmstartingFactor);
			a_writer.Key("linearSlop");
			a_writer.Double(a_info.m_linearSlop);
			a_writer.Key("maxErrorReduction");
			a_writer.Double(a_info.m_maxErrorReduction);
			a_writer.Key("sor");
			a_writer.Double(a_info.m_sor);
			a_writer.Key("splitImpulse");
			a_writer.Bool(a_info.m_splitImpulse);
			a_writer.Key("splitImpulseTurnErp");
			a_writer.Double(a_info.m_splitImpulseTurnErp);
			a_writer.Key("solverMode");
			a_writer.Int(a_info.m_solverMode);
			a_writer.EndObject();
		}

		std::string sanitizedName(const std::string& a_name)
		{
			std::string result;
			for (const char c : a_name) {
				if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')
					result.push_back(c);
			}
			return result.empty() ? std::string("trace") : result;
		}


		void writeFloats(std::ofstream& a_out, const float* a_values, size_t a_count)
		{
			a_out.write(reinterpret_cast<const char*>(a_values), static_cast<std::streamsize>(a_count * sizeof(float)));
		}

		uint64_t stepRecordBytes(size_t a_bodyCount)
		{
			return kStepHeaderBytes + a_bodyCount * (kPreFloatsPerBody + kPostFloatsPerBody) * sizeof(float);
		}
	}

	bool TraceRecorder::pollRequest(TraceRequest& a_request)
	{
		if (++m_pollCounter < kPollInterval)
			return false;
		m_pollCounter = 0;

		std::error_code error;
		if (!std::filesystem::exists(kRequestFile, error))
			return false;

		std::ifstream stream(kRequestFile);
		rapidjson::IStreamWrapper wrapper(stream);
		rapidjson::Document document;
		document.ParseStream(wrapper);
		stream.close();
		if (document.HasParseError() || !document.IsObject()) {
			std::filesystem::remove(kRequestFile, error);
			logger::warn("[Trace] Ignoring unreadable trace request");
			return false;
		}

		const bool hasSteps = document.HasMember("steps") && document["steps"].IsUint();
		const bool stop = (document.HasMember("stop") && document["stop"].IsBool() && document["stop"].GetBool()) || (hasSteps && document["steps"].GetUint() == 0);
		if (m_active && !stop)
			return false;
		std::filesystem::remove(kRequestFile, error);

		a_request = {};
		a_request.stop = stop;
		if (stop)
			return true;

		a_request.name = document.HasMember("name") && document["name"].IsString() ? sanitizedName(document["name"].GetString()) : "trace";
		if (document.HasMember("filter") && document["filter"].IsString())
			a_request.filter = document["filter"].GetString();
		if (!hasSteps) {
			logger::warn("[Trace] Ignoring trace request '{}': 'steps' is missing or not a positive integer", a_request.name);
			return false;
		}
		a_request.steps = document["steps"].GetUint();
		if (a_request.steps > kMaxSteps) {
			logger::warn("[Trace] Trace request '{}' asks for {} steps; recording {}", a_request.name, a_request.steps, kMaxSteps);
			a_request.steps = kMaxSteps;
		}
		return true;
	}

	void TraceRecorder::setWorldState(bool a_windEnabled, const btVector3& a_windSpeed, float a_windTime)
	{
		m_windEnabled = a_windEnabled;
		m_windSpeed = a_windSpeed;
		m_windTime = a_windTime;
	}

	bool TraceRecorder::begin(const TraceRequest& a_request, std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> a_bodies, const btAlignedObjectArray<btTypedConstraint*>& a_constraints, const btContactSolverInfo& a_solverInfo, const btVector3& a_gravity)
	{
		reset();
		m_request = a_request;

		std::unordered_map<const btRigidBody*, size_t> allBodies;
		for (size_t i = 0; i < a_bodies.size(); ++i)
			allBodies[&a_bodies[i]->m_rig] = i;

		std::vector<bool> selected(a_bodies.size(), false);
		bool any = false;
		for (size_t i = 0; i < a_bodies.size(); ++i) {
			if (a_request.filter.empty() || std::string_view(a_bodies[i]->m_name.c_str()).find(a_request.filter) != std::string_view::npos) {
				selected[i] = true;
				any = true;
			}
		}
		if (!any) {
			logger::warn("[Trace] No bones match filter '{}'", a_request.filter);
			return false;
		}

		std::vector<bool> boundary(a_bodies.size(), false);
		std::vector<btTypedConstraint*> constraints;
		for (int i = 0; i < a_constraints.size(); ++i) {
			btTypedConstraint* constraint = a_constraints[i];
			const auto a = allBodies.find(&constraint->getRigidBodyA());
			const auto b = allBodies.find(&constraint->getRigidBodyB());
			if (a == allBodies.end() || b == allBodies.end()) {
				logger::warn("[Trace] Skipping a constraint that joins a body no bone owns");
				continue;
			}
			if (!selected[a->second] && !selected[b->second])
				continue;
			if (!selected[a->second])
				boundary[a->second] = true;
			if (!selected[b->second])
				boundary[b->second] = true;
			constraints.push_back(constraint);
		}

		for (size_t i = 0; i < a_bodies.size(); ++i) {
			if (selected[i]) {
				m_bodyIndex[&a_bodies[i]->m_rig] = static_cast<int>(m_bones.size());
				m_bones.push_back(a_bodies[i]);
				m_boundary.push_back(false);
			}
		}
		for (size_t i = 0; i < a_bodies.size(); ++i) {
			if (boundary[i]) {
				m_bodyIndex[&a_bodies[i]->m_rig] = static_cast<int>(m_bones.size());
				m_bones.push_back(a_bodies[i]);
				m_boundary.push_back(true);
			}
		}

		const uint64_t projectedBytes = stepRecordBytes(m_bones.size()) * m_request.steps;
		if (projectedBytes > kMaxTraceBytes) {
			logger::warn("[Trace] Refusing '{}': {} steps of {} bodies would take {} MiB, over the {} MiB limit", m_request.name, m_request.steps, m_bones.size(), projectedBytes >> 20, kMaxTraceBytes >> 20);
			reset();
			return false;
		}

		m_solverInfo = a_solverInfo;
		m_gravity = a_gravity;
		m_startWind = { m_windEnabled, m_windSpeed, m_windTime };

		snapshot(constraints);

		std::error_code error;
		std::filesystem::create_directories(kTraceDirectory, error);
		const auto binPath = kTraceDirectory / (m_request.name + ".bin");
		m_bin.open(binPath, std::ios::binary | std::ios::trunc);
		if (!m_bin) {
			logger::warn("[Trace] Cannot open {}", binPath.string());
			reset();
			return false;
		}

		m_stepsRecorded = 0;
		m_ended = "running";
		if (!writeHeader()) {
			reset();
			return false;
		}
		m_active = true;
		logger::info("[Trace] Recording {} steps of {} bodies and {} constraints to {}", m_request.steps, m_bones.size(), m_constraintCount, m_request.name);
		return true;
	}

	void TraceRecorder::snapshot(const std::vector<btTypedConstraint*>& a_constraints)
	{
		for (size_t i = 0; i < m_bones.size(); ++i)
			m_bodiesJson.push_back(describeBody(*m_bones[i], m_boundary[i]));
		for (btTypedConstraint* constraint : a_constraints)
			m_constraintsJson.push_back(describeConstraint(*constraint));
		m_constraintCount = a_constraints.size();
	}

	std::string TraceRecorder::describeBody(const SkinnedMeshBone& a_bone, bool a_boundary) const
	{
		rapidjson::StringBuffer buffer;
		Writer writer(buffer);
		writeBody(writer, a_bone, a_boundary);
		return buffer.GetString();
	}

	std::string TraceRecorder::describeConstraint(btTypedConstraint& a_constraint) const
	{
		rapidjson::StringBuffer buffer;
		Writer writer(buffer);
		writer.StartObject();
		writer.Key("bodyA");
		writer.Int(m_bodyIndex.at(&a_constraint.getRigidBodyA()));
		writer.Key("bodyB");
		writer.Int(m_bodyIndex.at(&a_constraint.getRigidBodyB()));
		writer.Key("enabled");
		writer.Bool(a_constraint.isEnabled());
		if (auto* generic = dynamic_cast<Generic6DofConstraint*>(&a_constraint))
			writeGeneric6Dof(writer, *generic);
		else if (auto* cone = dynamic_cast<ConeTwistConstraint*>(&a_constraint))
			writeConeTwist(writer, *cone);
		else if (auto* spring = dynamic_cast<StiffSpringConstraint*>(&a_constraint))
			writeStiffSpring(writer, *spring);
		else {
			writer.Key("type");
			writer.String("unknown");
		}
		writer.EndObject();
		return buffer.GetString();
	}

	void TraceRecorder::recordPre(float a_timeStep)
	{
		if (!m_active)
			return;

		const uint32_t flags = (m_windEnabled && !btFuzzyZero(m_windSpeed.length())) ? kStepFlagWind : 0;
		m_bin.write(reinterpret_cast<const char*>(&m_frame), sizeof(m_frame));
		writeFloats(m_bin, &a_timeStep, 1);
		const float offset[3] = { m_frameOffset.x(), m_frameOffset.y(), m_frameOffset.z() };
		writeFloats(m_bin, offset, 3);
		m_bin.write(reinterpret_cast<const char*>(&flags), sizeof(flags));

		for (const auto& bone : m_bones) {
			const btRigidBody& body = bone->m_rig;
			const btVector3& position = body.getWorldTransform().getOrigin();
			btQuaternion rotation;
			body.getWorldTransform().getBasis().getRotation(rotation);
			const btVector3& linear = body.getLinearVelocity();
			const btVector3& angular = body.getAngularVelocity();
			const btVector3& interpolationLinear = body.getInterpolationLinearVelocity();
			const btVector3& interpolationAngular = body.getInterpolationAngularVelocity();
			const btVector3& force = body.getTotalForce();
			const btVector3& torque = body.getTotalTorque();
			const float values[kPreFloatsPerBody] = {
				position.x(), position.y(), position.z(),
				rotation.x(), rotation.y(), rotation.z(), rotation.w(),
				linear.x(), linear.y(), linear.z(),
				angular.x(), angular.y(), angular.z(),
				interpolationLinear.x(), interpolationLinear.y(), interpolationLinear.z(),
				interpolationAngular.x(), interpolationAngular.y(), interpolationAngular.z(),
				force.x(), force.y(), force.z(),
				torque.x(), torque.y(), torque.z(),
				body.isStaticOrKinematicObject() ? 1.0f : 0.0f
			};
			writeFloats(m_bin, values, kPreFloatsPerBody);
		}
	}

	void TraceRecorder::recordPost()
	{
		if (!m_active)
			return;

		for (const auto& bone : m_bones) {
			const btRigidBody& body = bone->m_rig;
			const btVector3& position = body.getWorldTransform().getOrigin();
			btQuaternion rotation;
			body.getWorldTransform().getBasis().getRotation(rotation);
			const btVector3& linear = body.getLinearVelocity();
			const btVector3& angular = body.getAngularVelocity();
			const float values[kPostFloatsPerBody] = {
				position.x(), position.y(), position.z(),
				rotation.x(), rotation.y(), rotation.z(), rotation.w(),
				linear.x(), linear.y(), linear.z(),
				angular.x(), angular.y(), angular.z()
			};
			writeFloats(m_bin, values, kPostFloatsPerBody);
		}

		if (!m_bin) {
			logger::warn("[Trace] Write to {}.bin failed after {} steps", m_request.name, m_stepsRecorded);
			finish("write error");
			return;
		}
		if (++m_stepsRecorded >= m_request.steps)
			finish("complete");
	}

	void TraceRecorder::stop()
	{
		if (m_active)
			finish("stopped");
	}

	void TraceRecorder::finish(const char* a_reason)
	{
		m_bin.close();
		m_ended = a_reason;
		const std::string name = m_request.name;
		const uint32_t steps = m_stepsRecorded;
		if (writeHeader())
			logger::info("[Trace] Wrote {} steps to {} ({})", steps, name, a_reason);
		else
			logger::warn("[Trace] Recorded {} steps to {}.bin but could not finalize {}.json", steps, name, name);
		reset();
	}

	void TraceRecorder::reset()
	{
		if (m_bin.is_open())
			m_bin.close();
		m_active = false;
		m_bones.clear();
		m_boundary.clear();
		m_bodyIndex.clear();
		m_bodiesJson.clear();
		m_constraintsJson.clear();
		m_constraintCount = 0;
	}

	bool TraceRecorder::writeHeader() const
	{
		rapidjson::StringBuffer buffer;
		Writer writer(buffer);
		writer.StartObject();
		writer.Key("version");
		writer.Uint(kFormatVersion);
		writer.Key("steps");
		writer.Uint(m_request.steps);
		writer.Key("stepsRecorded");
		writer.Uint(m_stepsRecorded);
		writer.Key("ended");
		writer.String(m_ended);
		writer.Key("filter");
		writer.String(m_request.filter.c_str());
		writer.Key("worldGravity");
		writeVector(writer, m_gravity);
		writer.Key("wind");
		writer.StartObject();
		writer.Key("enabled");
		writer.Bool(m_startWind.enabled);
		writer.Key("speed");
		writeVector(writer, m_startWind.speed);
		writer.Key("time");
		writer.Double(m_startWind.time);
		writer.EndObject();
		writer.Key("solver");
		writeSolverInfo(writer, m_solverInfo);

		writer.Key("layout");
		writer.StartObject();
		writer.Key("stepHeader");
		writer.String("uint32 frame, float32 timeStep, float32[3] frameOffset, uint32 flags (bit 0: wind was applied this frame)");
		writer.Key("pre");
		writer.String("per body float32[26]: position(3) rotationXYZW(4) linearVelocity(3) angularVelocity(3) interpolationLinearVelocity(3) interpolationAngularVelocity(3) totalForce(3) totalTorque(3) kinematic(1.0 or 0.0)");
		writer.Key("post");
		writer.String("per body float32[13]: position(3) rotationXYZW(4) linearVelocity(3) angularVelocity(3)");
		writer.Key("stepOrder");
		writer.String("stepHeader, pre for every body, post for every body; repeated per step. The file may end inside a record: trust stepsRecorded, not the file size");
		writer.Key("forces");
		writer.String("totalForce and totalTorque are read before the step and already include gravity and wind, so a replay that applies them must not also apply the body's gravity");
		writer.Key("boundary");
		writer.String("a body marked boundary is outside the filter and is recorded only because a selected body's constraint joins it; replay it from its recorded pose");
		writer.Key("notRecorded");
		writer.String("collision contacts");
		writer.EndObject();

		writer.Key("bodies");
		writer.StartArray();
		for (const std::string& body : m_bodiesJson)
			writer.RawValue(body.c_str(), body.size(), rapidjson::kObjectType);
		writer.EndArray();

		writer.Key("constraints");
		writer.StartArray();
		for (const std::string& constraint : m_constraintsJson)
			writer.RawValue(constraint.c_str(), constraint.size(), rapidjson::kObjectType);
		writer.EndArray();
		writer.EndObject();

		const auto path = kTraceDirectory / (m_request.name + ".json");
		std::ofstream out(path, std::ios::trunc);
		out << buffer.GetString();
		out.flush();
		if (!out) {
			logger::warn("[Trace] Cannot write {}", path.string());
			return false;
		}
		return true;
	}
}
