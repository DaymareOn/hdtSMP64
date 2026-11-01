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
		constexpr const char* kTraceDirectory = "Data/SKSE/Plugins/hdtSMP64/trace";
		constexpr const char* kRequestFile = "Data/SKSE/Plugins/hdtSMP64/trace/request.json";
		constexpr uint32_t kPollInterval = 30;
		constexpr uint32_t kFormatVersion = 1;
		constexpr uint32_t kPreFloatsPerBody = 19;
		constexpr uint32_t kPostFloatsPerBody = 13;

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

		void writeBody(Writer& a_writer, const SkinnedMeshBone& a_bone, const btVector3& a_gravity)
		{
			const btRigidBody& body = a_bone.m_rig;
			a_writer.StartObject();
			a_writer.Key("name");
			a_writer.String(a_bone.m_name.c_str());
			a_writer.Key("kinematic");
			a_writer.Bool(body.isStaticOrKinematicObject());
			a_writer.Key("inverseMass");
			a_writer.Double(body.getInvMass());
			a_writer.Key("inverseInertiaLocal");
			writeVector(a_writer, body.getInvInertiaDiagLocal());
			a_writer.Key("linearDamping");
			a_writer.Double(body.getLinearDamping());
			a_writer.Key("angularDamping");
			a_writer.Double(body.getAngularDamping());
			a_writer.Key("gravity");
			writeVector(a_writer, a_gravity);
			a_writer.Key("gravityFactor");
			a_writer.Double(a_bone.m_gravityFactor);
			a_writer.Key("centerOfMassOffset");
			writeTransform(a_writer, body.getCenterOfMassTransform().inverseTimes(body.getWorldTransform()));
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
	}

	bool TraceRecorder::pollRequest(TraceRequest& a_request)
	{
		if (m_active || ++m_pollCounter < kPollInterval)
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
		std::filesystem::remove(kRequestFile, error);
		if (document.HasParseError() || !document.IsObject()) {
			logger::warn("[Trace] Ignoring unreadable trace request");
			return false;
		}

		a_request = {};
		a_request.name = document.HasMember("name") && document["name"].IsString() ? sanitizedName(document["name"].GetString()) : "trace";
		if (document.HasMember("filter") && document["filter"].IsString())
			a_request.filter = document["filter"].GetString();
		if (document.HasMember("steps") && document["steps"].IsUint())
			a_request.steps = document["steps"].GetUint();
		return a_request.steps > 0;
	}

	bool TraceRecorder::begin(const TraceRequest& a_request, std::vector<RE::BSTSmartPointer<SkinnedMeshBone>> a_bodies, const btAlignedObjectArray<btTypedConstraint*>& a_constraints, const btContactSolverInfo& a_solverInfo, const btVector3& a_gravity)
	{
		m_request = a_request;
		m_bones.clear();
		for (auto& bone : a_bodies) {
			if (a_request.filter.empty() || std::string_view(bone->m_name.c_str()).find(a_request.filter) != std::string_view::npos)
				m_bones.push_back(bone);
		}
		if (m_bones.empty()) {
			logger::warn("[Trace] No bones match filter '{}'", a_request.filter);
			return false;
		}

		m_bodyIndex.clear();
		for (size_t i = 0; i < m_bones.size(); ++i)
			m_bodyIndex[&m_bones[i]->m_rig] = static_cast<int>(i);

		m_constraints.clear();
		for (int i = 0; i < a_constraints.size(); ++i) {
			btTypedConstraint* constraint = a_constraints[i];
			if (m_bodyIndex.count(&constraint->getRigidBodyA()) && m_bodyIndex.count(&constraint->getRigidBodyB()))
				m_constraints.push_back(constraint);
		}

		std::error_code error;
		std::filesystem::create_directories(kTraceDirectory, error);
		const auto binPath = std::filesystem::path(kTraceDirectory) / (m_request.name + ".bin");
		m_bin.open(binPath, std::ios::binary | std::ios::trunc);
		if (!m_bin) {
			logger::warn("[Trace] Cannot open {}", binPath.string());
			return false;
		}

		m_solverInfo = a_solverInfo;
		m_gravity = a_gravity;
		m_bodyGravity.assign(m_bones.size(), btVector3(0, 0, 0));
		m_stepsRecorded = 0;
		m_active = true;
		logger::info("[Trace] Recording {} steps of {} bodies and {} constraints to {}", m_request.steps, m_bones.size(), m_constraints.size(), m_request.name);
		return true;
	}

	void TraceRecorder::recordPre(float a_timeStep)
	{
		if (!m_active)
			return;

		m_bin.write(reinterpret_cast<const char*>(&m_frame), sizeof(m_frame));
		writeFloats(m_bin, &a_timeStep, 1);
		const float offset[3] = { m_frameOffset.x(), m_frameOffset.y(), m_frameOffset.z() };
		writeFloats(m_bin, offset, 3);

		for (size_t i = 0; i < m_bones.size(); ++i) {
			const btRigidBody& body = m_bones[i]->m_rig;
			if (m_stepsRecorded == 0)
				m_bodyGravity[i] = body.getGravity();
			const btVector3& position = body.getWorldTransform().getOrigin();
			btQuaternion rotation;
			body.getWorldTransform().getBasis().getRotation(rotation);
			const btVector3& linear = body.getLinearVelocity();
			const btVector3& angular = body.getAngularVelocity();
			const btVector3& interpolationLinear = body.getInterpolationLinearVelocity();
			const btVector3& interpolationAngular = body.getInterpolationAngularVelocity();
			const float values[kPreFloatsPerBody] = {
				position.x(), position.y(), position.z(),
				rotation.x(), rotation.y(), rotation.z(), rotation.w(),
				linear.x(), linear.y(), linear.z(),
				angular.x(), angular.y(), angular.z(),
				interpolationLinear.x(), interpolationLinear.y(), interpolationLinear.z(),
				interpolationAngular.x(), interpolationAngular.y(), interpolationAngular.z()
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

		if (++m_stepsRecorded >= m_request.steps)
			finish();
	}

	void TraceRecorder::finish()
	{
		m_bin.close();
		writeHeader();
		logger::info("[Trace] Wrote {} steps to {}", m_stepsRecorded, m_request.name);
		m_active = false;
		m_bones.clear();
		m_constraints.clear();
		m_bodyIndex.clear();
	}

	void TraceRecorder::writeHeader() const
	{
		rapidjson::StringBuffer buffer;
		Writer writer(buffer);
		writer.StartObject();
		writer.Key("version");
		writer.Uint(kFormatVersion);
		writer.Key("steps");
		writer.Uint(m_stepsRecorded);
		writer.Key("filter");
		writer.String(m_request.filter.c_str());
		writer.Key("worldGravity");
		writeVector(writer, m_gravity);
		writer.Key("solver");
		writeSolverInfo(writer, m_solverInfo);

		writer.Key("layout");
		writer.StartObject();
		writer.Key("stepHeader");
		writer.String("uint32 frame, float32 timeStep, float32[3] frameOffset");
		writer.Key("pre");
		writer.String("per body float32[19]: position(3) rotationXYZW(4) linearVelocity(3) angularVelocity(3) interpolationLinearVelocity(3) interpolationAngularVelocity(3)");
		writer.Key("post");
		writer.String("per body float32[13]: position(3) rotationXYZW(4) linearVelocity(3) angularVelocity(3)");
		writer.Key("stepOrder");
		writer.String("stepHeader, pre for every body, post for every body; repeated per step");
		writer.EndObject();

		writer.Key("bodies");
		writer.StartArray();
		for (size_t i = 0; i < m_bones.size(); ++i)
			writeBody(writer, *m_bones[i], m_bodyGravity[i]);
		writer.EndArray();

		writer.Key("constraints");
		writer.StartArray();
		for (btTypedConstraint* constraint : m_constraints) {
			writer.StartObject();
			writer.Key("bodyA");
			writer.Int(m_bodyIndex.at(&constraint->getRigidBodyA()));
			writer.Key("bodyB");
			writer.Int(m_bodyIndex.at(&constraint->getRigidBodyB()));
			writer.Key("enabled");
			writer.Bool(constraint->isEnabled());
			if (auto* generic = dynamic_cast<Generic6DofConstraint*>(constraint))
				writeGeneric6Dof(writer, *generic);
			else if (auto* cone = dynamic_cast<ConeTwistConstraint*>(constraint))
				writeConeTwist(writer, *cone);
			else if (auto* spring = dynamic_cast<StiffSpringConstraint*>(constraint))
				writeStiffSpring(writer, *spring);
			else {
				writer.Key("type");
				writer.String("unknown");
			}
			writer.EndObject();
		}
		writer.EndArray();
		writer.EndObject();

		std::ofstream out(std::filesystem::path(kTraceDirectory) / (m_request.name + ".json"));
		out << buffer.GetString();
	}
}
