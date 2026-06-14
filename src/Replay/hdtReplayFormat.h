#pragma once

// hdtReplayFormat.h - on-disk format for the SMP physics replay/benchmark harness.
//
// This header is shared verbatim by the in-game DLL (which writes captures) and the standalone
// smp_replay exe (which reads them), so it must not depend on CommonLibSSE, Bullet, or any other
// runtime type - only the C++ standard library. The single source of truth for the binary layout
// is the serialize()/deserialize() pair below (explicit little-endian field writes, never a memcpy
// of an aligned SIMD type). Keeping both sides on this one header is what guards against format
// drift (D5/D7), backstopped by the always-on parity test (D8).
//
// FROZEN after P0: changing any struct or the wire layout requires bumping kReplayFormatVersion
// and is a breaking change for every committed fixture.

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace hdt::replay
{
	// ---- versioned header constants (project "Magic values" rule: named, never inline literals) ----

	// File magic, written as the four ASCII bytes 'S','M','P','R'. Read back as a little-endian
	// uint32 this is 0x52504D53.
	inline constexpr char kReplayMagic[4] = { 'S', 'M', 'P', 'R' };

	// Bump on ANY change to the wire layout below. Load requires an exact match (fail closed).
	// v2: added Snapshot::skeletonId (the actor-group id, reserved for the active-state/disable-tag work).
	// v3: added Snapshot::buildTimeMicros (in-game XML+NIF build/read cost, measured at capture time).
	inline constexpr uint32_t kReplayFormatVersion = 3;

	// Fixed width of the embedded git sha string (zero-padded). 40 = full sha-1 hex.
	inline constexpr uint32_t kGitShaLength = 40;

	// Upper bound applied to EVERY count/length read from the file before it is trusted to size an
	// allocation. A capture file is a trust boundary (global input-validation rule): a corrupt or
	// hostile count must be rejected, never used to allocate. 64M elements is far above any real
	// capture yet small enough that count * sizeof(element) cannot overflow size_t on 64-bit.
	inline constexpr uint32_t kMaxCount = 64u * 1024u * 1024u;

	// ---- error type: any malformed input throws this; callers fail closed (non-zero exit) ----

	class ReplayFormatError : public std::runtime_error
	{
	public:
		explicit ReplayFormatError(const std::string& what) :
			std::runtime_error(what) {}
	};

	// ---- math PODs (mirror the Bullet types at the btQsTransform seam, §4) ----

	struct Quat
	{
		float x = 0, y = 0, z = 0, w = 1;
		bool operator==(const Quat&) const = default;
	};

	struct Vec3
	{
		float x = 0, y = 0, z = 0;
		bool operator==(const Vec3&) const = default;
	};

	struct Vec4
	{
		float x = 0, y = 0, z = 0, w = 0;
		bool operator==(const Vec4&) const = default;
	};

	// btQsTransform = quat(4f) + origin(3f) + scale(1f) (§9).
	struct QsTransform
	{
		Quat basis;
		Vec3 origin;
		float scale = 1.0f;
		bool operator==(const QsTransform&) const = default;
	};

	// btTransform serialized as rotation quaternion + origin.
	struct Transform
	{
		Quat basis;
		Vec3 origin;
		bool operator==(const Transform&) const = default;
	};

	// ---- world / solver configuration (§5, hdtSkyrimPhysicsWorld.cpp:12-51) ----

	struct SolverConfig
	{
		Vec3 gravity{ 0, 0, -9.8f };
		float friction = 0.0f;
		uint8_t splitImpulse = 1;
		float splitImpulsePenetrationThreshold = -0.01f;
		float erp2 = 0.15f;
		float globalCfm = 0.001f;
		float restitutionVelocityThreshold = 0.2f;
		int32_t solverMode = 0;
		float leastSquaresResidualThreshold = 0.0001f;
		float timeTick = 1.0f / 60.0f;
		int32_t maxSubSteps = 4;
		// wind settings
		uint8_t enableWind = 1;
		float windStrength = 2.0f;
		float distanceForNoWind = 50.0f;
		float distanceForMaxWind = 3000.0f;
		// gDisableDeactivation
		uint8_t disableDeactivation = 1;
		bool operator==(const SolverConfig&) const = default;
	};

	// ---- collision shapes (the 6 readShape types, §5 / hdtSkyrimSystem.cpp:496-646) ----

	enum class ShapeType : uint32_t
	{
		Box = 0,
		Sphere = 1,
		Capsule = 2,
		ConvexHull = 3,
		Cylinder = 4,
		Compound = 5,
	};

	struct CompoundChild
	{
		Transform localTransform;
		uint32_t shapeIndex = 0;  // index into Snapshot::shapes
		bool operator==(const CompoundChild&) const = default;
	};

	struct CollisionShape
	{
		ShapeType type = ShapeType::Sphere;
		float margin = 0.0f;
		Vec3 localScaling{ 1, 1, 1 };
		// box / cylinder
		Vec3 halfExtents;
		// sphere / capsule / cylinder
		float radius = 0.0f;
		float height = 0.0f;
		// convex hull
		std::vector<Vec3> hullPoints;
		// compound
		std::vector<CompoundChild> children;
		bool operator==(const CollisionShape&) const = default;
	};

	// ---- bones (§5, hdtSkinnedMeshBone.h / hdtSkyrimSystem.h) ----

	struct Bone
	{
		std::string name;
		float mass = 0.0f;
		float invMass = 0.0f;
		Vec3 invInertiaDiagLocal;
		Transform localToRig;
		Transform rigToLocal;
		int32_t shapeIndex = -1;  // index into Snapshot::shapes, -1 = empty shape
		float marginMultiplier = 1.0f;
		float boundingSphereMultiplier = 1.0f;
		float gravityFactor = 1.0f;
		float windFactor = 1.0f;
		uint32_t collisionFilter = 0;
		uint8_t kinematic = 0;  // 1 == kinematic/driver bone (mass 0)
		QsTransform initialWorldTransform;
		std::vector<std::string> canCollideWithBone;
		std::vector<std::string> noCollideWithBone;
		bool operator==(const Bone&) const = default;
	};

	// ---- mesh bodies (§5, hdtSkinnedMeshBody.h / hdtVertex.h) ----

	struct BodyVertex
	{
		Vec4 skinPos;        // m_skinPos.xyz + margin in w
		float weight[4] = { 0, 0, 0, 0 };
		uint32_t boneIdx[4] = { 0, 0, 0, 0 };
		bool operator==(const BodyVertex& rhs) const
		{
			return skinPos == rhs.skinPos &&
			       std::memcmp(weight, rhs.weight, sizeof(weight)) == 0 &&
			       std::memcmp(boneIdx, rhs.boneIdx, sizeof(boneIdx)) == 0;
		}
	};

	struct SphereVolume
	{
		Vec3 center;
		float radius = 0.0f;
		bool operator==(const SphereVolume&) const = default;
	};

	// One bone bound into a body's skinning (SkinnedMeshBody::SkinnedBone).
	struct BodySkinnedBone
	{
		uint32_t boneIndex = 0;  // index into the owning Snapshot::bones
		QsTransform vertexToBone;
		SphereVolume localBoundingSphere;
		float weightThreshold = 0.0f;
		uint8_t isKinematic = 0;
		bool operator==(const BodySkinnedBone&) const = default;
	};

	enum class BodyShapeKind : uint32_t
	{
		PerVertex = 0,
		PerTriangle = 1,
	};

	struct MeshBody
	{
		std::string name;
		BodyShapeKind shapeKind = BodyShapeKind::PerVertex;
		float margin = 1.0f;
		float penetration = 0.0f;
		std::vector<BodyVertex> vertices;
		std::vector<BodySkinnedBone> skinnedBones;
		std::vector<uint32_t> triangleIndices;  // per-triangle shapes; flat triples
		std::vector<std::string> tags;
		std::vector<std::string> canCollideWithTags;
		std::vector<std::string> noCollideWithTags;
		std::vector<std::string> canCollideWithBones;
		std::vector<std::string> noCollideWithBones;
		std::string disableTag;
		int32_t disablePriority = 0;
		bool operator==(const MeshBody&) const = default;
	};

	// ---- constraints (§5) ----

	enum class ConstraintType : uint32_t
	{
		Generic6Dof = 0,
		ConeTwist = 1,
		StiffSpring = 2,
	};

	// One generic constraint, serialized at the resolved-frame level so replay needs no XML.
	struct Constraint
	{
		ConstraintType type = ConstraintType::Generic6Dof;
		uint32_t boneA = 0;  // indices into Snapshot::bones
		uint32_t boneB = 0;
		Transform frameInA;
		Transform frameInB;
		uint8_t useLinearReferenceFrameA = 0;
		// generic-6DOF limits / springs / motors (also reused by the others where applicable)
		Vec3 linearLowerLimit;
		Vec3 linearUpperLimit;
		Vec3 angularLowerLimit;
		Vec3 angularUpperLimit;
		Vec3 linearStiffness;
		Vec3 angularStiffness;
		Vec3 linearDamping;
		Vec3 angularDamping;
		Vec3 linearEquilibrium;
		Vec3 angularEquilibrium;
		Vec3 linearBounce;
		Vec3 angularBounce;
		// cone-twist
		float swingSpan1 = 0.0f;
		float swingSpan2 = 0.0f;
		float twistSpan = 0.0f;
		float limitSoftness = 1.0f;
		float biasFactor = 0.3f;
		float relaxationFactor = 1.0f;
		// stiff-spring
		float minDistance = 0.0f;
		float maxDistance = 0.0f;
		float stiffness = 0.0f;
		float damping = 0.0f;
		float equilibrium = 0.0f;
		bool operator==(const Constraint&) const = default;
	};

	struct ConstraintGroup
	{
		std::vector<uint32_t> constraintIndices;  // indices into Snapshot::constraints
		bool operator==(const ConstraintGroup&) const = default;
	};

	struct BoneScaleConstraint
	{
		uint32_t bone = 0;  // index into Snapshot::bones
		float scale = 1.0f;
		bool operator==(const BoneScaleConstraint&) const = default;
	};

	// ---- a full system (one SkyrimSystem / one ReplaySystem) ----

	struct Snapshot
	{
		uint32_t systemId = 0;  // stable id used by the per-frame stream and scene-log remove events
		// Actor-group id: which skeleton (NiNode) this system belongs to. Multiple systems on one actor
		// share it. Reserved groundwork for the active-state/disable-tag dedup, which groups systems by
		// skeleton; not consumed yet, so the capture leaves it 0 until that feature populates it.
		uint32_t skeletonId = 0;
		// Wall-clock cost (microseconds) of building this system in-game: reading the physics XML file,
		// parsing it, extracting the NIF skin data, and constructing the bones/bodies/constraints. The
		// harness reports it as the build/read cost. It is MEASURED in-game (the build path is engine-
		// coupled, so it can't be re-run offline) and recorded here, not replayed. 0 if not timed.
		uint32_t buildTimeMicros = 0;
		std::vector<CollisionShape> shapes;
		std::vector<Bone> bones;
		std::vector<MeshBody> bodies;
		std::vector<Constraint> constraints;
		std::vector<ConstraintGroup> constraintGroups;
		std::vector<BoneScaleConstraint> boneScaleConstraints;
		bool operator==(const Snapshot&) const = default;
	};

	// ---- scene-log: incremental actor churn (D3) ----

	enum class SceneEventKind : uint32_t
	{
		AddSystem = 0,
		RemoveSystem = 1,
	};

	struct SceneEvent
	{
		SceneEventKind kind = SceneEventKind::AddSystem;
		uint32_t frame = 0;     // frame index at which to apply the event
		uint32_t systemId = 0;  // RemoveSystem: which system; AddSystem: snapshot.systemId
		Snapshot snapshot;      // populated only for AddSystem
		bool operator==(const SceneEvent&) const = default;
	};

	// ---- per-frame stream (§6) ----

	struct BoneTarget
	{
		uint32_t systemId = 0;
		uint32_t boneIndex = 0;
		QsTransform target;
		bool operator==(const BoneTarget&) const = default;
	};

	struct BodyDisabled
	{
		uint32_t systemId = 0;
		uint32_t bodyIndex = 0;
		uint8_t disabled = 0;
		bool operator==(const BodyDisabled&) const = default;
	};

	struct BoneOutput
	{
		uint32_t systemId = 0;
		uint32_t boneIndex = 0;
		QsTransform transform;
		bool operator==(const BoneOutput&) const = default;
	};

	struct Frame
	{
		float remainingTimeStep = 0.0f;
		float tick = 0.0f;
		Vec3 windSpeed;
		uint8_t reset = 0;
		std::vector<BoneTarget> kinematicTargets;
		std::vector<BodyDisabled> disabled;
		// Present only when this capture is a D8 golden (header.hasGolden == 1): the post-writeTransform
		// transform of each dynamic bone, used by the parity test as the expected output.
		std::vector<BoneOutput> golden;
		bool operator==(const Frame&) const = default;
	};

	// ---- the whole document ----

	struct Header
	{
		uint32_t formatVersion = kReplayFormatVersion;
		std::string gitSha;     // up to kGitShaLength chars, identifies the producing build
		uint32_t configHash = 0;  // hash of the build's compile-time physics config
		uint32_t threadCountHint = 1;  // TBB threads at capture time (parity replays single-threaded)
		uint8_t hasGolden = 0;  // 1 == frames carry golden dynamic-bone outputs
		bool operator==(const Header&) const = default;
	};

	struct Document
	{
		Header header;
		SolverConfig solver;
		std::vector<Snapshot> initialSystems;
		std::vector<SceneEvent> sceneLog;
		std::vector<Frame> frames;
		bool operator==(const Document&) const = default;
	};

	// ===================== binary writer (little-endian, explicit fields) =====================

	class ByteWriter
	{
	public:
		/// Appends one raw byte.
		void u8(uint8_t v) { m_buf.push_back(v); }

		/// Appends a uint32 in little-endian byte order.
		void u32(uint32_t v)
		{
			m_buf.push_back(static_cast<uint8_t>(v & 0xFF));
			m_buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
			m_buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
			m_buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
		}

		/// Appends a signed int32 (two's-complement, little-endian).
		void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }

		/// Appends an IEEE-754 float via its 32-bit pattern (no SIMD-aligned memcpy of the value).
		void f32(float v)
		{
			uint32_t bits;
			std::memcpy(&bits, &v, sizeof(bits));
			u32(bits);
		}

		/// Appends n raw bytes from p.
		void bytes(const char* p, size_t n) { m_buf.insert(m_buf.end(), p, p + n); }

		/// Appends a length-prefixed (uint32) UTF-8 string.
		void str(const std::string& s)
		{
			u32(static_cast<uint32_t>(s.size()));
			bytes(s.data(), s.size());
		}

		/// Appends a fixed-width, zero-padded string (used for the git sha); truncates if longer.
		void fixedStr(const std::string& s, uint32_t width)
		{
			for (uint32_t i = 0; i < width; ++i)
				u8(i < s.size() ? static_cast<uint8_t>(s[i]) : 0);
		}

		/// Appends a Vec3 as three floats (x, y, z).
		void vec3(const Vec3& v) { f32(v.x); f32(v.y); f32(v.z); }
		/// Appends a Vec4 as four floats (x, y, z, w).
		void vec4(const Vec4& v) { f32(v.x); f32(v.y); f32(v.z); f32(v.w); }
		/// Appends a Quat as four floats (x, y, z, w).
		void quat(const Quat& q) { f32(q.x); f32(q.y); f32(q.z); f32(q.w); }
		/// Appends a QsTransform as quat + origin + scale.
		void qs(const QsTransform& t) { quat(t.basis); vec3(t.origin); f32(t.scale); }
		/// Appends a Transform as quat + origin.
		void tf(const Transform& t) { quat(t.basis); vec3(t.origin); }

		/// Moves the accumulated buffer out (leaves this writer empty).
		std::vector<uint8_t> take() { return std::move(m_buf); }
		/// Read-only view of the bytes written so far.
		const std::vector<uint8_t>& data() const { return m_buf; }

	private:
		std::vector<uint8_t> m_buf;
	};

	// ===================== binary reader (bounds-checked, fail-closed) =====================

	class ByteReader
	{
	public:
		/// Wraps a byte span; the reader never reads past [data, data + size).
		ByteReader(const uint8_t* data, size_t size) :
			m_data(data), m_size(size) {}

		/// Bytes not yet consumed.
		size_t remaining() const { return m_size - m_pos; }
		/// True once every byte has been consumed.
		bool atEnd() const { return m_pos == m_size; }

		/// Reads one raw byte (throws if none remain).
		uint8_t u8()
		{
			need(1);
			return m_data[m_pos++];
		}

		/// Reads a little-endian uint32 (throws if fewer than 4 bytes remain).
		uint32_t u32()
		{
			need(4);
			uint32_t v = static_cast<uint32_t>(m_data[m_pos]) |
			             (static_cast<uint32_t>(m_data[m_pos + 1]) << 8) |
			             (static_cast<uint32_t>(m_data[m_pos + 2]) << 16) |
			             (static_cast<uint32_t>(m_data[m_pos + 3]) << 24);
			m_pos += 4;
			return v;
		}

		/// Reads a signed int32 (two's-complement, little-endian).
		int32_t i32() { return static_cast<int32_t>(u32()); }

		/// Reads an IEEE-754 float from its 32-bit pattern.
		float f32()
		{
			uint32_t bits = u32();
			float v;
			std::memcpy(&v, &bits, sizeof(v));
			return v;
		}

		/// Reads a length-prefixed string. The length is validated (via count()) against kMaxCount and
		/// the bytes actually remaining before anything is allocated.
		std::string str()
		{
			uint32_t n = count();
			need(n);
			std::string s(reinterpret_cast<const char*>(m_data + m_pos), n);
			m_pos += n;
			return s;
		}

		/// Reads a fixed-width field and returns the string up to the first zero pad byte.
		std::string fixedStr(uint32_t width)
		{
			need(width);
			size_t len = 0;
			while (len < width && m_data[m_pos + len] != 0)
				++len;
			std::string s(reinterpret_cast<const char*>(m_data + m_pos), len);
			m_pos += width;
			return s;
		}

		/// Reads a Vec3 (three floats).
		Vec3 vec3() { Vec3 v; v.x = f32(); v.y = f32(); v.z = f32(); return v; }
		/// Reads a Vec4 (four floats).
		Vec4 vec4() { Vec4 v; v.x = f32(); v.y = f32(); v.z = f32(); v.w = f32(); return v; }
		/// Reads a Quat (four floats).
		Quat quat() { Quat q; q.x = f32(); q.y = f32(); q.z = f32(); q.w = f32(); return q; }
		/// Reads a QsTransform (quat + origin + scale).
		QsTransform qs() { QsTransform t; t.basis = quat(); t.origin = vec3(); t.scale = f32(); return t; }
		/// Reads a Transform (quat + origin).
		Transform tf() { Transform t; t.basis = quat(); t.origin = vec3(); return t; }

		/// Reads and validates a count field used to size a container. Rejecting here is the core of
		/// the trust-boundary guarantee: a count is never used to allocate before it is bounded both
		/// by kMaxCount and by the bytes physically left in the buffer (every element is >= 1 byte).
		uint32_t count()
		{
			uint32_t n = u32();
			if (n > kMaxCount)
				throw ReplayFormatError("count exceeds kMaxCount (corrupt or hostile file)");
			if (n > remaining())
				throw ReplayFormatError("count exceeds remaining bytes (truncated file)");
			return n;
		}

		/// Reads one byte and throws unless it equals `expected` (used for the file magic).
		void expectByte(uint8_t expected, const char* context)
		{
			if (u8() != expected)
				throw ReplayFormatError(std::string("magic mismatch: ") + context);
		}

	private:
		/// Throws ReplayFormatError unless at least n bytes remain.
		void need(size_t n)
		{
			if (n > remaining())
				throw ReplayFormatError("unexpected end of file (truncated)");
		}

		const uint8_t* m_data;
		size_t m_size;
		size_t m_pos = 0;
	};

	// ===================== element (de)serializers =====================
	//
	// Each writeX(ByteWriter&, const X&) appends one X to the stream, and the matching readX(ByteReader&)
	// reads one back, in the same field order. They are the single source of truth for X's wire layout
	// (writers and readers must stay in lockstep). Every variable-length section is length-prefixed; on
	// read, ByteReader::count() validates the length before allocating, so a malformed file is rejected
	// rather than trusted (the trust-boundary guarantee). The names are self-describing
	// (writeShape/readShape, writeBone/readBone, ...); see the corresponding struct for field meaning.

	inline void writeShape(ByteWriter& w, const CollisionShape& s)
	{
		w.u32(static_cast<uint32_t>(s.type));
		w.f32(s.margin);
		w.vec3(s.localScaling);
		w.vec3(s.halfExtents);
		w.f32(s.radius);
		w.f32(s.height);
		w.u32(static_cast<uint32_t>(s.hullPoints.size()));
		for (auto& p : s.hullPoints)
			w.vec3(p);
		w.u32(static_cast<uint32_t>(s.children.size()));
		for (auto& c : s.children) {
			w.tf(c.localTransform);
			w.u32(c.shapeIndex);
		}
	}

	inline CollisionShape readShape(ByteReader& r)
	{
		CollisionShape s;
		uint32_t type = r.u32();
		if (type > static_cast<uint32_t>(ShapeType::Compound))
			throw ReplayFormatError("invalid shape type");
		s.type = static_cast<ShapeType>(type);
		s.margin = r.f32();
		s.localScaling = r.vec3();
		s.halfExtents = r.vec3();
		s.radius = r.f32();
		s.height = r.f32();
		uint32_t nh = r.count();
		s.hullPoints.reserve(nh);
		for (uint32_t i = 0; i < nh; ++i)
			s.hullPoints.push_back(r.vec3());
		uint32_t nc = r.count();
		s.children.reserve(nc);
		for (uint32_t i = 0; i < nc; ++i) {
			CompoundChild c;
			c.localTransform = r.tf();
			c.shapeIndex = r.u32();
			s.children.push_back(c);
		}
		return s;
	}

	inline void writeStrings(ByteWriter& w, const std::vector<std::string>& v)
	{
		w.u32(static_cast<uint32_t>(v.size()));
		for (auto& s : v)
			w.str(s);
	}

	inline std::vector<std::string> readStrings(ByteReader& r)
	{
		uint32_t n = r.count();
		std::vector<std::string> v;
		v.reserve(n);
		for (uint32_t i = 0; i < n; ++i)
			v.push_back(r.str());
		return v;
	}

	inline void writeBone(ByteWriter& w, const Bone& b)
	{
		w.str(b.name);
		w.f32(b.mass);
		w.f32(b.invMass);
		w.vec3(b.invInertiaDiagLocal);
		w.tf(b.localToRig);
		w.tf(b.rigToLocal);
		w.i32(b.shapeIndex);
		w.f32(b.marginMultiplier);
		w.f32(b.boundingSphereMultiplier);
		w.f32(b.gravityFactor);
		w.f32(b.windFactor);
		w.u32(b.collisionFilter);
		w.u8(b.kinematic);
		w.qs(b.initialWorldTransform);
		writeStrings(w, b.canCollideWithBone);
		writeStrings(w, b.noCollideWithBone);
	}

	inline Bone readBone(ByteReader& r)
	{
		Bone b;
		b.name = r.str();
		b.mass = r.f32();
		b.invMass = r.f32();
		b.invInertiaDiagLocal = r.vec3();
		b.localToRig = r.tf();
		b.rigToLocal = r.tf();
		b.shapeIndex = r.i32();
		b.marginMultiplier = r.f32();
		b.boundingSphereMultiplier = r.f32();
		b.gravityFactor = r.f32();
		b.windFactor = r.f32();
		b.collisionFilter = r.u32();
		b.kinematic = r.u8();
		b.initialWorldTransform = r.qs();
		b.canCollideWithBone = readStrings(r);
		b.noCollideWithBone = readStrings(r);
		return b;
	}

	inline void writeBody(ByteWriter& w, const MeshBody& b)
	{
		w.str(b.name);
		w.u32(static_cast<uint32_t>(b.shapeKind));
		w.f32(b.margin);
		w.f32(b.penetration);
		w.u32(static_cast<uint32_t>(b.vertices.size()));
		for (auto& v : b.vertices) {
			w.vec4(v.skinPos);
			for (float wt : v.weight)
				w.f32(wt);
			for (uint32_t bi : v.boneIdx)
				w.u32(bi);
		}
		w.u32(static_cast<uint32_t>(b.skinnedBones.size()));
		for (auto& sb : b.skinnedBones) {
			w.u32(sb.boneIndex);
			w.qs(sb.vertexToBone);
			w.vec3(sb.localBoundingSphere.center);
			w.f32(sb.localBoundingSphere.radius);
			w.f32(sb.weightThreshold);
			w.u8(sb.isKinematic);
		}
		w.u32(static_cast<uint32_t>(b.triangleIndices.size()));
		for (uint32_t idx : b.triangleIndices)
			w.u32(idx);
		writeStrings(w, b.tags);
		writeStrings(w, b.canCollideWithTags);
		writeStrings(w, b.noCollideWithTags);
		writeStrings(w, b.canCollideWithBones);
		writeStrings(w, b.noCollideWithBones);
		w.str(b.disableTag);
		w.i32(b.disablePriority);
	}

	inline MeshBody readBody(ByteReader& r)
	{
		MeshBody b;
		b.name = r.str();
		uint32_t kind = r.u32();
		if (kind > static_cast<uint32_t>(BodyShapeKind::PerTriangle))
			throw ReplayFormatError("invalid body shape kind");
		b.shapeKind = static_cast<BodyShapeKind>(kind);
		b.margin = r.f32();
		b.penetration = r.f32();
		uint32_t nv = r.count();
		b.vertices.reserve(nv);
		for (uint32_t i = 0; i < nv; ++i) {
			BodyVertex v;
			v.skinPos = r.vec4();
			for (float& wt : v.weight)
				wt = r.f32();
			for (uint32_t& bi : v.boneIdx)
				bi = r.u32();
			b.vertices.push_back(v);
		}
		uint32_t nsb = r.count();
		b.skinnedBones.reserve(nsb);
		for (uint32_t i = 0; i < nsb; ++i) {
			BodySkinnedBone sb;
			sb.boneIndex = r.u32();
			sb.vertexToBone = r.qs();
			sb.localBoundingSphere.center = r.vec3();
			sb.localBoundingSphere.radius = r.f32();
			sb.weightThreshold = r.f32();
			sb.isKinematic = r.u8();
			b.skinnedBones.push_back(sb);
		}
		uint32_t nt = r.count();
		b.triangleIndices.reserve(nt);
		for (uint32_t i = 0; i < nt; ++i)
			b.triangleIndices.push_back(r.u32());
		b.tags = readStrings(r);
		b.canCollideWithTags = readStrings(r);
		b.noCollideWithTags = readStrings(r);
		b.canCollideWithBones = readStrings(r);
		b.noCollideWithBones = readStrings(r);
		b.disableTag = r.str();
		b.disablePriority = r.i32();
		return b;
	}

	inline void writeConstraint(ByteWriter& w, const Constraint& c)
	{
		w.u32(static_cast<uint32_t>(c.type));
		w.u32(c.boneA);
		w.u32(c.boneB);
		w.tf(c.frameInA);
		w.tf(c.frameInB);
		w.u8(c.useLinearReferenceFrameA);
		w.vec3(c.linearLowerLimit);
		w.vec3(c.linearUpperLimit);
		w.vec3(c.angularLowerLimit);
		w.vec3(c.angularUpperLimit);
		w.vec3(c.linearStiffness);
		w.vec3(c.angularStiffness);
		w.vec3(c.linearDamping);
		w.vec3(c.angularDamping);
		w.vec3(c.linearEquilibrium);
		w.vec3(c.angularEquilibrium);
		w.vec3(c.linearBounce);
		w.vec3(c.angularBounce);
		w.f32(c.swingSpan1);
		w.f32(c.swingSpan2);
		w.f32(c.twistSpan);
		w.f32(c.limitSoftness);
		w.f32(c.biasFactor);
		w.f32(c.relaxationFactor);
		w.f32(c.minDistance);
		w.f32(c.maxDistance);
		w.f32(c.stiffness);
		w.f32(c.damping);
		w.f32(c.equilibrium);
	}

	inline Constraint readConstraint(ByteReader& r)
	{
		Constraint c;
		uint32_t type = r.u32();
		if (type > static_cast<uint32_t>(ConstraintType::StiffSpring))
			throw ReplayFormatError("invalid constraint type");
		c.type = static_cast<ConstraintType>(type);
		c.boneA = r.u32();
		c.boneB = r.u32();
		c.frameInA = r.tf();
		c.frameInB = r.tf();
		c.useLinearReferenceFrameA = r.u8();
		c.linearLowerLimit = r.vec3();
		c.linearUpperLimit = r.vec3();
		c.angularLowerLimit = r.vec3();
		c.angularUpperLimit = r.vec3();
		c.linearStiffness = r.vec3();
		c.angularStiffness = r.vec3();
		c.linearDamping = r.vec3();
		c.angularDamping = r.vec3();
		c.linearEquilibrium = r.vec3();
		c.angularEquilibrium = r.vec3();
		c.linearBounce = r.vec3();
		c.angularBounce = r.vec3();
		c.swingSpan1 = r.f32();
		c.swingSpan2 = r.f32();
		c.twistSpan = r.f32();
		c.limitSoftness = r.f32();
		c.biasFactor = r.f32();
		c.relaxationFactor = r.f32();
		c.minDistance = r.f32();
		c.maxDistance = r.f32();
		c.stiffness = r.f32();
		c.damping = r.f32();
		c.equilibrium = r.f32();
		return c;
	}

	inline void writeSnapshot(ByteWriter& w, const Snapshot& s)
	{
		w.u32(s.systemId);
		w.u32(s.skeletonId);
		w.u32(s.buildTimeMicros);
		w.u32(static_cast<uint32_t>(s.shapes.size()));
		for (auto& sh : s.shapes)
			writeShape(w, sh);
		w.u32(static_cast<uint32_t>(s.bones.size()));
		for (auto& b : s.bones)
			writeBone(w, b);
		w.u32(static_cast<uint32_t>(s.bodies.size()));
		for (auto& b : s.bodies)
			writeBody(w, b);
		w.u32(static_cast<uint32_t>(s.constraints.size()));
		for (auto& c : s.constraints)
			writeConstraint(w, c);
		w.u32(static_cast<uint32_t>(s.constraintGroups.size()));
		for (auto& g : s.constraintGroups) {
			w.u32(static_cast<uint32_t>(g.constraintIndices.size()));
			for (uint32_t idx : g.constraintIndices)
				w.u32(idx);
		}
		w.u32(static_cast<uint32_t>(s.boneScaleConstraints.size()));
		for (auto& bsc : s.boneScaleConstraints) {
			w.u32(bsc.bone);
			w.f32(bsc.scale);
		}
	}

	inline Snapshot readSnapshot(ByteReader& r)
	{
		Snapshot s;
		s.systemId = r.u32();
		s.skeletonId = r.u32();
		s.buildTimeMicros = r.u32();
		uint32_t ns = r.count();
		s.shapes.reserve(ns);
		for (uint32_t i = 0; i < ns; ++i)
			s.shapes.push_back(readShape(r));
		uint32_t nb = r.count();
		s.bones.reserve(nb);
		for (uint32_t i = 0; i < nb; ++i)
			s.bones.push_back(readBone(r));
		uint32_t nbd = r.count();
		s.bodies.reserve(nbd);
		for (uint32_t i = 0; i < nbd; ++i)
			s.bodies.push_back(readBody(r));
		uint32_t nc = r.count();
		s.constraints.reserve(nc);
		for (uint32_t i = 0; i < nc; ++i)
			s.constraints.push_back(readConstraint(r));
		uint32_t ng = r.count();
		s.constraintGroups.reserve(ng);
		for (uint32_t i = 0; i < ng; ++i) {
			ConstraintGroup g;
			uint32_t ngi = r.count();
			g.constraintIndices.reserve(ngi);
			for (uint32_t j = 0; j < ngi; ++j)
				g.constraintIndices.push_back(r.u32());
			s.constraintGroups.push_back(std::move(g));
		}
		uint32_t nbsc = r.count();
		s.boneScaleConstraints.reserve(nbsc);
		for (uint32_t i = 0; i < nbsc; ++i) {
			BoneScaleConstraint bsc;
			bsc.bone = r.u32();
			bsc.scale = r.f32();
			s.boneScaleConstraints.push_back(bsc);
		}
		return s;
	}

	inline void writeFrame(ByteWriter& w, const Frame& f)
	{
		w.f32(f.remainingTimeStep);
		w.f32(f.tick);
		w.vec3(f.windSpeed);
		w.u8(f.reset);
		w.u32(static_cast<uint32_t>(f.kinematicTargets.size()));
		for (auto& t : f.kinematicTargets) {
			w.u32(t.systemId);
			w.u32(t.boneIndex);
			w.qs(t.target);
		}
		w.u32(static_cast<uint32_t>(f.disabled.size()));
		for (auto& d : f.disabled) {
			w.u32(d.systemId);
			w.u32(d.bodyIndex);
			w.u8(d.disabled);
		}
		w.u32(static_cast<uint32_t>(f.golden.size()));
		for (auto& g : f.golden) {
			w.u32(g.systemId);
			w.u32(g.boneIndex);
			w.qs(g.transform);
		}
	}

	inline Frame readFrame(ByteReader& r)
	{
		Frame f;
		f.remainingTimeStep = r.f32();
		f.tick = r.f32();
		f.windSpeed = r.vec3();
		f.reset = r.u8();
		uint32_t nt = r.count();
		f.kinematicTargets.reserve(nt);
		for (uint32_t i = 0; i < nt; ++i) {
			BoneTarget t;
			t.systemId = r.u32();
			t.boneIndex = r.u32();
			t.target = r.qs();
			f.kinematicTargets.push_back(t);
		}
		uint32_t nd = r.count();
		f.disabled.reserve(nd);
		for (uint32_t i = 0; i < nd; ++i) {
			BodyDisabled d;
			d.systemId = r.u32();
			d.bodyIndex = r.u32();
			d.disabled = r.u8();
			f.disabled.push_back(d);
		}
		uint32_t ng = r.count();
		f.golden.reserve(ng);
		for (uint32_t i = 0; i < ng; ++i) {
			BoneOutput g;
			g.systemId = r.u32();
			g.boneIndex = r.u32();
			g.transform = r.qs();
			f.golden.push_back(g);
		}
		return f;
	}

	// ===================== top-level document (de)serialization =====================

	/// Serializes a whole Document to a fresh little-endian byte buffer (magic + version + header +
	/// solver + initial systems + scene-log + per-frame stream).
	inline std::vector<uint8_t> serialize(const Document& doc)
	{
		ByteWriter w;
		// fixed header
		w.bytes(kReplayMagic, sizeof(kReplayMagic));
		w.u32(kReplayFormatVersion);
		w.fixedStr(doc.header.gitSha, kGitShaLength);
		w.u32(doc.header.configHash);
		w.u32(doc.header.threadCountHint);
		w.u8(doc.header.hasGolden);

		// solver config
		const SolverConfig& s = doc.solver;
		w.vec3(s.gravity);
		w.f32(s.friction);
		w.u8(s.splitImpulse);
		w.f32(s.splitImpulsePenetrationThreshold);
		w.f32(s.erp2);
		w.f32(s.globalCfm);
		w.f32(s.restitutionVelocityThreshold);
		w.i32(s.solverMode);
		w.f32(s.leastSquaresResidualThreshold);
		w.f32(s.timeTick);
		w.i32(s.maxSubSteps);
		w.u8(s.enableWind);
		w.f32(s.windStrength);
		w.f32(s.distanceForNoWind);
		w.f32(s.distanceForMaxWind);
		w.u8(s.disableDeactivation);

		// initial systems
		w.u32(static_cast<uint32_t>(doc.initialSystems.size()));
		for (auto& snap : doc.initialSystems)
			writeSnapshot(w, snap);

		// scene log
		w.u32(static_cast<uint32_t>(doc.sceneLog.size()));
		for (auto& e : doc.sceneLog) {
			w.u32(static_cast<uint32_t>(e.kind));
			w.u32(e.frame);
			w.u32(e.systemId);
			if (e.kind == SceneEventKind::AddSystem)
				writeSnapshot(w, e.snapshot);
		}

		// per-frame stream
		w.u32(static_cast<uint32_t>(doc.frames.size()));
		for (auto& f : doc.frames)
			writeFrame(w, f);

		return w.take();
	}

	/// Parses a Document from a raw byte span. Validates the magic, requires an exact format-version
	/// match, and bounds-checks every count/length before allocating; throws ReplayFormatError on any
	/// malformed input (fail closed). This is the trust boundary for capture files.
	inline Document deserialize(const uint8_t* data, size_t size)
	{
		ByteReader r(data, size);
		Document doc;

		// magic
		for (char c : kReplayMagic)
			r.expectByte(static_cast<uint8_t>(c), "bad file magic (not an SMP replay capture)");

		// version: exact match required, fail closed
		doc.header.formatVersion = r.u32();
		if (doc.header.formatVersion != kReplayFormatVersion)
			throw ReplayFormatError("unsupported format version (expected " +
			                        std::to_string(kReplayFormatVersion) + ")");

		doc.header.gitSha = r.fixedStr(kGitShaLength);
		doc.header.configHash = r.u32();
		doc.header.threadCountHint = r.u32();
		doc.header.hasGolden = r.u8();

		SolverConfig& s = doc.solver;
		s.gravity = r.vec3();
		s.friction = r.f32();
		s.splitImpulse = r.u8();
		s.splitImpulsePenetrationThreshold = r.f32();
		s.erp2 = r.f32();
		s.globalCfm = r.f32();
		s.restitutionVelocityThreshold = r.f32();
		s.solverMode = r.i32();
		s.leastSquaresResidualThreshold = r.f32();
		s.timeTick = r.f32();
		s.maxSubSteps = r.i32();
		s.enableWind = r.u8();
		s.windStrength = r.f32();
		s.distanceForNoWind = r.f32();
		s.distanceForMaxWind = r.f32();
		s.disableDeactivation = r.u8();

		uint32_t nsys = r.count();
		doc.initialSystems.reserve(nsys);
		for (uint32_t i = 0; i < nsys; ++i)
			doc.initialSystems.push_back(readSnapshot(r));

		uint32_t nlog = r.count();
		doc.sceneLog.reserve(nlog);
		for (uint32_t i = 0; i < nlog; ++i) {
			SceneEvent e;
			uint32_t kind = r.u32();
			if (kind > static_cast<uint32_t>(SceneEventKind::RemoveSystem))
				throw ReplayFormatError("invalid scene-event kind");
			e.kind = static_cast<SceneEventKind>(kind);
			e.frame = r.u32();
			e.systemId = r.u32();
			if (e.kind == SceneEventKind::AddSystem)
				e.snapshot = readSnapshot(r);
			doc.sceneLog.push_back(std::move(e));
		}

		uint32_t nframes = r.count();
		doc.frames.reserve(nframes);
		for (uint32_t i = 0; i < nframes; ++i)
			doc.frames.push_back(readFrame(r));

		return doc;
	}

	/// Convenience overload: parses a Document from a byte vector.
	inline Document deserialize(const std::vector<uint8_t>& buf)
	{
		return deserialize(buf.data(), buf.size());
	}
}
