#pragma once

#include "hdtConvertNi.h"
#include "hdtSkyrimBone.h"
#include "hdtSkyrimBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"
#include "hdtSkinnedMesh/hdtGeneric6DofConstraint.h"
#include "hdtSkinnedMesh/hdtStiffSpringConstraint.h"
#include "hdtSkinnedMesh/hdtConeTwistConstraint.h"
#include "hdtDefaultBBP.h"

namespace hdt
{
	class SkyrimSystem : public SkinnedMeshSystem
	{
		friend class SkyrimSystemCreator;
	public:
		struct BoneData
		{
			uint16_t boneWeights[4];
			uint8_t boneIndices[4];
		};

		SkyrimSystem(NiNode* skeleton);

		SkinnedMeshBone* findBone(IDStr name);
		SkinnedMeshBody* findBody(IDStr name);
		int findBoneIdx(IDStr name);

		void readTransform(float timeStep) override;
		void writeTransform() override;

		const std::vector<Ref<SkinnedMeshBody>>& meshes() const { return m_meshes; }


		Ref<NiNode> m_skeleton;
		Ref<NiNode> m_oldRoot;
		bool m_initialized = false;

		// angular velocity damper
		btQuaternion m_lastRootRotation;
	};

	class XMLReader;

	class SkyrimSystemCreator
	{
	public:
		SkyrimSystemCreator();
		Ref<SkyrimSystem> createSystem(NiNode* skeleton, NiAVObject* model, const DefaultBBP::PhysicsFile file,
		                             std::unordered_map<IDStr, IDStr> renameMap);
		Ref<SkyrimSystem> updateSystem(SkyrimSystem* old_system, NiNode* skeleton, NiAVObject* model, const DefaultBBP::PhysicsFile file,
			std::unordered_map<IDStr, IDStr> renameMap);
	protected:

		using VertexOffsetMap = std::unordered_map<std::string, int>;

		IDStr getRenamedBone(IDStr name);

		Ref<SkyrimSystem> m_mesh;
		NiNode* m_skeleton;
		NiAVObject* m_model;
		XMLReader* m_reader;
		std::unordered_map<IDStr, IDStr> m_renameMap;

		NiNode* findObjectByName(const IDStr& name);
		SkyrimBone* getOrCreateBone(const IDStr& name);

		std::string m_filePath;

		bool findBones(const IDStr& bodyAName, const IDStr& bodyBName, SkyrimBone*& bodyA, SkyrimBone*& bodyB);

		struct BoneTemplate : public btRigidBody::btRigidBodyConstructionInfo
		{
			static btEmptyShape emptyShape[1];

			BoneTemplate() : btRigidBodyConstructionInfo(0, nullptr, emptyShape)
			{
				m_centerOfMassTransform = btTransform::getIdentity();
				m_marginMultipler = 1.f;
			}

			std::shared_ptr<btCollisionShape> m_shape;
			std::vector<IDStr> m_canCollideWithBone;
			std::vector<IDStr> m_noCollideWithBone;
			btTransform m_centerOfMassTransform;
			float m_marginMultipler;
			float m_gravityFactor = 1.0f;
			U32 m_collisionFilter = 0;
		};

		std::unordered_map<IDStr, BoneTemplate> m_boneTemplates;

		enum FrameType
		{
			FrameInA,
			FrameInB,
			FrameInLerp,
			AWithXPointToB,
			AWithYPointToB,
			AWithZPointToB
		};

		bool parseFrameType(const std::string& name, FrameType& type, btTransform& frame);
		static void calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA,
		                      const btQsTransform& trB, btTransform& frameA, btTransform& frameB);

		struct GenericConstraintTemplate
		{
			FrameType frameType = FrameInB;
			bool useLinearReferenceFrameA = false;
			btTransform frame = btTransform::getIdentity();
			btVector3 linearLowerLimit = btVector3(1, 1, 1);
			btVector3 linearUpperLimit = btVector3(-1, -1, -1);
			btVector3 angularLowerLimit = btVector3(1, 1, 1);
			btVector3 angularUpperLimit = btVector3(-1, -1, -1);
			btVector3 linearStiffness = btVector3(0, 0, 0);
			btVector3 angularStiffness = btVector3(0, 0, 0);
			btVector3 linearDamping = btVector3(0, 0, 0);
			btVector3 angularDamping = btVector3(0, 0, 0);
			btVector3 linearEquilibrium = btVector3(0, 0, 0);
			btVector3 angularEquilibrium = btVector3(0, 0, 0);
			btVector3 linearBounce = btVector3(0, 0, 0);
			btVector3 angularBounce = btVector3(0, 0, 0);
		};

		std::unordered_map<IDStr, GenericConstraintTemplate> m_genericConstraintTemplates;

		struct StiffSpringConstraintTemplate
		{
			float minDistanceFactor = 1;
			float maxDistanceFactor = 1;
			float stiffness = 0;
			float damping = 0;
			float equilibriumFactor = 0.5;
		};

		std::unordered_map<IDStr, StiffSpringConstraintTemplate> m_stiffSpringConstraintTemplates;

		struct ConeTwistConstraintTemplate
		{
			btTransform frame = btTransform::getIdentity();
			FrameType frameType = FrameInB;
			bool angularOnly = false;
			float swingSpan1 = 0;
			float swingSpan2 = 0;
			float twistSpan = 0;
			float limitSoftness = 1.0f;
			float biasFactor = 0.3f;
			float relaxationFactor = 1.0f;
		};

		std::unordered_map<IDStr, ConeTwistConstraintTemplate> m_coneTwistConstraintTemplates;
		std::unordered_map<IDStr, std::shared_ptr<btCollisionShape>> m_shapes;

		std::pair< Ref<SkyrimBody>, VertexOffsetMap > generateMeshBody(const std::string name, const DefaultBBP::NameSet& names);

		void readFrameLerp(btTransform& tr);
		void readBoneTemplate(BoneTemplate& dest);
		void readGenericConstraintTemplate(GenericConstraintTemplate& dest);
		void readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest);
		void readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest);

		const BoneTemplate& getBoneTemplate(const IDStr& name);
		const GenericConstraintTemplate& getGenericConstraintTemplate(const IDStr& name);
		const StiffSpringConstraintTemplate& getStiffSpringConstraintTemplate(const IDStr& name);
		const ConeTwistConstraintTemplate& getConeTwistConstraintTemplate(const IDStr& name);

		SkyrimBone* SkyrimSystemCreator::createBoneFromNodeName(const IDStr& bodyName, const IDStr& templateName = "", const bool readTemplate = false, SkyrimSystem* old_system = nullptr);
		void SkyrimSystemCreator::readOrUpdateBone(SkyrimSystem* old_system = nullptr);
		Ref<SkyrimBody> readPerVertexShape(DefaultBBP::NameMap meshNameMap);
		Ref<SkyrimBody> readPerTriangleShape(DefaultBBP::NameMap meshNameMap);
		Ref<Generic6DofConstraint> readGenericConstraint();
		Ref<StiffSpringConstraint> readStiffSpringConstraint();
		Ref<ConeTwistConstraint> readConeTwistConstraint();
		Ref<ConstraintGroup> readConstraintGroup();
		std::shared_ptr<btCollisionShape> readShape();

		template <typename ... Args>
		void Error(const char* fmt, Args ... args);
		template <typename ... Args>
		void Warning(const char* fmt, Args ... args);
		template <typename ... Args>
		void VMessage(const char* fmt, Args ... args);

		std::vector<std::shared_ptr<btCollisionShape>> m_shapeRefs;
	};
}
