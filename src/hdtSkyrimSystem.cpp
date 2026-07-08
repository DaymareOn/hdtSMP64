#include "hdtSkyrimSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"

#include "HavokUtils.h"
#include "XmlReader.h"
#include "hdtSkyrimPhysicsWorld.h"

// F16C isn't supported on super old processors. AVX2+ (AVX processors can have it, but not guaranteed)
#if defined(__AVX2__) || defined(__AVX512F__)
#	include <immintrin.h>
#else
// float32
// Martin Kallman
//
// Fast half-precision to single-precision floating point conversion
//  - Supports signed zero and denormals-as-zero (DAZ)
//  - Does not support infinities or NaN
//  - Few, partially pipelinable, non-branching instructions,
//  - Core operations ~6 clock cycles on modern x86-64
static void __float32(float* __restrict out, const uint16_t in)
{
	uint32_t t1;
	uint32_t t2;
	uint32_t t3;

	t1 = in & 0x7fff;  // Non-sign bits
	t2 = in & 0x8000;  // Sign bit
	t3 = in & 0x7c00;  // Exponent

	t1 <<= 13;  // Align mantissa on MSB
	t2 <<= 16;  // Shift sign bit into position

	t1 += 0x38000000;  // Adjust bias

	t1 = (t3 == 0 ? 0 : t1);  // Denormals-as-zero

	t1 |= t2;  // Re-insert sign bit

	*((uint32_t*)out) = t1;
};
#endif

namespace hdt
{
	static constexpr float PI = 3.1415926535897932384626433832795f;

	btEmptyShape SkyrimSystemCreator::BoneTemplate::emptyShape[1];

	SkinnedMeshBone* SkyrimSystem::findBone(const RE::BSFixedString& name)
	{
		for (auto i : m_bones) {
			if (i->m_name == name) {
				return i.get();
			}
		}

		return nullptr;
	}

	SkinnedMeshBody* SkyrimSystem::findBody(const RE::BSFixedString& name)
	{
		for (auto i : m_meshes) {
			if (i->m_name == name) {
				return i.get();
			}
		}

		return nullptr;
	}

	int SkyrimSystem::findBoneIdx(const RE::BSFixedString& name)
	{
		for (int i = 0; i < m_bones.size(); ++i) {
			if (m_bones[i]->m_name == name) {
				return i;
			}
		}

		return -1;
	}

	SkyrimSystem::SkyrimSystem(RE::NiNode* skeleton) :
		m_skeleton(skeleton), m_oldRoot(nullptr)
	{
		m_oldRoot = m_skeleton;
	}

	float SkyrimSystem::prepareForRead(float timeStep)
	{
		auto newRoot = m_skeleton.get();
		while (newRoot->parent) {
			newRoot = newRoot->parent;
		}

		if (m_oldRoot != newRoot) {
			timeStep = RESET_PHYSICS;
		}

		if (!m_initialized) {
			timeStep = RESET_PHYSICS;
			m_initialized = true;
		}

		if (timeStep <= RESET_PHYSICS) {
			if (!this->block_resetting) {
				updateTransformUpDown(m_skeleton.get(), true);
			}

			m_lastRootRotation = convertNi(m_skeleton->world.rotate);
		} else if (m_skeleton->parent == RE::PlayerCharacter::GetSingleton()->Get3D2()) {
			if (SkyrimPhysicsWorld::get()->m_resetPc > 0) {
				timeStep = RESET_PHYSICS;
				updateTransformUpDown(m_skeleton.get(), true);
				m_lastRootRotation = convertNi(m_skeleton->world.rotate);
			} else if (!RE::PlayerCamera::GetSingleton()->GetRuntimeData2().isWeapSheathed || RE::PlayerCamera::GetSingleton()->currentState->id == RE::CameraState::kFirstPerson)  // isWeaponSheathed or potentially isCameraFree || cameraState is first person
			{
				m_lastRootRotation = convertNi(m_skeleton->world.rotate);
			} else {
				btQuaternion newRot = convertNi(m_skeleton->world.rotate);
				btVector3 rotAxis;
				float rotAngle;
				btTransformUtil::calculateDiffAxisAngleQuaternion(m_lastRootRotation, newRot, rotAxis, rotAngle);

				if (SkyrimPhysicsWorld::get()->m_clampRotations) {
					float limit = SkyrimPhysicsWorld::get()->m_rotationSpeedLimit * timeStep;

					if (rotAngle < -limit || rotAngle > limit) {
						rotAngle = btClamped(rotAngle, -limit, limit);
						btQuaternion clampedRot(rotAxis, rotAngle);
						m_lastRootRotation = clampedRot * m_lastRootRotation;
						m_skeleton->world.rotate = convertBt(m_lastRootRotation);

						const auto& children = m_skeleton->GetChildren();
						for (uint16_t i = 0; i < children.size(); ++i) {
							auto node = castNiNode(children[i].get());
							if (node) {
								updateTransformUpDown(node, true);
							}
						}
					}
				} else if (SkyrimPhysicsWorld::get()->m_unclampedResets) {
					float limit = SkyrimPhysicsWorld::get()->m_unclampedResetAngle * timeStep;

					if (rotAngle < -limit || rotAngle > limit) {
						timeStep = RESET_PHYSICS;
						updateTransformUpDown(m_skeleton.get(), true);
						m_lastRootRotation = convertNi(m_skeleton->world.rotate);
					}
				}
			}
		}

		m_oldRoot = hdt::make_nismart(newRoot);
		return timeStep;
	}

	SkyrimSystemCreator::SkyrimSystemCreator()
	{
	}

	void SkyrimSystemCreator::indexBone(SkyrimBone* bone)
	{
		m_boneIndex.emplace(bone->m_name.data(), bone);
	}

	SkyrimBone* SkyrimSystemCreator::findBoneFromIndex(const RE::BSFixedString& name) const
	{
		auto it = m_boneIndex.find(name.data());
		return it != m_boneIndex.end() ? it->second : nullptr;
	}

	RE::NiNode* SkyrimSystemCreator::findObjectByName(const RE::BSFixedString& name)
	{
		// TODO check it's not a lurker skeleton
		return findNode(m_skeleton, name);
	}

	SkyrimBone* SkyrimSystemCreator::getOrCreateBone(const RE::BSFixedString& name)
	{
		auto bone = findBoneFromIndex(getRenamedBone(name));
		if (bone) {
			return bone;
		}

		logger::warn("Bone {} used before being created, trying to create it with current default values", name.c_str());
		// Create from the renamed name too: the lookup above used it, and under a rename
		// map (skeleton merge / DynamicHDT) the raw name's node typically no longer
		// exists — creating from the raw name would drop the reference or split the
		// bone into two identities. Constraint readers already create from renamed names.
		return createBoneFromNodeName(getRenamedBone(name));
	}

	RE::BSFixedString SkyrimSystemCreator::getRenamedBone(const RE::BSFixedString& name)
	{
		auto iter = m_renameMap.find(name);
		if (iter != m_renameMap.end())
			return iter->second;
		return name;
	}

	RE::BSTSmartPointer<SkyrimSystem> SkyrimSystemCreator::createOrUpdateSystem(RE::NiNode* skeleton, RE::NiAVObject* model, DefaultBBP::PhysicsFile_t* file, std::unordered_map<RE::BSFixedString, RE::BSFixedString>&& renameMap, SkyrimSystem* old_system, RE::NiPoint3 clipCenter, float clipRadius, ObstructionCache* cache, std::vector<RE::NiPoint3>* outClippedWorldTris, bool useCollisionMesh)
	{
		auto path = file->first;
		if (path.empty()) {
			return nullptr;
		}

		auto loaded = readAllFile(path.c_str());
		if (loaded.empty()) {
			return nullptr;
		}

		m_renameMap = std::move(renameMap);
		m_skeleton = skeleton;
		m_model = model;
		m_filePath = path;
		m_clipCenter = clipCenter;
		m_clipRadius = clipRadius;
		m_obstructionCache = cache;
		m_useCollisionMesh = useCollisionMesh;
		m_clippedTris.clear();
		m_outClippedWorldTris = outClippedWorldTris;
		if (m_outClippedWorldTris)
			m_outClippedWorldTris->clear();

		XMLReader reader((uint8_t*)loaded.data(), loaded.size());
		m_reader = &reader;

		m_reader->nextStartElement();
		if (m_reader->GetName() != "system") {
			if (!old_system) {
				updateTransformUpDown(m_skeleton, true);
			}

			return nullptr;
		}

		auto meshNameMap = file->second;

		m_mesh = RE::make_smart<SkyrimSystem>(skeleton);
		m_boneIndex.clear();

		// This forces the skeleton into a neutral reference pose, which avoids building invalid shape data
		// We pull the references directly from havok for the exact same reference data the engine uses
		std::vector<std::pair<RE::NiAVObject*, RE::NiTransform>> savedPoses;

		if (auto* userData = skeleton->GetUserData()) {
			if (auto* actor = userData->As<RE::Actor>()) {
				if (auto havokSkel = havok::getAnimationSkeleton(actor)) {
					savedPoses.reserve(havokSkel->bones.size());

					for (int32_t i = 0; i < static_cast<int32_t>(havokSkel->bones.size()); ++i) {
						if (auto boneNode = skeleton->GetObjectByName(RE::BSFixedString(havokSkel->bones[i].name.data()))) {
							savedPoses.emplace_back(boneNode, boneNode->local);

							RE::hkQsTransform refPose;
							if (havok::getReferencePoseByIndex(havokSkel, i, refPose))
								boneNode->local = havok::hKQsTransformToNiTransform(refPose);
						}
					}

					if (!savedPoses.empty()) {
						RE::NiUpdateData updateData;
						skeleton->Update(updateData);
					}
				}
			}
		}

		if (!old_system) {
			updateTransformUpDown(m_skeleton, true);
		}

		try {
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					const auto name = m_reader->GetName();
					if (name == "bone") {
						readOrUpdateBone();
					} else if (name == "bone-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultBoneInfo = getBoneTemplate(extends);
						readBoneTemplate(defaultBoneInfo);
						m_boneTemplates[clsname] = defaultBoneInfo;
					} else if (name == "per-vertex-shape") {
						auto shape = readPerVertexShape(meshNameMap);
						if (shape && shape->m_vertices.size()) {
							m_mesh->m_meshes.push_back(shape);
							shape->m_mesh = m_mesh.get();
						}
					} else if (name == "per-triangle-shape") {
						auto shape = readPerTriangleShape(&meshNameMap);
						if (shape && shape->m_vertices.size()) {
							m_mesh->m_meshes.push_back(shape);
							shape->m_mesh = m_mesh.get();
						}
					} else if (name == "constraint-group") {
						auto constraint = readConstraintGroup();
						if (constraint)
							m_mesh->m_constraintGroups.push_back(constraint);
					} else if (name == "generic-constraint") {
						auto constraint = readGenericConstraint();
						if (constraint)
							m_mesh->m_constraints.push_back(constraint);
					} else if (name == "stiffspring-constraint") {
						auto constraint = readStiffSpringConstraint();
						if (constraint)
							m_mesh->m_constraints.push_back(constraint);
					} else if (name == "conetwist-constraint") {
						auto constraint = readConeTwistConstraint();
						if (constraint)
							m_mesh->m_constraints.push_back(constraint);
					} else if (name == "generic-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
						readGenericConstraintTemplate(defaultGenericConstraintTemplate);
						m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
					} else if (name == "stiffspring-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
						readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
						m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
					} else if (name == "conetwist-constraint-default") {
						auto clsname = m_reader->getAttribute("name", "");
						auto extends = m_reader->getAttribute("extends", "");
						auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
						readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
						m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
					} else if (name == "shape") {
						auto attrName = m_reader->getAttribute("name");
						auto shape = readShape();
						if (shape) {
							m_shapeRefs.push_back(shape);
							m_shapes.insert(std::make_pair(attrName, shape));
						}
					} else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
		} catch (const std::string& err) {
			logger::error("xml parse error - {}", err.c_str());
			return nullptr;
		}

		if (m_deferredBuilds.size() > 2) {
			tbb::parallel_for_each(
				m_deferredBuilds.begin(), m_deferredBuilds.end(),
				[](const DeferredBuild& db) {
					if (db.vertexShape)
						db.vertexShape->autoGen();
					db.body->finishBuild();
				});
		} else if (!m_deferredBuilds.empty()) {
			for (const auto& db : m_deferredBuilds) {
				if (db.vertexShape)
					db.vertexShape->autoGen();
				db.body->finishBuild();
			}
		}

		m_deferredBuilds.clear();

		if (m_reader->GetErrorCode() != Xml::ErrorCode::None) {
			logger::error("xml parse error - {}", m_reader->GetErrorMessage());
			return nullptr;
		}

		m_mesh->m_skeleton = hdt::make_nismart(m_skeleton);
		m_mesh->m_shapeRefs.swap(m_shapeRefs);
		std::sort(m_mesh->m_bones.begin(), m_mesh->m_bones.end(), [](const auto& a, const auto& b) {
			return static_cast<SkyrimBone*>(a.get())->m_depth < static_cast<SkyrimBone*>(b.get())->m_depth;
		});

		// Restore the original pose to avoid a visual 1 Havok tick T-pose (only for visual reasons, it won't break anything otherwise)
		for (auto& [node, transform] : savedPoses) node->local = transform;
		if (!savedPoses.empty()) {
			RE::NiUpdateData updateData;
			skeleton->Update(updateData);
		}

		return m_mesh->valid() ? m_mesh : nullptr;
	}

	RE::BSTSmartPointer<ConstraintGroup> SkyrimSystemCreator::readConstraintGroup()
	{
		RE::BSTSmartPointer<ConstraintGroup> ret = RE::make_smart<ConstraintGroup>();

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();

				if (name == "generic-constraint") {
					auto constraint = readGenericConstraint();
					if (constraint)
						ret->m_constraints.push_back(constraint);
				} else if (name == "stiffspring-constraint") {
					auto constraint = readStiffSpringConstraint();
					if (constraint)
						ret->m_constraints.push_back(constraint);
				} else if (name == "conetwist-constraint") {
					auto constraint = readConeTwistConstraint();
					if (constraint)
						ret->m_constraints.push_back(constraint);
				} else if (name == "generic-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultGenericConstraintTemplate = getGenericConstraintTemplate(extends);
					readGenericConstraintTemplate(defaultGenericConstraintTemplate);
					m_genericConstraintTemplates[clsname] = defaultGenericConstraintTemplate;
				} else if (name == "stiffspring-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultStiffSpringConstraintTemplate = getStiffSpringConstraintTemplate(extends);
					readStiffSpringConstraintTemplate(defaultStiffSpringConstraintTemplate);
					m_stiffSpringConstraintTemplates[clsname] = defaultStiffSpringConstraintTemplate;
				} else if (name == "conetwist-constraint-default") {
					auto clsname = m_reader->getAttribute("name", "");
					auto extends = m_reader->getAttribute("extends", "");
					auto defaultConeTwistConstraintTemplate = getConeTwistConstraintTemplate(extends);
					readConeTwistConstraintTemplate(defaultConeTwistConstraintTemplate);
					m_coneTwistConstraintTemplates[clsname] = defaultConeTwistConstraintTemplate;
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
		return ret;
	}

	void SkyrimSystemCreator::readBoneTemplate(BoneTemplate& cinfo)
	{
		bool clearCollide = true;
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (name == "mass")
					cinfo.m_mass = m_reader->readFloat();
				else if (name == "inertia")
					cinfo.m_localInertia = m_reader->readVector3();
				else if (name == "centerOfMassTransform")
					cinfo.m_centerOfMassTransform = m_reader->readTransform();
				else if (name == "linearDamping")
					cinfo.m_linearDamping = m_reader->readFloat();
				else if (name == "angularDamping")
					cinfo.m_angularDamping = m_reader->readFloat();
				else if (name == "friction")
					cinfo.m_friction = m_reader->readFloat();
				else if (name == "rollingFriction")
					cinfo.m_rollingFriction = m_reader->readFloat();
				else if (name == "restitution")
					cinfo.m_restitution = m_reader->readFloat();
				else if (name == "margin-multiplier")
					cinfo.m_marginMultipler = m_reader->readFloat();
				else if (name == "shape") {
					auto shape = readShape();
					if (shape) {
						m_shapeRefs.push_back(shape);
						cinfo.m_collisionShape = shape.get();
					} else
						cinfo.m_collisionShape = BoneTemplate::emptyShape;
				} else if (name == "collision-filter")
					cinfo.m_collisionFilter = m_reader->readInt();
				else if (name == "can-collide-with-bone") {
					if (clearCollide) {
						cinfo.m_canCollideWithBone.clear();
						cinfo.m_noCollideWithBone.clear();
						clearCollide = false;
					}
					cinfo.m_canCollideWithBone.push_back(m_reader->readText());
				} else if (name == "no-collide-with-bone") {
					if (clearCollide) {
						cinfo.m_canCollideWithBone.clear();
						cinfo.m_noCollideWithBone.clear();
						clearCollide = false;
					}
					cinfo.m_noCollideWithBone.push_back(m_reader->readText());
				} else if (name == "gravity-factor") {
					cinfo.m_gravityFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
				} else if (name == "wind-factor") {
					cinfo.m_windFactor = std::max(m_reader->readFloat(), 0.0f);
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	std::shared_ptr<btCollisionShape> SkyrimSystemCreator::readShape()
	{
		auto typeStr = m_reader->getAttribute("type");
		if (typeStr == "ref") {
			auto shapeName = m_reader->getAttribute("name");
			m_reader->skipCurrentElement();
			auto iter = m_shapes.find(shapeName);
			if (iter != m_shapes.end())
				return iter->second;
			logger::warn("unknown shape - {}", shapeName.c_str());
			return nullptr;
		}
		if (typeStr == "box") {
			btVector3 halfExtend(0, 0, 0);
			float margin = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "halfExtend")
						halfExtend = m_reader->readVector3();
					else if (name == "margin")
						margin = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			auto ret = std::make_shared<btBoxShape>(halfExtend);
			ret->setMargin(margin);
			return ret;
		}
		if (typeStr == "sphere") {
			float radius = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "radius")
						radius = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			return std::make_shared<btSphereShape>(radius);
		}
		if (typeStr == "capsule") {
			float radius = 0;
			float height = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "radius")
						radius = m_reader->readFloat();
					else if (name == "height")
						height = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			return std::make_shared<btCapsuleShape>(radius, height);
		}
		if (typeStr == "hull") {
			float margin = 0;
			auto ret = std::make_shared<btConvexHullShape>();
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "point")
						ret->addPoint(m_reader->readVector3(), false);
					else if (name == "margin")
						margin = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			ret->recalcLocalAabb();
			return ret->getNumPoints() ? ret : nullptr;
		}
		if (typeStr == "cylinder") {
			float height = 0;
			float radius = 0;
			float margin = 0;
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					auto name = m_reader->GetName();
					if (name == "height")
						height = m_reader->readFloat();
					else if (name == "radius")
						radius = m_reader->readFloat();
					else if (name == "margin")
						margin = m_reader->readFloat();
					else {
						logger::warn("unknown element - {}", name.c_str());
						m_reader->skipCurrentElement();
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}

			if (radius >= 0 && height >= 0) {
				auto ret = std::make_shared<btCylinderShape>(btVector3(radius, height, radius));
				ret->setMargin(margin);
				return ret;
			}
			return nullptr;
		}
		if (typeStr == "compound") {
			auto ret = std::make_shared<btCompoundShape>();
			while (m_reader->Inspect()) {
				if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
					if (m_reader->GetName() == "child") {
						btTransform tr;
						std::shared_ptr<btCollisionShape> shape;

						while (m_reader->Inspect()) {
							if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
								if (m_reader->GetName() == "transform") {
									tr = m_reader->readTransform();
								} else if (m_reader->GetName() == "shape") {
									shape = readShape();
								} else {
									logger::warn("unknown element - {}", m_reader->GetName().c_str());
									m_reader->skipCurrentElement();
								}
							} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
								break;
						}

						if (shape) {
							ret->addChildShape(tr, shape.get());
							m_shapeRefs.push_back(shape);
						}
					}
				} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
					break;
			}
			return ret->getNumChildShapes() ? ret : nullptr;
		}
		logger::warn("Unknown shape type {}", typeStr.c_str());
		return nullptr;
	}

	void SkyrimSystemCreator::readOrUpdateBone()
	{
		RE::BSFixedString name = getRenamedBone(m_reader->getAttribute("name"));
		if (findBoneFromIndex(name)) {
			logger::warn("Bone {} already exists, skipped", name.c_str());
			m_reader->skipCurrentElement();
			return;
		}

		RE::BSFixedString cls = m_reader->getAttribute("template", "");
		if (!createBoneFromNodeName(name, cls, true))
			m_reader->skipCurrentElement();
	}

	SkyrimBone* SkyrimSystemCreator::createBoneFromNodeName(const RE::BSFixedString& bodyName, const RE::BSFixedString& templateName, const bool readTemplate)
	{
		auto node = findObjectByName(bodyName);
		if (node) {
			logger::info("Found node named {}, creating bone", bodyName.c_str());
			auto boneTemplate = getBoneTemplate(templateName);
			if (readTemplate)
				readBoneTemplate(boneTemplate);
			auto bone = new SkyrimBone(node->name.c_str(), node, this->m_skeleton, boneTemplate);
			bone->m_localToRig = boneTemplate.m_centerOfMassTransform;
			bone->m_rigToLocal = boneTemplate.m_centerOfMassTransform.inverse();
			bone->m_marginMultipler = boneTemplate.m_marginMultipler;
			bone->m_gravityFactor = boneTemplate.m_gravityFactor;
			bone->m_windFactor = boneTemplate.m_windFactor;

			bone->readTransform(RESET_PHYSICS);

			m_mesh->m_bones.push_back(hdt::make_smart(bone));
			indexBone(bone);
			return bone;
		}
		logger::warn("Node named {} doesn't exist, skipped, no bone created", bodyName.c_str());
		return nullptr;
	}

	// ---- Lever B: build obstructions from the game's coarse HAVOK collision mesh instead of the dense render
	// mesh. World statics store a compressed collision mesh (hkpCompressedMeshShape) that is 1-2 orders of
	// magnitude coarser than the render geometry, so colliding hair/cloth against it is far cheaper. Objects
	// whose collision is not an extractable compressed mesh (terrain heightfields, box/convex primitives) get
	// no collider -- there is no render-mesh fallback, so the gap is visible in the obstruction counts. ----

	// 1 / bhkWorldScale (0.0142875): converts Havok units to Skyrim units.
	static constexpr float kHavokToSkyrim = 69.99124f;
	// Bethesda quantizes each compressed-mesh chunk vertex to 1/1000 Havok unit around the chunk offset.
	static constexpr float kChunkQuant = 1.0f / 1000.0f;

	static inline void hkStore(const RE::hkVector4& v, float out[4]) { _mm_storeu_ps(out, v.quad); }

	// Transform a shape-local Havok vertex by the rigid body's world transform (R*v + T), then to Skyrim world.
	static RE::NiPoint3 havokLocalToSkyrimWorld(const RE::hkTransform& xf, const RE::hkVector4& vLocal)
	{
		float v[4], c0[4], c1[4], c2[4], t[4];
		hkStore(vLocal, v);
		hkStore(xf.rotation.col0, c0);
		hkStore(xf.rotation.col1, c1);
		hkStore(xf.rotation.col2, c2);
		hkStore(xf.translation, t);
		return RE::NiPoint3{
			(c0[0] * v[0] + c1[0] * v[1] + c2[0] * v[2] + t[0]) * kHavokToSkyrim,
			(c0[1] * v[0] + c1[1] * v[1] + c2[1] * v[2] + t[1]) * kHavokToSkyrim,
			(c0[2] * v[0] + c1[2] * v[1] + c2[2] * v[2] + t[2]) * kHavokToSkyrim
		};
	}

	// Apply a chunk's per-instance transform, matching Bethesda's compressed-mesh layout (as decoded by
	// NifSkope's reference gltools.cpp): the transform's translation is added to the chunk-space point
	// (offset + quantized vertex) FIRST, then the whole sum is rotated by the transform's quaternion.
	// The QsTransform's scale is 1 for these rigid chunk transforms and is deliberately ignored, exactly
	// as the reference decoder does. (px,py,pz) is the already-dequantized chunk-space point.
	static RE::hkVector4 applyChunkTransform(const RE::hkQsTransform& xf, float px, float py, float pz)
	{
		float q[4], t[4];
		hkStore(xf.rotation.vec, q);
		hkStore(xf.translation, t);
		const float vx = px + t[0], vy = py + t[1], vz = pz + t[2];
		const float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
		const float ax = 2.f * (qy * vz - qz * vy);
		const float ay = 2.f * (qz * vx - qx * vz);
		const float az = 2.f * (qx * vy - qy * vx);
		return RE::hkVector4(
			vx + qw * ax + (qy * az - qz * ay),
			vy + qw * ay + (qz * ax - qx * az),
			vz + qw * az + (qx * ay - qy * ax), 0.f);
	}

	// Compose the bhkRigidBodyT shape offset into the body transform. A bhkRigidBodyT stores the collision
	// shape's transform relative to the body frame (a quaternion + translation) separately from the body's
	// own transform; GetTransform returns ONLY the body/node transform, so shape-local vertices placed with
	// it alone land at the body origin instead of the shape origin (a chair's collision came out ~0.5m too
	// low). Return xf * T so shape-local -> world is correct: R_eff = R_xf * R_T, t_eff = R_xf * t_T + t_xf.
	static RE::hkTransform composeBodyT(const RE::hkTransform& xf, const RE::bhkRigidBodyT* rbT)
	{
		float c0[4], c1[4], c2[4], tx[4], q[4], tt[4];
		hkStore(xf.rotation.col0, c0);
		hkStore(xf.rotation.col1, c1);
		hkStore(xf.rotation.col2, c2);
		hkStore(xf.translation, tx);
		hkStore(rbT->rotation.vec, q);
		hkStore(rbT->translation, tt);

		// Columns of the offset's rotation matrix, from its quaternion (x,y,z,w).
		const float x = q[0], y = q[1], z = q[2], w = q[3];
		const float tcol[3][3] = {
			{ 1.f - 2.f * (y * y + z * z), 2.f * (x * y + w * z), 2.f * (x * z - w * y) },      // col 0
			{ 2.f * (x * y - w * z), 1.f - 2.f * (x * x + z * z), 2.f * (y * z + w * x) },      // col 1
			{ 2.f * (x * z + w * y), 2.f * (y * z - w * x), 1.f - 2.f * (x * x + y * y) }       // col 2
		};
		// Left-multiply each offset column (and the offset translation) by the body rotation R_xf.
		const auto byRxf = [&](const float v[3], float out[3]) {
			out[0] = c0[0] * v[0] + c1[0] * v[1] + c2[0] * v[2];
			out[1] = c0[1] * v[0] + c1[1] * v[1] + c2[1] * v[2];
			out[2] = c0[2] * v[0] + c1[2] * v[1] + c2[2] * v[2];
		};
		float e0[3], e1[3], e2[3], et[3];
		byRxf(tcol[0], e0);
		byRxf(tcol[1], e1);
		byRxf(tcol[2], e2);
		byRxf(tt, et);

		RE::hkTransform eff;
		eff.rotation.col0 = RE::hkVector4(e0[0], e0[1], e0[2], 0.f);
		eff.rotation.col1 = RE::hkVector4(e1[0], e1[1], e1[2], 0.f);
		eff.rotation.col2 = RE::hkVector4(e2[0], e2[1], e2[2], 0.f);
		eff.translation = RE::hkVector4(et[0] + tx[0], et[1] + tx[1], et[2] + tx[2], 0.f);
		return eff;
	}

	// Unwrap MOPP / bv-tree single-shape containers down to the compressed mesh shape, if any.
	static const RE::hkpCompressedMeshShape* asCompressedMesh(const RE::hkpShape* shape)
	{
		for (int guard = 0; shape && guard < 8; ++guard) {
			if (shape->type == RE::hkpShapeType::kCompressedMesh)
				return static_cast<const RE::hkpCompressedMeshShape*>(shape);
			// A MOPP wraps the collection directly in its child member. Do NOT go through GetContainer()
			// here: that returns the wrapped COLLECTION's container interface, whose GetChildShape decodes
			// a single triangle of the mesh -- which would skip right past the compressed mesh we want.
			if (shape->type == RE::hkpShapeType::kMOPP) {
				shape = static_cast<const RE::hkpMoppBvTreeShape*>(shape)->child.childShape;
				continue;
			}
			const RE::hkpShapeContainer* container = shape->GetContainer();
			if (!container)
				return nullptr;
			RE::hkpShapeBuffer buffer;
			shape = container->GetChildShape(container->GetFirstKey(), buffer);
		}
		return nullptr;
	}

	// Collect every compressed collision mesh under obj (recursing children), each with its rigid body's
	// world transform, so the vertices can be placed in Skyrim world space.
	static void gatherCollisionMeshes(RE::NiAVObject* obj,
		std::vector<std::pair<const RE::hkpCompressedMeshShape*, RE::hkTransform>>& out)
	{
		if (!obj)
			return;
		if (auto* col = obj->GetCollisionObject())
			if (auto* rb = col->GetRigidBody())
				if (auto* hkrb = rb->GetRigidBody()) {
					if (auto* mesh = asCompressedMesh(hkrb->GetShape())) {
						RE::hkTransform xf;
						rb->GetTransform(xf);
						// A bhkRigidBodyT offsets its collision shape from the body frame; GetTransform omits that
						// offset, so fold it in or the mesh sits at the body origin (a chair came out ~0.5m low).
						if (auto* rbT = skyrim_cast<RE::bhkRigidBodyT*>(rb))
							xf = composeBodyT(xf, rbT);
						out.emplace_back(mesh, xf);
					} else if (auto* shape = hkrb->GetShape()) {
						logger::info("world collision: node '{}' has collision shape type {} (not a compressed mesh), skipped",
							obj->name.c_str(), static_cast<int>(shape->type));
					}
				}
		if (auto* node = obj->AsNode())
			for (auto& child : node->GetChildren())
				gatherCollisionMeshes(child.get(), out);
	}

	// Cheap predicate mirroring gatherCollisionMeshes' walk, but stopping at the first compressed mesh:
	// answers "would Lever B build any collider for this object?" without decoding vertices. Cell detection
	// uses it to admit only genuinely collidable objects, so it never tracks empty (non-colliding) obstructions.
	bool nodeHasExtractableCollision(RE::NiAVObject* root)
	{
		if (!root)
			return false;
		if (auto* col = root->GetCollisionObject())
			if (auto* rb = col->GetRigidBody())
				if (auto* hkrb = rb->GetRigidBody())
					if (asCompressedMesh(hkrb->GetShape()))
						return true;
		if (auto* node = root->AsNode())
			for (auto& child : node->GetChildren())
				if (nodeHasExtractableCollision(child.get()))
					return true;
		return false;
	}

	// Decode the object's havok collision geometry into world-space vertices + triangle indices. Handles the
	// big (uncompressed) triangles and the compressed chunks (quantized verts + triangle strips/lists).
	static bool extractCollisionMesh(RE::NiAVObject* objectRoot, std::vector<RE::NiPoint3>& outWorld,
		std::vector<uint32_t>& outIdx)
	{
		std::vector<std::pair<const RE::hkpCompressedMeshShape*, RE::hkTransform>> shapes;
		gatherCollisionMeshes(objectRoot, shapes);

		const auto emit = [&](uint32_t a, uint32_t b, uint32_t c) {
			if (a != b && b != c && a != c) {  // drop degenerate (strip-stitch) triangles
				outIdx.push_back(a);
				outIdx.push_back(b);
				outIdx.push_back(c);
			}
		};

		for (const auto& [mesh, xf] : shapes) {
			const uint32_t bigBase = static_cast<uint32_t>(outWorld.size());
			for (const auto& bv : mesh->bigVertices)
				outWorld.push_back(havokLocalToSkyrimWorld(xf, bv));
			for (const auto& bt : mesh->bigTriangles)
				emit(bigBase + bt.a, bigBase + bt.b, bigBase + bt.c);

			for (const auto& chunk : mesh->chunks) {
				const uint32_t base = static_cast<uint32_t>(outWorld.size());
				float off[4];
				hkStore(chunk.offset, off);
				const bool hasXf = chunk.transformIndex != 0xFFFF && chunk.transformIndex < mesh->transforms.size();
				const RE::hkQsTransform* qxf = hasXf ? &mesh->transforms[chunk.transformIndex] : nullptr;

				const int nv = chunk.vertices.size() / 3;
				for (int i = 0; i < nv; ++i) {
					const float lx = off[0] + chunk.vertices[i * 3 + 0] * kChunkQuant;
					const float ly = off[1] + chunk.vertices[i * 3 + 1] * kChunkQuant;
					const float lz = off[2] + chunk.vertices[i * 3 + 2] * kChunkQuant;
					const RE::hkVector4 local = qxf ? applyChunkTransform(*qxf, lx, ly, lz) : RE::hkVector4(lx, ly, lz, 0.f);
					outWorld.push_back(havokLocalToSkyrimWorld(xf, local));
				}

				const int nIdx = chunk.indices.size();
				int pos = 0;
				for (const auto stripLen : chunk.stripLengths) {  // triangle strips (alternating winding)
					for (int k = 0; k + 2 < stripLen && pos + k + 2 < nIdx; ++k) {
						uint32_t a = base + chunk.indices[pos + k];
						uint32_t b = base + chunk.indices[pos + k + 1];
						uint32_t c = base + chunk.indices[pos + k + 2];
						if (k & 1)
							std::swap(b, c);
						emit(a, b, c);
					}
					pos += stripLen;
				}
				for (; pos + 2 < nIdx; pos += 3)  // any trailing indices are a plain triangle list
					emit(base + chunk.indices[pos], base + chunk.indices[pos + 1], base + chunk.indices[pos + 2]);
			}
		}
		return !outIdx.empty();
	}

	// Fill the obstruction cache from the object's havok collision mesh (in tri's local space, so the existing
	// crop/build path reuses it exactly like the render-mesh path). Returns false if there is no extractable
	// compressed collision mesh, leaving the cache empty so the object gets no collider.
	static bool fillGeomFromHavok(RE::NiAVObject* objectRoot, RE::BSTriShape* tri, ObstructionMeshCache& geom)
	{
		std::vector<RE::NiPoint3> world;
		std::vector<uint32_t> idx;
		if (!extractCollisionMesh(objectRoot, world, idx)) {
			logger::info("world collision: no extractable collision mesh under '{}'", objectRoot->name.c_str());
			return false;
		}
		logger::info("world collision: extracted {} verts / {} tris from '{}' havok collision",
			world.size(), idx.size() / 3, objectRoot->name.c_str());
		geom.world = tri->world;
		const RE::NiTransform inv = tri->world.Invert();
		geom.localPos.resize(world.size());
		for (size_t i = 0; i < world.size(); ++i)
			geom.localPos[i] = inv * world[i];
		geom.indices = std::move(idx);
		return true;
	}

	// Squared distance from point p to triangle (a,b,c). Standard closest-feature test (Ericson, Real-Time
	// Collision Detection): find whether the closest point lies on a vertex, an edge, or the face interior,
	// and return the squared distance to it. Used by the obstruction crop so a big floor triangle is kept
	// when the clip sphere touches its SURFACE, even if all three of its vertices are outside the sphere --
	// the previous vertex-only test dropped exactly those large triangles the feature needs.
	static float sqDistPointTri(const RE::NiPoint3& p, const RE::NiPoint3& a, const RE::NiPoint3& b, const RE::NiPoint3& c)
	{
		const RE::NiPoint3 ab = b - a, ac = c - a, ap = p - a;
		const float d1 = ab.Dot(ap), d2 = ac.Dot(ap);
		if (d1 <= 0.f && d2 <= 0.f)
			return ap.SqrLength();  // closest is vertex A
		const RE::NiPoint3 bp = p - b;
		const float d3 = ab.Dot(bp), d4 = ac.Dot(bp);
		if (d3 >= 0.f && d4 <= d3)
			return bp.SqrLength();  // vertex B
		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
			const float v = d1 / (d1 - d3);
			return (ap - ab * v).SqrLength();  // edge AB
		}
		const RE::NiPoint3 cp = p - c;
		const float d5 = ab.Dot(cp), d6 = ac.Dot(cp);
		if (d6 >= 0.f && d5 <= d6)
			return cp.SqrLength();  // vertex C
		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
			const float w = d2 / (d2 - d6);
			return (ap - ac * w).SqrLength();  // edge AC
		}
		const float va = d3 * d6 - d5 * d4;
		if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
			const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return (bp - (c - b) * w).SqrLength();  // edge BC
		}
		const float denom = 1.f / (va + vb + vc);
		const float v = vb * denom, w = vc * denom;
		const RE::NiPoint3 closest = a + ab * v + ac * w;
		return (p - closest).SqrLength();  // face interior
	}

	std::pair<RE::BSTSmartPointer<SkyrimBody>, SkyrimSystemCreator::VertexOffsetMap> SkyrimSystemCreator::generateMeshBody(const std::string name, DefaultBBP::NameSet_t* names)
	{
		RE::BSTSmartPointer<SkyrimBody> body = RE::make_smart<SkyrimBody>();
		body->m_name = name;

		int vertexStart = 0;
		int boneStart = 0;

		VertexOffsetMap vertexOffsetMap;

		for (auto& meshName : *names) {
			// We wouldn't find the trishape here without the ActorManager::fixArmorNameMaps() fix when the related bug happens
			// (for example when doing the smp reset).
			auto* triShape = castBSTriShape(findObject(m_model, meshName.c_str()));
			auto* dynamicShape = castBSDynamicTriShape(findObject(m_model, meshName.c_str()));
			if (!triShape) {
				continue;
			}

			if (!triShape->GetGeometryRuntimeData().skinInstance) {
				// Unskinned mesh (e.g. static world geometry): there is no skin data to read, so we
				// fake a trivial, static skin entirely in our own structures - every vertex is bound
				// 100% to a single bone (the mesh's parent node), with the mesh's local transform as
				// the skin-to-bone bind. The default bone template has mass 0, so that bone is
				// kinematic and the resulting body is a static collider. We only READ the live
				// geometry buffer; we never fabricate NiSkinInstance/NiSkinData/NiSkinPartition.
				auto* parentNode = triShape->parent;
				const auto& grd = triShape->GetGeometryRuntimeData();
				auto* renderer = grd.rendererData;
				const auto vertexCount = triShape->GetTrishapeRuntimeData().vertexCount;
				// Collision-mesh mode never reads the render buffers (geometry comes from havok), so it must
				// not require them: many statics keep no CPU-visible copy of their render geometry.
				if (!parentNode || (!m_useCollisionMesh && (!renderer || !renderer->rawVertexData || vertexCount == 0))) {
					continue;
				}

				const RE::BSFixedString boneName = parentNode->name;
				auto bone = static_cast<SkinnedMeshBone*>(findBoneFromIndex(boneName));
				if (!bone) {
					auto defaultBoneInfo = getBoneTemplate("");
					auto newBone = new SkyrimBone(boneName, parentNode, this->m_skeleton, defaultBoneInfo);
					m_mesh->m_bones.push_back(hdt::make_smart(newBone));
					indexBone(newBone);
					bone = newBone;
				}

				// skin-to-bone maps the mesh's rest vertices into the parent (bone) frame; that is just
				// the mesh's local transform. For a cropped obstruction, use the CLIP SPHERE (in this mesh's
				// local space) as the bone bound instead of the whole-mesh model bound: otherwise the
				// broadphase AABB covers the entire monolithic cell mesh and every actor in the cell pairs
				// with the obstruction (and pays its per-frame update) even thousands of units away.
				RE::NiPoint3 boundCenter;
				float boundRadius;
				if (m_clipRadius > 0.f) {
					boundCenter = triShape->world.Invert() * m_clipCenter;  // world -> mesh-local
					boundRadius = m_clipRadius;
				} else {
					const auto modelBound = triShape->GetModelData().modelBound;
					boundCenter = modelBound.center;
					boundRadius = modelBound.radius;
				}
				body->addBone(bone, convertNi(triShape->local), BoundingSphere(convertNi(boundCenter), boundRadius));

				auto vDesc = grd.vertexDesc;
				const auto vSize = vDesc.GetSize();
				const bool fullPrec = vDesc.HasFlag(RE::BSGraphics::Vertex::Flags::VF_FULLPREC);
				// May be null in collision-mesh mode (which never calls readPos); guarded above otherwise.
				uint8_t* vBlock = renderer ? renderer->rawVertexData : nullptr;

				// Decode one packed vertex position (full- or half-precision) from the GPU buffer.
				auto readPos = [&](uint32_t j) -> RE::NiPoint3 {
					RE::NiPoint3 pos;
					if (fullPrec) {
						pos = *reinterpret_cast<RE::NiPoint3*>(&vBlock[j * vSize]);
					} else {
						const uint16_t* h = reinterpret_cast<const uint16_t*>(&vBlock[j * vSize]);
#if defined(__AVX2__) || defined(__AVX512F__)
						float fp[4];
						_mm_storeu_ps(fp, _mm_cvtph_ps(_mm_loadl_epi64(reinterpret_cast<const __m128i*>(h))));
						pos.x = fp[0];
						pos.y = fp[1];
						pos.z = fp[2];
#else
						__float32(&pos.x, h[0]);
						__float32(&pos.y, h[1]);
						__float32(&pos.z, h[2]);
#endif
					}
					return pos;
				};
				// Fill one body vertex bound 100% to the single (kinematic) bone.
				auto fillVertex = [&](Vertex& v, const RE::NiPoint3& local) {
					v.m_skinPos = convertNi(local);
					v.m_weight[0] = 1.0f;
					v.m_weight[1] = v.m_weight[2] = v.m_weight[3] = 0.0f;
					v.m_boneIdx[0] = boneStart;
					v.m_boneIdx[1] = v.m_boneIdx[2] = v.m_boneIdx[3] = 0;
				};

				if (m_clipRadius <= 0.f) {
					// No cropping (also the fallback for any non-obstruction caller): take all vertices.
					body->m_vertices.resize(vertexStart + vertexCount);
					for (uint32_t j = 0; j < vertexCount; ++j)
						fillVertex(body->m_vertices[j + vertexStart], readPos(j));
					vertexOffsetMap.insert({ meshName, vertexStart });
				} else if (m_obstructionCache && (m_useCollisionMesh || renderer->rawIndexData)) {
					// Obstruction crop. Read the raw geometry into the per-obstruction cache once (keyed by
					// trishape); rebuilds as the actor moves reuse it with no GPU read. Then keep only the
					// triangles with a vertex inside the clip sphere, compacting the vertices they reference.
					auto& geom = (*m_obstructionCache)[triShape];
					if (geom.localPos.empty()) {
						if (m_useCollisionMesh) {
							// Lever B: fill from the object's coarse havok collision mesh. Leaves geom empty if
							// there is no extractable compressed collision mesh, so this object gets no collider.
							fillGeomFromHavok(m_model, triShape, geom);
						} else {
							geom.world = triShape->world;
							geom.localPos.resize(vertexCount);
							for (uint32_t j = 0; j < vertexCount; ++j)
								geom.localPos[j] = readPos(j);
							const uint16_t* idxBuf = renderer->rawIndexData;
							const auto triangleCount = triShape->GetTrishapeRuntimeData().triangleCount;
							geom.indices.assign(idxBuf, idxBuf + static_cast<size_t>(triangleCount) * 3);
						}
					}

					const float r2 = m_clipRadius * m_clipRadius;
					// World positions of every source vertex (once): reused for the near flag, the
					// sphere-vs-triangle test, and the debug wireframe capture.
					std::vector<RE::NiPoint3> worldPos(geom.localPos.size());
					std::vector<uint8_t> nearFlag(geom.localPos.size(), 0);
					for (size_t j = 0; j < geom.localPos.size(); ++j) {
						worldPos[j] = geom.world * geom.localPos[j];
						if ((m_clipCenter - worldPos[j]).SqrLength() <= r2)
							nearFlag[j] = 1;
					}

					std::vector<int> remap(geom.localPos.size(), -1);
					auto& kept = m_clippedTris[std::string(meshName)];
					for (size_t t = 0; t + 2 < geom.indices.size(); t += 3) {
						const uint32_t vi[3] = { geom.indices[t], geom.indices[t + 1], geom.indices[t + 2] };
						// Keep the triangle if any vertex is inside the sphere (cheap common case), or if the
						// sphere reaches the triangle's surface -- the latter catches large floor triangles
						// whose vertices are all outside the sphere but whose face is right under the actor.
						bool keep = nearFlag[vi[0]] || nearFlag[vi[1]] || nearFlag[vi[2]];
						if (!keep)
							keep = sqDistPointTri(m_clipCenter, worldPos[vi[0]], worldPos[vi[1]], worldPos[vi[2]]) <= r2;
						if (!keep)
							continue;
						// Capture the kept triangle's world-space vertices for the debug wireframe overlay, up to a
						// cap so a huge render-mesh crop can't balloon this buffer (the overlay samples it anyway).
						if (m_outClippedWorldTris && m_outClippedWorldTris->size() < 12000)
							for (int k = 0; k < 3; ++k)
								m_outClippedWorldTris->push_back(worldPos[vi[k]]);
						for (int k = 0; k < 3; ++k) {
							if (remap[vi[k]] < 0) {
								remap[vi[k]] = static_cast<int>(body->m_vertices.size());
								body->m_vertices.emplace_back();
								fillVertex(body->m_vertices.back(), geom.localPos[vi[k]]);
							}
							kept.push_back(remap[vi[k]]);
						}
					}
					logger::info("world collision: crop '{}' kept {} of {} tris ({} verts), captured {} viz tris",
						meshName, kept.size() / 3, geom.indices.size() / 3, body->m_vertices.size(),
						m_outClippedWorldTris ? m_outClippedWorldTris->size() / 3 : 0);
					vertexOffsetMap.insert({ meshName, vertexStart });
				}

				boneStart = static_cast<int>(body->m_skinnedBones.size());
				vertexStart = static_cast<int>(body->m_vertices.size());
				continue;
			}

			RE::NiSkinInstance* skinInstance = triShape->GetGeometryRuntimeData().skinInstance.get();
			RE::NiSkinData* skinData = skinInstance->skinData.get();
			for (uint32_t boneIdx = 0; boneIdx < skinData->GetBoneCount(); ++boneIdx) {
				auto node = skinInstance->bones[boneIdx];
				if (!node) {
					continue;
				}
				const auto& boneBound = skinData->GetBoneDataBound(boneIdx);
				auto boundingSphere = BoundingSphere(convertNi(boneBound.center), boneBound.radius);
				const RE::BSFixedString& boneName = node->name;
				auto bone = static_cast<SkinnedMeshBone*>(findBoneFromIndex(boneName));
				if (!bone) {
					auto defaultBoneInfo = getBoneTemplate("");
					auto newBone = new SkyrimBone(boneName, node->AsNode(), this->m_skeleton, defaultBoneInfo);
					m_mesh->m_bones.push_back(hdt::make_smart(newBone));
					indexBone(newBone);
					bone = newBone;
					logger::info("Created bone {} added to body {}, created without default values", boneName.c_str(), name);
				}

				body->addBone(bone, convertNi(skinData->GetBoneDataSkinToBone(boneIdx)), boundingSphere);
			}

			RE::NiSkinPartition* skinPartition = triShape->GetGeometryRuntimeData().skinInstance->skinPartition.get();
			body->m_vertices.resize(vertexStart + skinPartition->vertexCount);

			// vertices data are all the same in every partitions
			auto partition = skinPartition->partitions.data();
			auto vFlags = partition->vertexDesc.GetFlags();
			auto vSize = partition->vertexDesc.GetSize();
			auto vertexBlock = partition->buffData->rawVertexData;

			uint8_t* dynamicVData = nullptr;
			if (dynamicShape)
				dynamicVData = static_cast<uint8_t*>(dynamicShape->GetDynamicTrishapeRuntimeData().dynamicData);

			uint8_t boneOffset = 0;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_VERTEX)
				boneOffset += 16;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_UV_2)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_NORMAL)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_TANGENT)
				boneOffset += 4;

			if (vFlags & RE::BSGraphics::Vertex::Flags::VF_COLORS)
				boneOffset += 4;

			for (uint32_t j = 0; j < skinPartition->vertexCount; ++j) {
				RE::NiPoint3* vertexPos;

				if (dynamicShape && dynamicVData)
					vertexPos = reinterpret_cast<RE::NiPoint3*>(&dynamicVData[j * 16]);
				else
					vertexPos = reinterpret_cast<RE::NiPoint3*>(&vertexBlock[j * vSize]);

				body->m_vertices[j + vertexStart].m_skinPos = convertNi(*vertexPos);

				SkyrimSystem::BoneData* boneData = reinterpret_cast<SkyrimSystem::BoneData*>(&vertexBlock[j * vSize + boneOffset]);

#if defined(__AVX2__) || defined(__AVX512F__)
				// batch convert all 4 bone weights FP16 to FP32 through F16C hardware instruction
				__m128i halfWeights = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(boneData->boneWeights));
				_mm_storeu_ps(body->m_vertices[j + vertexStart].m_weight, _mm_cvtph_ps(halfWeights));
				// cleanse garbage NIF data for unused bones
				for (int k = partition->bonesPerVertex; k < 4; ++k) {
					body->m_vertices[j + vertexStart].m_weight[k] = 0.0f;
				}
#else
				for (int k = 0; k < partition->bonesPerVertex && k < 4; ++k) {
					__float32(&body->m_vertices[j + vertexStart].m_weight[k], boneData->boneWeights[k]);
				}
#endif

				for (int k = 0; k < partition->bonesPerVertex && k < 4; ++k) {
					auto localBoneIndex = boneData->boneIndices[k];
					assert(localBoneIndex < body->m_skinnedBones.size());
					body->m_vertices[j + vertexStart].m_boneIdx[k] = localBoneIndex + boneStart;
				}
			}

			vertexOffsetMap.insert({ meshName, vertexStart });
			boneStart = static_cast<int>(body->m_skinnedBones.size());
			vertexStart = static_cast<int>(body->m_vertices.size());
		}

		if (0 == vertexStart) {
			m_reader->skipCurrentElement();
			return { nullptr, {} };
		}

		for (auto& i : body->m_vertices)
			i.sortWeight();

		return { body, vertexOffsetMap };
	}

	RE::BSTSmartPointer<SkyrimBody> SkyrimSystemCreator::readPerVertexShape(DefaultBBP::NameMap_t meshNameMap)
	{
		auto name = m_reader->getAttribute("name");
		auto it = meshNameMap.find(name);
		auto names = (it == meshNameMap.end()) ? DefaultBBP::NameSet_t({ name }) : it->second;

		auto body = generateMeshBody(name, &names).first;
		if (!body) {
			return nullptr;
		}

		auto shape = RE::make_smart<PerVertexShape>(body.get());

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto nodeName = m_reader->GetName();
				if (nodeName == "priority") {
					logger::warn("priority is deprecated and no longer used");
					m_reader->skipCurrentElement();
				} else if (nodeName == "margin") {
					shape->m_shapeProp.margin = m_reader->readFloat();
				} else if (nodeName == "shared") {
					auto str = m_reader->readText();
					if (str == "public") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					} else if (str == "internal") {
						body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
					} else if (str == "external") {
						body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
					} else if (str == "private") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
					} else {
						logger::warn("unknown shared value, use default value \"public\"");
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					}
				} else if (nodeName == "tag") {
					body->m_tags.push_back(m_reader->readText());
				} else if (nodeName == "can-collide-with-tag") {
					body->m_canCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "no-collide-with-tag") {
					body->m_noCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_canCollideWithBones.insert(bone);
				} else if (nodeName == "no-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_noCollideWithBones.insert(bone);
				} else if (nodeName == "weight-threshold") {
					auto boneName = m_reader->getAttribute("bone");
					float wt = m_reader->readFloat();
					for (int i = 0; i < body->m_skinnedBones.size(); ++i) {
						if (body->m_skinnedBones[i].ptr->m_name == getRenamedBone(boneName)) {
							body->m_skinnedBones[i].weightThreshold = wt;
							break;
						}
					}
				} else if (nodeName == "disable-tag") {
					body->m_disableTag = m_reader->readText();
				} else if (nodeName == "disable-priority") {
					body->m_disablePriority = m_reader->readInt();
				} else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}

		m_deferredBuilds.push_back({ body.get(), shape.get() });

		return body;
	}

	RE::BSTSmartPointer<SkyrimBody> SkyrimSystemCreator::readPerTriangleShape(DefaultBBP::NameMap_t* meshNameMap)
	{
		auto name = m_reader->getAttribute("name");
		auto it = meshNameMap->find(name);
		auto names = (it == meshNameMap->end()) ? DefaultBBP::NameSet_t({ name }) : it->second;

		auto bodyData = generateMeshBody(name, &names);
		auto body = bodyData.first;
		auto vertexOffsetMap = bodyData.second;
		if (!body)
			return nullptr;

		auto shape = RE::make_smart<PerTriangleShape>(body.get());

		for (auto entry : vertexOffsetMap) {
			auto* g = castBSTriShape(findObject(m_model, entry.first.c_str()));
			if (!g) {
				continue;
			}
			if (g->GetGeometryRuntimeData().skinInstance) {
				int offset = entry.second;
				RE::NiSkinPartition* skinPartition = g->GetGeometryRuntimeData().skinInstance->skinPartition.get();
				for (int i = 0; i < skinPartition->partitions.size(); ++i) {
					auto& partition = skinPartition->partitions[i];
					for (int j = 0; j < partition.triangles; ++j)
						shape->addTriangle(partition.triList[j * 3] + offset, partition.triList[j * 3 + 1] + offset,
							partition.triList[j * 3 + 2] + offset);
				}
			} else if (m_clipRadius > 0.f) {
				// Cropped obstruction: generateMeshBody already selected the near triangles (as flat,
				// body-local vertex indices) into m_clippedTris. Just emit them; no offset needed.
				auto clipIt = m_clippedTris.find(entry.first);
				if (clipIt != m_clippedTris.end())
					for (size_t t = 0; t + 2 < clipIt->second.size(); t += 3)
						shape->addTriangle(clipIt->second[t], clipIt->second[t + 1], clipIt->second[t + 2]);
			} else {
				// Unskinned, uncropped mesh: triangles come from the geometry's own index buffer rather than
				// a skin partition's triList. Mirrors the unskinned vertex path in generateMeshBody.
				const auto& grd = g->GetGeometryRuntimeData();
				auto* renderer = grd.rendererData;
				const auto triangleCount = g->GetTrishapeRuntimeData().triangleCount;
				if (!renderer || !renderer->rawIndexData) {
					logger::warn("Unskinned shape {} has no index data, skipped", entry.first.c_str());
					continue;
				}
				const int offset = entry.second;
				const uint16_t* idx = renderer->rawIndexData;
				for (uint32_t t = 0; t < triangleCount; ++t)
					shape->addTriangle(idx[t * 3] + offset, idx[t * 3 + 1] + offset, idx[t * 3 + 2] + offset);
			}
		}

		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto nodeName = m_reader->GetName();
				if (nodeName == "priority") {
					logger::warn("priority is deprecated and no longer used");
					m_reader->skipCurrentElement();
				} else if (nodeName == "margin") {
					shape->m_shapeProp.margin = m_reader->readFloat();
				} else if (nodeName == "shared") {
					auto str = m_reader->readText();
					if (str == "public") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					} else if (str == "internal") {
						body->m_shared = SkyrimBody::SharedType::SHARED_INTERNAL;
					} else if (str == "external") {
						body->m_shared = SkyrimBody::SharedType::SHARED_EXTERNAL;
					} else if (str == "private") {
						body->m_shared = SkyrimBody::SharedType::SHARED_PRIVATE;
					} else {
						logger::warn("unknown shared value, use default value \"public\"");
						body->m_shared = SkyrimBody::SharedType::SHARED_PUBLIC;
					}
				} else if (nodeName == "prenetration" || nodeName == "penetration") {
					shape->m_shapeProp.penetration = m_reader->readFloat();
				} else if (nodeName == "tag") {
					body->m_tags.push_back(m_reader->readText());
				} else if (nodeName == "no-collide-with-tag") {
					body->m_noCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-tag") {
					body->m_canCollideWithTags.insert(m_reader->readText());
				} else if (nodeName == "can-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_canCollideWithBones.insert(bone);
				} else if (nodeName == "no-collide-with-bone") {
					auto bone = getOrCreateBone(m_reader->readText());
					if (bone)
						body->m_noCollideWithBones.insert(bone);
				} else if (nodeName == "weight-threshold") {
					auto boneName = m_reader->getAttribute("bone");
					float wt = m_reader->readFloat();
					for (int i = 0; i < body->m_skinnedBones.size(); ++i) {
						if (body->m_skinnedBones[i].ptr->m_name == getRenamedBone(boneName)) {
							body->m_skinnedBones[i].weightThreshold = wt;
						}
					}
				} else if (nodeName == "disable-tag") {
					body->m_disableTag = m_reader->readText();
				} else if (nodeName == "disable-priority") {
					body->m_disablePriority = m_reader->readInt();
				} else {
					logger::warn("unknown element - {}", nodeName.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag) {
				break;
			}
		}

		m_deferredBuilds.push_back({ body.get(), nullptr });

		return body;
	}

	void SkyrimSystemCreator::readFrameLerp(btTransform& tr)
	{
		tr.setIdentity();
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (name == "translationLerp")
					tr.getOrigin().setX(m_reader->readFloat());
				else if (name == "rotationLerp")
					tr.getOrigin().setY(m_reader->readFloat());
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	bool SkyrimSystemCreator::parseFrameType(const std::string& name, FrameType& frameType, btTransform& frame)
	{
		if (name == "frameInA") {
			frameType = FrameInA;
			frame = m_reader->readTransform();
		} else if (name == "frameInB") {
			frameType = FrameInB;
			frame = m_reader->readTransform();
		} else if (name == "frameInLerp") {
			frameType = FrameInLerp;
			readFrameLerp(frame);
		} else
			return false;
		return true;
	}

	void SkyrimSystemCreator::readGenericConstraintTemplate(GenericConstraintTemplate& dest)
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (parseFrameType(name, dest.frameType, dest.frame))
					;
				else if (name == "enableLinearSprings")
					dest.enableLinearSprings = m_reader->readBool();
				else if (name == "enableAngularSprings")
					dest.enableAngularSprings = m_reader->readBool();
				else if (name == "linearStiffnessLimited")
					dest.linearStiffnessLimited = m_reader->readBool();
				else if (name == "angularStiffnessLimited")
					dest.angularStiffnessLimited = m_reader->readBool();

				else if (name == "springDampingLimited")
					dest.springDampingLimited = m_reader->readBool();
				else if (name == "linearNonHookeanDamping")
					dest.linearNonHookeanDamping = m_reader->readVector3();
				else if (name == "angularNonHookeanDamping")
					dest.angularNonHookeanDamping = m_reader->readVector3();
				else if (name == "linearNonHookeanStiffness")
					dest.linearNonHookeanStiffness = m_reader->readVector3();
				else if (name == "angularNonHookeanStiffness")
					dest.angularNonHookeanStiffness = m_reader->readVector3();

				else if (name == "linearMotors")
					dest.linearMotors = m_reader->readBool();
				else if (name == "angularMotors")
					dest.angularMotors = m_reader->readBool();
				else if (name == "linearServoMotors")
					dest.linearServoMotors = m_reader->readBool();
				else if (name == "angularServoMotors")
					dest.angularServoMotors = m_reader->readBool();
				else if (name == "linearTargetVelocity")
					dest.linearTargetVelocity = m_reader->readVector3();
				else if (name == "angularTargetVelocity")
					dest.angularTargetVelocity = m_reader->readVector3();
				else if (name == "linearMaxMotorForce")
					dest.linearMaxMotorForce = m_reader->readVector3();
				else if (name == "angularMaxMotorForce")
					dest.angularMaxMotorForce = m_reader->readVector3();

				else if (name == "stopERP")
					dest.stopERP = m_reader->readFloat();
				else if (name == "stopCFM")
					dest.stopCFM = m_reader->readFloat();
				else if (name == "motorERP")
					dest.motorERP = m_reader->readFloat();
				else if (name == "motorCFM")
					dest.motorCFM = m_reader->readFloat();

				else if (name == "useLinearReferenceFrameA")
					dest.useLinearReferenceFrameA = m_reader->readBool();
				else if (name == "linearLowerLimit")
					dest.linearLowerLimit = m_reader->readVector3();
				else if (name == "linearUpperLimit")
					dest.linearUpperLimit = m_reader->readVector3();
				else if (name == "angularLowerLimit")
					dest.angularLowerLimit = m_reader->readVector3();
				else if (name == "angularUpperLimit")
					dest.angularUpperLimit = m_reader->readVector3();
				else if (name == "linearStiffness")
					dest.linearStiffness = m_reader->readVector3();
				else if (name == "angularStiffness")
					dest.angularStiffness = m_reader->readVector3();
				else if (name == "linearDamping")
					dest.linearDamping = m_reader->readVector3();
				else if (name == "angularDamping")
					dest.angularDamping = m_reader->readVector3();
				else if (name == "linearEquilibrium")
					dest.linearEquilibrium = m_reader->readVector3();
				else if (name == "angularEquilibrium")
					dest.angularEquilibrium = m_reader->readVector3();
				else if (name == "linearBounce")
					dest.linearBounce = m_reader->readVector3();
				else if (name == "angularBounce")
					dest.angularBounce = m_reader->readVector3();
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	bool SkyrimSystemCreator::findBones(const RE::BSFixedString& bodyAName, const RE::BSFixedString& bodyBName, SkyrimBone*& bodyA, SkyrimBone*& bodyB)
	{
		bodyA = findBoneFromIndex(bodyAName);
		bodyB = findBoneFromIndex(bodyBName);

		if (!bodyA) {
			logger::warn("constraint {} <-> {} : bone for bodyA doesn't exist, will try to create it", bodyAName.c_str(), bodyBName.c_str());
			bodyA = createBoneFromNodeName(bodyAName);
			if (!bodyA) {
				m_reader->skipCurrentElement();
				return false;
			}
		}
		if (!bodyB) {
			logger::warn("constraint {} <-> {} : bone for bodyB doesn't exist, will try to create it", bodyAName.c_str(), bodyBName.c_str());
			bodyB = createBoneFromNodeName(bodyBName);
			if (!bodyB) {
				m_reader->skipCurrentElement();
				return false;
			}
		}
		if (bodyA == bodyB) {
			logger::warn("constraint between same object {} <-> {}, skipped", bodyAName.c_str(), bodyBName.c_str());
			m_reader->skipCurrentElement();
			return false;
		}

		if (bodyA->m_rig.isKinematicObject() && bodyB->m_rig.isKinematicObject()) {
			logger::warn("constraint between two kinematic object {} <-> {}, skipped", bodyAName.c_str(), bodyBName.c_str());
			m_reader->skipCurrentElement();
			return false;
		}

		logger::info("OK: constraint between object {} <-> {}", bodyAName.c_str(), bodyBName.c_str());
		return true;
	}

	btQuaternion rotFromAtoB(const btVector3& a, const btVector3& b)
	{
		auto axis = a.cross(b);
		if (axis.fuzzyZero())
			return btQuaternion::getIdentity();
		float sinA = axis.length();
		float cosA = a.dot(b);
		float angle = btAtan2(cosA, sinA);
		return btQuaternion(axis, angle);
	}

	void SkyrimSystemCreator::calcFrame(FrameType type, const btTransform& frame, const btQsTransform& trA, const btQsTransform& trB, btTransform& frameA, btTransform& frameB)
	{
		btQsTransform frameInWorld;
		switch (type) {
		case FrameInA:
			frameA = frame;
			frameInWorld = trA * frame;
			frameB = (trB.inverse() * frameInWorld).asTransform();
			break;
		case FrameInB:
			frameB = frame;
			frameInWorld = trB * frameB;
			frameA = (trA.inverse() * frameInWorld).asTransform();
			break;
		case FrameInLerp:
			{
				auto trans = trA.getOrigin().lerp(trB.getOrigin(), frame.getOrigin().x());
				auto rot = trA.getBasis().slerp(trB.getBasis(), frame.getOrigin().y());
				frameInWorld = btQsTransform(rot, trans);
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		case AWithXPointToB:
			{
				btMatrix3x3 matr(trA.getBasis());
				frameInWorld = trA;
				auto old = matr.getColumn(0).normalized();
				auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
				auto q = rotFromAtoB(old, a2b);
				frameInWorld.getBasis() *= q;
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		case AWithYPointToB:
			{
				btMatrix3x3 matr(trA.getBasis());
				frameInWorld = trA;
				auto old = matr.getColumn(1).normalized();
				auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
				auto q = rotFromAtoB(old, a2b);
				frameInWorld.getBasis() *= q;
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		case AWithZPointToB:
			{
				btMatrix3x3 matr(trA.getBasis());
				frameInWorld = trA;
				auto old = matr.getColumn(2).normalized();
				auto a2b = (trB.getOrigin() - trA.getOrigin()).normalized();
				auto q = rotFromAtoB(old, a2b);
				frameInWorld.getBasis() *= q;
				frameA = (trA.inverse() * frameInWorld).asTransform();
				frameB = (trB.inverse() * frameInWorld).asTransform();
				break;
			}
		}
	}

	RE::BSTSmartPointer<Generic6DofConstraint> SkyrimSystemCreator::readGenericConstraint()
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA, *bodyB;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
			return nullptr;

		auto trA = bodyA->m_currentTransform;
		auto trB = bodyB->m_currentTransform;

		auto cinfo = getGenericConstraintTemplate(clsname);
		readGenericConstraintTemplate(cinfo);
		btTransform frameA, frameB;
		calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

		RE::BSTSmartPointer<Generic6DofConstraint> constraint;
		if (cinfo.useLinearReferenceFrameA) {
			constraint = RE::make_smart<Generic6DofConstraint>(bodyB, bodyA, frameB, frameA);
		} else {
			constraint = RE::make_smart<Generic6DofConstraint>(bodyA, bodyB, frameA, frameB);
		}

		constraint->setLinearLowerLimit(cinfo.linearLowerLimit);
		constraint->setLinearUpperLimit(cinfo.linearUpperLimit);
		constraint->setAngularLowerLimit(cinfo.angularLowerLimit);
		constraint->setAngularUpperLimit(cinfo.angularUpperLimit);
		for (int i = 0; i < 3; ++i) {
			constraint->setStiffness(i, cinfo.linearStiffness[i], cinfo.linearStiffnessLimited);
			constraint->setStiffness(i + 3, cinfo.angularStiffness[i], cinfo.angularStiffnessLimited);
			constraint->setDamping(i, cinfo.linearDamping[i], cinfo.springDampingLimited);
			constraint->setDamping(i + 3, cinfo.angularDamping[i], cinfo.springDampingLimited);

			constraint->setEquilibriumPoint(i, cinfo.linearEquilibrium[i]);
			constraint->setEquilibriumPoint(i + 3, cinfo.angularEquilibrium[i]);

			constraint->setNonHookeanDamping(i, cinfo.linearNonHookeanDamping[i]);
			constraint->setNonHookeanDamping(i + 3, cinfo.angularNonHookeanDamping[i]);
			constraint->setNonHookeanStiffness(i, cinfo.linearNonHookeanStiffness[i]);
			constraint->setNonHookeanStiffness(i + 3, cinfo.angularNonHookeanStiffness[i]);

			constraint->enableSpring(i, cinfo.enableLinearSprings);
			constraint->enableSpring(i + 3, cinfo.enableAngularSprings);

			constraint->enableMotor(i, cinfo.linearMotors);
			constraint->enableMotor(i + 3, cinfo.angularMotors);
			constraint->setServo(i, cinfo.linearServoMotors);
			constraint->setServo(i + 3, cinfo.angularServoMotors);
			// TODO: Test if servo motors go to [0, 0, 0], or whatever equilibrium is.  Provide option to set server motor target.  Hard coded to equilibrium right now.
			constraint->setServoTarget(i, cinfo.linearEquilibrium[i]);
			constraint->setServoTarget(i + 3, cinfo.angularEquilibrium[i]);
			constraint->setTargetVelocity(i, cinfo.linearTargetVelocity[i]);
			constraint->setTargetVelocity(i + 3, cinfo.angularTargetVelocity[i]);
			constraint->setMaxMotorForce(i, cinfo.linearMaxMotorForce[i]);
			constraint->setMaxMotorForce(i + 3, cinfo.angularMaxMotorForce[i]);

			constraint->setParam(BT_CONSTRAINT_ERP, cinfo.motorERP, i);
			constraint->setParam(BT_CONSTRAINT_CFM, cinfo.motorCFM, i);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP, cinfo.stopERP, i);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM, cinfo.stopCFM, i);

			auto rotMotor = constraint->getRotationalLimitMotor(i);
			if (rotMotor) {
				rotMotor->m_motorERP = cinfo.motorERP;
				rotMotor->m_motorCFM = cinfo.motorCFM;
				rotMotor->m_stopERP = cinfo.stopERP;
				rotMotor->m_stopCFM = cinfo.stopCFM;
			}
		}
		constraint->getTranslationalLimitMotor()->m_bounce = cinfo.linearBounce;
		constraint->getRotationalLimitMotor(0)->m_bounce = cinfo.angularBounce[0];
		constraint->getRotationalLimitMotor(1)->m_bounce = cinfo.angularBounce[1];
		constraint->getRotationalLimitMotor(2)->m_bounce = cinfo.angularBounce[2];
		/*constraint->getTranslationalLimitMotor()->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(0)->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(1)->m_limitSoftness = 1;
		constraint->getRotationalLimitMotor(2)->m_limitSoftness = 1;*/

		return constraint;
	}

	void SkyrimSystemCreator::readStiffSpringConstraintTemplate(StiffSpringConstraintTemplate& dest)
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (name == "minDistanceFactor")
					dest.minDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "maxDistanceFactor")
					dest.maxDistanceFactor = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "stiffness")
					dest.stiffness = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "damping")
					dest.damping = std::max(m_reader->readFloat(), 0.0f);
				else if (name == "equilibrium")
					dest.equilibriumFactor = btClamped(m_reader->readFloat(), 0.0f, 1.0f);
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	void SkyrimSystemCreator::readConeTwistConstraintTemplate(ConeTwistConstraintTemplate& dest)
	{
		while (m_reader->Inspect()) {
			if (m_reader->GetInspected() == XMLReader::Inspected::StartTag) {
				auto name = m_reader->GetName();
				if (parseFrameType(name, dest.frameType, dest.frame))
					;
				else if (name == "swingSpan1" || name == "coneLimit" || name == "limitZ")
					dest.swingSpan1 = std::max(m_reader->readFloat(), 0.f);
				else if (name == "swingSpan2" || name == "planeLimit" || name == "limitY")
					dest.swingSpan2 = std::max(m_reader->readFloat(), 0.f);
				else if (name == "twistSpan" || name == "twistLimit" || name == "limitX")
					dest.twistSpan = std::max(m_reader->readFloat(), 0.f);
				else if (name == "limitSoftness")
					dest.limitSoftness = btClamped(m_reader->readFloat(), 0.f, 1.f);
				else if (name == "biasFactor")
					dest.biasFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
				else if (name == "relaxationFactor")
					dest.relaxationFactor = btClamped(m_reader->readFloat(), 0.f, 1.f);
				else {
					logger::warn("unknown element - {}", name.c_str());
					m_reader->skipCurrentElement();
				}
			} else if (m_reader->GetInspected() == XMLReader::Inspected::EndTag)
				break;
		}
	}

	const SkyrimSystemCreator::BoneTemplate& SkyrimSystemCreator::getBoneTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_boneTemplates.find(name);
		if (iter == m_boneTemplates.end())
			return m_boneTemplates[RE::BSFixedString()];
		return iter->second;
	}

	const SkyrimSystemCreator::GenericConstraintTemplate& SkyrimSystemCreator::getGenericConstraintTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_genericConstraintTemplates.find(name);
		if (iter == m_genericConstraintTemplates.end())
			return m_genericConstraintTemplates[RE::BSFixedString()];
		return iter->second;
	}

	const SkyrimSystemCreator::StiffSpringConstraintTemplate& SkyrimSystemCreator::getStiffSpringConstraintTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_stiffSpringConstraintTemplates.find(name);
		if (iter == m_stiffSpringConstraintTemplates.end())
			return m_stiffSpringConstraintTemplates[RE::BSFixedString()];
		return iter->second;
	}

	const SkyrimSystemCreator::ConeTwistConstraintTemplate& SkyrimSystemCreator::getConeTwistConstraintTemplate(const RE::BSFixedString& name)
	{
		auto iter = m_coneTwistConstraintTemplates.find(name);
		if (iter == m_coneTwistConstraintTemplates.end())
			return m_coneTwistConstraintTemplates[RE::BSFixedString()];
		return iter->second;
	}

	RE::BSTSmartPointer<StiffSpringConstraint> SkyrimSystemCreator::readStiffSpringConstraint()
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA, *bodyB;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB))
			return nullptr;

		StiffSpringConstraintTemplate cinfo = getStiffSpringConstraintTemplate(clsname);
		readStiffSpringConstraintTemplate(cinfo);

		RE::BSTSmartPointer<StiffSpringConstraint> constraint = RE::make_smart<StiffSpringConstraint>(bodyA, bodyB);
		constraint->m_minDistance *= cinfo.minDistanceFactor;
		constraint->m_maxDistance *= cinfo.maxDistanceFactor;
		constraint->m_stiffness = cinfo.stiffness;
		constraint->m_damping = cinfo.damping;
		constraint->m_equilibriumPoint = constraint->m_minDistance * cinfo.equilibriumFactor + constraint->m_maxDistance * (1 - cinfo.equilibriumFactor);
		return constraint;
	}

	RE::BSTSmartPointer<ConeTwistConstraint> SkyrimSystemCreator::readConeTwistConstraint()
	{
		auto bodyAName = getRenamedBone(m_reader->getAttribute("bodyA"));
		auto bodyBName = getRenamedBone(m_reader->getAttribute("bodyB"));
		auto clsname = m_reader->getAttribute("template", "");

		SkyrimBone *bodyA = nullptr, *bodyB = nullptr;
		if (!findBones(bodyAName, bodyBName, bodyA, bodyB)) {
			return nullptr;
		}

		auto trA = bodyA->m_currentTransform;
		auto trB = bodyB->m_currentTransform;

		auto cinfo = getConeTwistConstraintTemplate(clsname);
		readConeTwistConstraintTemplate(cinfo);
		btTransform frameA, frameB;
		calcFrame(cinfo.frameType, cinfo.frame, trA, trB, frameA, frameB);

		RE::BSTSmartPointer<ConeTwistConstraint> constraint = RE::make_smart<ConeTwistConstraint>(bodyA, bodyB, frameA, frameB);
		constraint->setLimit(cinfo.swingSpan1, cinfo.swingSpan2, cinfo.twistSpan, cinfo.limitSoftness, cinfo.biasFactor, cinfo.relaxationFactor);

		return constraint;
	}
}
