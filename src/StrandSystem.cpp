#include "StrandSystem.h"

#include "NetImmerseUtils.h"
#include "hdtConvertNi.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <unordered_set>

namespace hdt
{
	bool loadTfx(const std::string& path, StrandGroom& out)
	{
		// TressFX .tfx layout: a fixed 160-byte header, then numStrands*numVerts float4 positions
		// at offsetVertexPosition (xyz = position, w = inverse mass). We only need positions for
		// the PoC. Everything crossing this trust boundary is validated before use; on any
		// malformed field we fail closed and leave `out` untouched.
		struct TfxHeader
		{
			float version;
			std::uint32_t numStrands;
			std::uint32_t numVerts;
			std::uint32_t offsetPos;
			std::uint32_t offsetStrandUV;
			std::uint32_t offsetVertUV;
			std::uint32_t offsetThickness;
			std::uint32_t offsetColor;
			std::uint32_t reserved[32];
		};
		static_assert(sizeof(TfxHeader) == 160, "TressFX header must be 160 bytes");

		static constexpr std::uint64_t kMaxVerts = 4'000'000;  // ~64 MB of positions; ample for any wig

		std::ifstream f(path, std::ios::binary);
		if (!f)
			return false;

		TfxHeader h{};
		f.read(reinterpret_cast<char*>(&h), sizeof(h));
		if (!f)
			return false;
		if (h.numStrands == 0 || h.numStrands > 1'000'000)
			return false;
		if (!(h.numVerts == 4 || h.numVerts == 8 || h.numVerts == 16 || h.numVerts == 32 || h.numVerts == 64))
			return false;
		if (h.offsetPos < sizeof(TfxHeader))
			return false;

		const std::uint64_t n = static_cast<std::uint64_t>(h.numStrands) * h.numVerts;
		if (n > kMaxVerts)
			return false;

		// Validate the whole position block fits in the file BEFORE allocating/reading, rather than
		// discovering a short/truncated file mid-read.
		f.seekg(0, std::ios::end);
		const std::streamoff fileSize = f.tellg();
		if (fileSize < 0)
			return false;
		const std::uint64_t need = static_cast<std::uint64_t>(h.offsetPos) + n * 16u;  // float4 per vertex
		if (need > static_cast<std::uint64_t>(fileSize))
			return false;

		std::vector<float> raw(static_cast<size_t>(n) * 4);
		f.seekg(static_cast<std::streamoff>(h.offsetPos), std::ios::beg);
		f.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float)));
		if (!f)
			return false;

		std::vector<btVector3> rest;
		rest.reserve(static_cast<size_t>(n));
		for (size_t i = 0; i < static_cast<size_t>(n); ++i)
			rest.emplace_back(raw[i * 4 + 0], raw[i * 4 + 1], raw[i * 4 + 2]);  // xyz; w (inverse mass) unused

		out.strandCount = h.numStrands;
		out.vertsPerStrand = h.numVerts;
		out.restLocal = std::move(rest);
		return true;
	}

	// ---------------------------------------------------------------- StrandInstance

	bool StrandInstance::buildScalpGroom(RE::NiNode* headBone, const StrandConfig& config, StrandGroom& out)
	{
		// The head's true centre + radius in world space, from the real head geometry bounds.
		// The head bone (confirmed via the bind log to be the player's head) is the anchor. The scalp
		// sits ~10 units above the bone origin along world-up; use fixed head-sized dimensions -- the
		// merged geometry bound proved unreliable/inflated on the physics worker thread.
		const float R = 8.0f;  // scalp cap radius (head half-width)
		const RE::NiPoint3 C = headBone->world.translate + RE::NiPoint3(0.0f, 0.0f, 10.0f);
		const RE::NiTransform headInv = headBone->world.Invert();

		// Deterministic per-strand pseudo-random in [0,1): gives length/direction variation so the
		// groom reads as a hair mass rather than a uniform helmet, with no stateful RNG.
		const auto hash = [](std::uint32_t n, float k) {
			float v = std::sin(static_cast<float>(n) * k) * 43758.5453f;
			return v - std::floor(v);
		};

		StrandGroom g;
		g.strandCount = static_cast<std::uint32_t>(config.strandCount);
		g.vertsPerStrand = static_cast<std::uint32_t>(config.vertsPerStrand);
		const float baseLength = config.length;
		const float goldenAngle = 2.399963f;
		g.restLocal.reserve(static_cast<size_t>(g.strandCount) * g.vertsPerStrand);

		for (std::uint32_t s = 0; s < g.strandCount; ++s) {
			// Fibonacci point biased to the crown + upper sides (zf in [0.15,1]); the lowest ring
			// near face level is skipped so strands don't sprout from the forehead.
			const float t = (static_cast<float>(s) + 0.5f) / static_cast<float>(g.strandCount);
			const float zf = 0.15f + 0.85f * t;
			const float ring = std::sqrt(std::max(0.0f, 1.0f - zf * zf));
			const float phi = static_cast<float>(s) * goldenAngle;
			const RE::NiPoint3 dirW(ring * std::cos(phi), ring * std::sin(phi), zf);

			// Rest pose drapes down-and-out with per-strand jitter; gravity + the head-sphere
			// collider refine it at runtime.
			const float h1 = hash(s, 12.9898f);
			const float h2 = hash(s, 78.233f);
			const RE::NiPoint3 jitter((h1 - 0.5f) * 0.4f, (h2 - 0.5f) * 0.4f, 0.0f);
			RE::NiPoint3 growW = dirW * 0.35f + jitter - RE::NiPoint3(0.0f, 0.0f, 1.0f) * 0.65f;
			const float gl = growW.Length();
			if (gl > 1e-4f)
				growW = growW * (1.0f / gl);

			const float len = baseLength * (1.0f - config.lengthVariation + 2.0f * config.lengthVariation * h1);
			const float seg = len / static_cast<float>(g.vertsPerStrand - 1);

			RE::NiPoint3 pW = C + dirW * (R * 0.98f);  // root sits on the scalp surface
			for (std::uint32_t v = 0; v < g.vertsPerStrand; ++v) {
				g.restLocal.push_back(convertNi(headInv * pW));  // head-local so it follows the head
				pW = pW + growW * seg;
			}
		}
		out = std::move(g);
		return true;
	}

	StrandInstance::StrandInstance(const StrandConfig& config, std::shared_ptr<const StrandGroom> authoredGroom) :
		m_config(config), m_authoredGroom(std::move(authoredGroom))
	{
		// Gravity is expressed in Skyrim units/s^2 to match SMP's own world (which uses
		// -9.8 * scaleSkyrim); the solver otherwise works entirely in raw Skyrim units. The solver
		// is primed later in bind(), once the groom (authored or procedural) is chosen.
		m_params.gravity = btVector3(0, 0, -9.8f * scaleSkyrim);
		m_params.globalStiffness = config.stiffness;  // hold the styled shape; less wet-noodle droop
		m_params.damping = config.damping;
	}

	bool StrandInstance::bind(RE::NiNode* skeletonRoot)
	{
		if (!skeletonRoot)
			return false;
		// skeletonRoot is one actor's own skeleton (from the SMP system list), so this lookup is
		// unambiguous -- it can only resolve to THIS actor's head. No player identification needed.
		RE::NiNode* head = findNodeSafe(skeletonRoot, "NPC Head [Head]");
		if (!head)
			return false;

		if (m_authoredGroom) {
			m_groom = *m_authoredGroom;  // authored .tfx geometry (shared, immutable) copied per instance
			// Authored positions are world-up-axis offsets from the head origin (Skyrim units). Convert
			// to head-local the same way the procedural cap does, so the wig follows the head and sits
			// right whatever the head bone's orientation -- the head rotation cancels at render time.
			const RE::NiTransform headInv = head->world.Invert();
			for (auto& p : m_groom.restLocal) {
				const RE::NiPoint3 pW = head->world.translate + RE::NiPoint3(p.x(), p.y(), p.z());
				p = convertNi(headInv * pW);
			}
		} else {
			StrandGroom scalp;
			if (!buildScalpGroom(head, m_config, scalp))
				return false;  // head geometry not ready yet; caller retries next frame
			m_groom = std::move(scalp);
		}
		// Render material always comes from this actor's config, never the shared groom geometry.
		m_groom.colorRoot = { m_config.colorRoot[0], m_config.colorRoot[1], m_config.colorRoot[2] };
		m_groom.colorTip = { m_config.colorTip[0], m_config.colorTip[1], m_config.colorTip[2] };
		m_groom.strandRadius = m_config.width;

		m_anchor = make_nismart(head);
		m_solver.init(m_groom.strandCount, m_groom.vertsPerStrand, m_groom.restLocal);
		m_colliders.clear();

		// Minimal body proxy so hair doesn't sink into the skull/shoulders. Radii are Skyrim-unit
		// guesses to be tuned; a head sphere, a neck capsule, and two clavicle spheres.
		const auto addCollider = [&](const char* a, const char* b, btScalar radius) {
			RE::NiNode* na = findNodeSafe(skeletonRoot, a);
			if (!na)
				return;
			RE::NiNode* nb = b ? findNodeSafe(skeletonRoot, b) : nullptr;
			m_colliders.push_back({ make_nismart(na), nb ? make_nismart(nb) : RE::NiPointer<RE::NiNode>(), radius });
		};
		addCollider("NPC Head [Head]", nullptr, 11.0f);
		addCollider("NPC Head [Head]", "NPC Spine2 [Spn2]", 8.0f);
		addCollider("NPC L Clavicle [LClv]", nullptr, 7.0f);
		addCollider("NPC R Clavicle [RClv]", nullptr, 7.0f);
		return true;
	}

	void StrandInstance::step(btScalar totalDt, btScalar tick, std::mutex& publishLock)
	{
		if (!m_anchor)
			return;
		// If the head node detached (cell change / actor reload) its NiPointer keeps it alive but
		// it's no longer in the live scene graph -- drop it so the manager rebinds to fresh 3D.
		if (!m_anchor->parent) {
			m_anchor.reset();
			return;
		}

		const btTransform root = convertNi(m_anchor->world).asTransform();

		std::vector<StrandCollider> cols;
		cols.reserve(m_colliders.size());
		for (const auto& cd : m_colliders) {
			if (!cd.a)
				continue;
			const btVector3 pa = convertNi(cd.a->world.translate);
			const btVector3 pb = cd.b ? convertNi(cd.b->world.translate) : pa;
			cols.push_back({ pa, pb, cd.radius });
		}

		// Match SMP's fixed-tick substepping so the hair is stable regardless of frame time.
		int n = static_cast<int>(std::ceil(totalDt / tick));
		n = std::clamp(n, 1, 8);
		const btScalar dt = totalDt / static_cast<btScalar>(n);
		for (int k = 0; k < n; ++k)
			m_solver.step(root, dt, m_params, cols);

		std::lock_guard<std::mutex> lock(publishLock);
		m_published = m_solver.positions();
		m_lastHeadWorld = root;
	}

	void StrandInstance::fillDesc(FSMPWigInstanceDesc& d, std::uint32_t id) const
	{
		d.instanceId = id;
		d.strandCount = m_groom.strandCount;
		d.vertsPerStrand = m_groom.vertsPerStrand;

		const btMatrix3x3& b = m_lastHeadWorld.getBasis();
		const btVector3& o = m_lastHeadWorld.getOrigin();
		for (int r = 0; r < 3; ++r) {
			d.headWorld[r * 4 + 0] = b[r].x();
			d.headWorld[r * 4 + 1] = b[r].y();
			d.headWorld[r * 4 + 2] = b[r].z();
			d.headWorld[r * 4 + 3] = o[r];
		}
		d.headWorld[12] = d.headWorld[13] = d.headWorld[14] = 0.0f;
		d.headWorld[15] = 1.0f;

		for (int i = 0; i < 3; ++i) {
			d.colorRoot[i] = m_groom.colorRoot[i];
			d.colorTip[i] = m_groom.colorTip[i];
		}
		d.roughness = m_groom.roughness;
		d.strandRadius = m_groom.strandRadius;
	}

	std::uint32_t StrandInstance::copyPositions(float* dst, std::uint32_t capacityFloats) const
	{
		const size_t beads = std::min<size_t>(m_published.size(), capacityFloats / 3);
		for (size_t i = 0; i < beads; ++i) {
			dst[i * 3 + 0] = m_published[i].x();
			dst[i * 3 + 1] = m_published[i].y();
			dst[i * 3 + 2] = m_published[i].z();
		}
		return static_cast<std::uint32_t>(beads * 3);
	}

	// ---------------------------------------------------------------- StrandManager

	StrandManager::StrandManager()
	{
		// A no-code toggle for quick in-game testing without a renderer: drop an empty file at
		// Data/SKSE/Plugins/FSMPWig/enable.txt and the simulation runs on load.
		std::error_code ec;
		if (std::filesystem::exists("Data/SKSE/Plugins/FSMPWig/enable.txt", ec))
			m_enabled = true;
		loadStrandConfig("Data/SKSE/Plugins/FSMPWig/wig.xml", m_config);
	}

	StrandManager& StrandManager::instance()
	{
		static StrandManager s;
		return s;
	}

	// Path of a per-id config: 8-uppercase-hex formID, e.g. wigs/00000014.xml for the player (0x14).
	static std::string wigConfigPath(std::uint32_t id)
	{
		char buf[64];
		std::snprintf(buf, sizeof(buf), "Data/SKSE/Plugins/FSMPWig/wigs/%08X.xml", id);
		return std::string(buf);
	}

	const StrandConfig* StrandManager::loadCachedConfig(std::uint32_t id)
	{
		if (id == 0)
			return nullptr;
		if (auto it = m_configCache.find(id); it != m_configCache.end())
			return it->second ? &*it->second : nullptr;
		StrandConfig cfg = m_config;  // inherit the global config; the file overrides only its own tags
		const bool found = loadStrandConfig(wigConfigPath(id), cfg);
		auto& slot = m_configCache[id];
		if (found)
			slot = cfg;
		else
			slot = std::nullopt;  // remember the miss so we don't re-stat the file each frame
		return found ? &*slot : nullptr;
	}

	const StrandConfig& StrandManager::configFor(std::uint32_t actorFormID, std::uint32_t wigFormID)
	{
		if (const StrandConfig* c = loadCachedConfig(wigFormID))
			return *c;
		if (const StrandConfig* c = loadCachedConfig(actorFormID))
			return *c;
		return m_config;
	}

	std::shared_ptr<const StrandGroom> StrandManager::loadCachedGroom(const StrandConfig& cfg)
	{
		if (cfg.groomFile.empty())
			return nullptr;
		// Same .tfx at a different scale is a distinct groom, so the scale is part of the key.
		char scaleKey[32];
		std::snprintf(scaleKey, sizeof(scaleKey), "%.4f", cfg.groomScale);
		const std::string key = cfg.groomFile + "|" + scaleKey;
		if (auto it = m_groomCache.find(key); it != m_groomCache.end())
			return it->second;

		std::shared_ptr<const StrandGroom> result;  // stays null if load fails
		auto groom = std::make_shared<StrandGroom>();
		const std::string path = "Data/SKSE/Plugins/FSMPWig/grooms/" + cfg.groomFile;
		if (loadTfx(path, *groom) && groom->strandCount > 0 && groom->vertsPerStrand >= 2) {
			if (cfg.groomScale != 1.0f)
				for (auto& p : groom->restLocal)
					p *= cfg.groomScale;
			result = groom;
		}
		m_groomCache[key] = result;  // cache the miss too, so a bad/missing file isn't re-read each frame
		return result;
	}

	void StrandManager::step(btScalar totalDt, btScalar tick, const std::vector<StrandActor>& actors)
	{
		if (m_resetRequested.exchange(false)) {
			std::lock_guard<std::mutex> lock(m_publishLock);
			m_instances.clear();
			m_order.clear();
			// Re-read configs so editing wig.xml / a per-wig file + reloading a save applies without
			// a restart.
			m_config = StrandConfig{};
			loadStrandConfig("Data/SKSE/Plugins/FSMPWig/wig.xml", m_config);
			m_configCache.clear();
			m_groomCache.clear();
		}
		if (!m_enabled)
			return;

		// Create a wig for any newly-seen qualifying actor (bind confined to that actor's own subtree).
		// An actor qualifies when it wears a wig-slot armor, or has an explicit per-actor config file,
		// or the global policy attaches to everyone. Config is chosen by worn wig, then per-actor, then
		// the global default. Multiple SMP systems can share one skeleton root; the map de-dupes them.
		std::unordered_set<RE::NiNode*> present;
		present.reserve(actors.size());
		for (const auto& a : actors) {
			if (!a.root)
				continue;
			const StrandConfig* actorCfg = loadCachedConfig(a.actorFormID);
			const bool qualifies = a.wigFormID != 0 || actorCfg != nullptr || !m_config.attachToWigArmorOnly;
			if (!qualifies)
				continue;
			present.insert(a.root);
			if (auto it = m_instances.find(a.root); it != m_instances.end()) {
				// An instance that lost its head node (cell reload) must re-bind to the fresh 3D.
				if (!it->second->bound())
					it->second->bind(a.root);
				continue;
			}
			const StrandConfig& cfg = configFor(a.actorFormID, a.wigFormID);
			auto inst = std::make_unique<StrandInstance>(cfg, loadCachedGroom(cfg));
			if (!inst->bind(a.root))
				continue;  // head not ready yet; retry next frame
			std::lock_guard<std::mutex> lock(m_publishLock);
			m_instances.emplace(a.root, std::move(inst));
		}

		// Drop wigs whose actor is gone, then rebuild the render-side index order -- both under the
		// publish lock so the render thread never sees a half-updated set.
		{
			std::lock_guard<std::mutex> lock(m_publishLock);
			for (auto it = m_instances.begin(); it != m_instances.end();) {
				if (present.find(it->first) == present.end())
					it = m_instances.erase(it);
				else
					++it;
			}
			m_order.clear();
			m_order.reserve(m_instances.size());
			for (auto& kv : m_instances)
				m_order.push_back(kv.second.get());
		}

		// Step every wig. The solve runs outside the publish lock; each instance takes it only to
		// swap in its fresh position snapshot.
		for (auto& kv : m_instances)
			kv.second->step(totalDt, tick, m_publishLock);
	}

	std::uint32_t StrandManager::instanceCount()
	{
		std::lock_guard<std::mutex> lock(m_publishLock);
		return static_cast<std::uint32_t>(m_order.size());
	}

	std::uint32_t StrandManager::getDesc(std::uint32_t index, FSMPWigInstanceDesc* out)
	{
		std::lock_guard<std::mutex> lock(m_publishLock);
		if (index >= m_order.size())
			return 0;
		m_order[index]->fillDesc(*out, index);
		return 1;
	}

	std::uint32_t StrandManager::copyPositions(std::uint32_t index, float* dst, std::uint32_t capacityFloats)
	{
		std::lock_guard<std::mutex> lock(m_publishLock);
		if (index >= m_order.size())
			return 0;
		return m_order[index]->copyPositions(dst, capacityFloats);
	}
}
