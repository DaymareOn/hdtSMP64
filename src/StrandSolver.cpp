#include "StrandSolver.h"

#include <algorithm>

namespace hdt
{
	namespace
	{
		/**
		 * Push a point out to the surface of a capsule if it is inside it.
		 *
		 * A capsule is a line segment (p0..p1) grown by a radius. We find the closest point
		 * on that core segment (clamping the projection so the ends stay rounded), and if the
		 * point is nearer than the radius we shove it straight out along the surface normal.
		 * The degenerate case (point exactly on the axis, no normal direction) pushes straight
		 * up so a hair bead never gets pinned inside the core.
		 */
		void projectOutOfCapsule(btVector3& pt, const StrandCollider& c)
		{
			const btVector3 ab = c.p1 - c.p0;
			const btScalar abLen2 = ab.length2();
			const btScalar t = abLen2 > SIMD_EPSILON ?
			                       std::clamp(ab.dot(pt - c.p0) / abLen2, btScalar(0), btScalar(1)) :
			                       btScalar(0);
			const btVector3 closest = c.p0 + ab * t;
			const btVector3 d = pt - closest;
			const btScalar dist = d.length();
			if (dist < c.radius) {
				if (dist > SIMD_EPSILON)
					pt = closest + d * (c.radius / dist);
				else
					pt = closest + btVector3(0, 0, c.radius);
			}
		}
	}

	void StrandSolver::init(std::uint32_t numStrands, std::uint32_t vertsPerStrand,
		const std::vector<btVector3>& restLocal)
	{
		m_numStrands = numStrands;
		m_vps = vertsPerStrand;
		m_restLocal = restLocal;

		const size_t n = static_cast<size_t>(numStrands) * vertsPerStrand;
		// Precompute each bead's rest distance to its parent (index 0 of each strand is the
		// pinned root and keeps a rest length of 0). This is the target the FTL pass enforces.
		m_restLen.assign(n, btScalar(0));
		for (std::uint32_t s = 0; s < numStrands; ++s) {
			for (std::uint32_t v = 1; v < vertsPerStrand; ++v) {
				const size_t i = static_cast<size_t>(s) * vertsPerStrand + v;
				m_restLen[i] = (m_restLocal[i] - m_restLocal[i - 1]).length();
			}
		}
		m_pos.assign(n, btVector3(0, 0, 0));
		m_prevPos.assign(n, btVector3(0, 0, 0));
		m_initialized = n > 0 && vertsPerStrand >= 2 && m_restLocal.size() == n;
		m_warm = false;
	}

	void StrandSolver::step(const btTransform& rootTransform, btScalar dt,
		const StrandParams& p, const std::vector<StrandCollider>& colliders)
	{
		if (!m_initialized || dt <= btScalar(0))
			return;

		const std::uint32_t vps = m_vps;
		const btVector3 accel = p.gravity + p.wind;
		const btScalar dt2 = dt * dt;

		// Cold start: place the whole groom on its rest pose so a first frame with empty
		// previous-positions cannot fling beads to infinity.
		if (!m_warm) {
			for (size_t i = 0; i < m_pos.size(); ++i) {
				m_pos[i] = rootTransform(m_restLocal[i]);
				m_prevPos[i] = m_pos[i];
			}
			m_warm = true;
		}

		for (std::uint32_t s = 0; s < m_numStrands; ++s) {
			const size_t base = static_cast<size_t>(s) * vps;

			// The root bead is glued to the scalp: it teleports with the head every substep,
			// and its motion is what drags the rest of the chain through the constraints below.
			m_pos[base] = rootTransform(m_restLocal[base]);
			m_prevPos[base] = m_pos[base];

			// 1. Verlet integration of the free beads (carry momentum, add gravity + wind).
			for (std::uint32_t v = 1; v < vps; ++v) {
				const size_t i = base + v;
				const btVector3 vel = (m_pos[i] - m_prevPos[i]) * p.damping;
				const btVector3 pred = m_pos[i] + vel + accel * dt2;
				m_prevPos[i] = m_pos[i];
				m_pos[i] = pred;
			}

			// 2. Shape pull: nudge each bead toward where the hairstyle says it should be
			//    (rest pose carried by the current head pose). This gives the hair volume and
			//    stops gravity collapsing it into a straight rope.
			if (p.globalStiffness > btScalar(0)) {
				for (std::uint32_t v = 1; v < vps; ++v) {
					const size_t i = base + v;
					const btVector3 target = rootTransform(m_restLocal[i]);
					m_pos[i] += (target - m_pos[i]) * p.globalStiffness;
				}
			}

			// 3. Follow-The-Leader inextensibility: walk root->tip and snap each bead back to
			//    its exact rest distance from its parent. O(n) per strand, never stretches.
			for (std::uint32_t v = 1; v < vps; ++v) {
				const size_t i = base + v;
				btVector3 dir = m_pos[i] - m_pos[i - 1];
				const btScalar len = dir.length();
				if (len > SIMD_EPSILON)
					m_pos[i] = m_pos[i - 1] + dir * (m_restLen[i] / len);
			}

			// 4. Keep the free beads out of the body capsules.
			if (!colliders.empty()) {
				for (std::uint32_t v = 1; v < vps; ++v) {
					const size_t i = base + v;
					for (const auto& c : colliders)
						projectOutOfCapsule(m_pos[i], c);
				}
			}
		}
	}
}
