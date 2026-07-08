#pragma once

#include <LinearMath/btTransform.h>
#include <cstdint>
#include <vector>

namespace hdt
{
	/// A capsule (or sphere when p0 == p1) that hair vertices are pushed out of.
	/// Endpoints are in the same world space as the solver's positions.
	struct StrandCollider
	{
		btVector3 p0;
		btVector3 p1;
		btScalar radius;
	};

	/// Tuning for one solve. All stiffnesses are blend factors in [0,1] applied
	/// once per substep; gravity/wind are accelerations in the solver's world units.
	struct StrandParams
	{
		btScalar globalStiffness = 0.15f;  // pull back toward the rest hairstyle (keeps the shape)
		btScalar damping = 0.92f;          // velocity retained each substep (1 = no damping)
		btVector3 gravity = btVector3(0, 0, -9.8f);
		btVector3 wind = btVector3(0, 0, 0);
	};

	/**
	 * CPU guide-strand solver for the wig proof-of-concept.
	 *
	 * The mental model: each strand is a little chain of beads. Bead 0 is glued to the
	 * scalp and rides wherever the head bone goes; the rest of the beads are free and want
	 * to fall, but they must stay strung the correct distance apart and roughly keep the
	 * original hairstyle. Every substep we do four cheap passes:
	 *   1. Integrate  - move each free bead by its own momentum (Verlet) plus gravity/wind.
	 *   2. Shape pull - nudge each bead a little toward where the hairstyle says it should be
	 *                   (the rest pose carried by the current head transform); this is what
	 *                   stops the hair collapsing into a straight wet-noodle under gravity.
	 *   3. FTL length - walk root->tip and snap each bead back to its exact rest distance
	 *                   from its parent. Follow-The-Leader is O(n) and perfectly inextensible,
	 *                   so strands never stretch no matter how hard the head yanks them.
	 *   4. Collision  - push any bead that ended up inside a body capsule back to its surface.
	 *
	 * Momentum falls out of the Verlet formulation for free: because bead 0 teleports with
	 * the head each substep while the free beads lag, the chain naturally swings and settles.
	 *
	 * All positions live in the physics world space FSMP already uses; the caller supplies the
	 * root transform (head-local -> world) and the collider list in that same space.
	 */
	class StrandSolver
	{
	public:
		/**
		 * Bind the solver to a groom.
		 * @param numStrands       how many guide strands.
		 * @param vertsPerStrand   beads per strand (constant across the groom, TressFX-style).
		 * @param restLocal        head-local rest position of every bead, strand-major
		 *                         (strand s, bead v at index s*vertsPerStrand + v).
		 *
		 * We cache the rest positions (for the shape-pull target) and precompute each bead's
		 * rest distance to its parent (for the FTL length pass). Bead 0 of every strand is the
		 * pinned root. The first solve seeds world positions from an identity-less bind by
		 * mapping restLocal through the first step()'s transform.
		 */
		void init(std::uint32_t numStrands, std::uint32_t vertsPerStrand,
			const std::vector<btVector3>& restLocal);

		/**
		 * Advance the whole groom by one substep.
		 * @param rootTransform maps a bead's head-local rest position into world space; changes
		 *                      every frame as the actor's head moves, and is what drags the hair.
		 * @param dt            substep length in seconds (FSMP uses 1/60 with up to 4 substeps).
		 * @param p             tuning for this solve.
		 * @param colliders     body capsules in world space to keep hair out of.
		 *
		 * On the very first call (or after re-init) world positions are cold, so we snap the
		 * whole groom onto the rest pose transformed by rootTransform before integrating,
		 * which avoids a one-frame explosion from garbage previous-positions.
		 */
		void step(const btTransform& rootTransform, btScalar dt,
			const StrandParams& p, const std::vector<StrandCollider>& colliders);

		/// World-space positions of every bead, same strand-major layout as restLocal.
		const std::vector<btVector3>& positions() const { return m_pos; }
		std::uint32_t strandCount() const { return m_numStrands; }
		std::uint32_t vertsPerStrand() const { return m_vps; }
		bool initialized() const { return m_initialized; }

	private:
		std::uint32_t m_numStrands = 0;
		std::uint32_t m_vps = 0;
		std::vector<btVector3> m_restLocal;  // head-local, per bead
		std::vector<btScalar> m_restLen;     // distance to parent bead; index%vps==0 is 0 (root)
		std::vector<btVector3> m_pos;        // world space, current
		std::vector<btVector3> m_prevPos;    // world space, previous substep (for Verlet)
		bool m_initialized = false;          // false until init()
		bool m_warm = false;                 // false until the first step seeds world positions
	};
}
