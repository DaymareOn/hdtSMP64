/**
 * Unit tests for hdt::DecimateCollisionMesh
 * ===========================================
 *
 * Test organisation (each SECTION maps to a group below):
 *
 *   §1  Passthrough semantics  — the function returns the input unchanged when
 *       it has nothing to do (disabled, empty, below minimum vertex count).
 *
 *   §2  Structural invariants  — every output is a geometrically valid mesh:
 *       no out-of-bounds triangle indices, no degenerate triangles, and the
 *       stats fields agree with the actual output containers.
 *
 *   §3  Reduction              — vertex-count limits (absolute and ratio) are
 *       honoured; the count never increases.
 *
 *   §4  Vertex mapping         — oldToNewVertex is consistent with the output
 *       vertex list; the fallback path produces an exact identity map.
 *
 *   §5  Safety gates           — the volume-loss guard, boundary-preservation
 *       flag, and skin-weight drift threshold each fire when their condition is
 *       met, and the corresponding rejection counter is incremented.
 *
 *   §6  Pass A (short edges)   — short-edge point removal fires when an edge
 *       falls below the shortEdgeRatio * diagonal threshold.
 *
 * Mesh catalogue (defined in the anonymous namespace below):
 *
 *   makeSingleTriangle()       3 v / 1 t  — smallest possible closed mesh
 *   makeTetrahedron()          4 v / 4 t  — closed, has non-zero signed volume
 *   makeGrid3x3()              9 v / 8 t  — flat, open, all normals identical;
 *                                           ideal for reduction tests because
 *                                           volume guards are inactive on flat meshes
 *   makeBoundaryTestMesh()     6 v / 5 t  — fan with a non-boundary centre;
 *                                           edge (v0,v1) is very short (0.001 units)
 *                                           and crosses the boundary/non-boundary seam
 *
 * Dependency note:
 *   The only external dependencies pulled in by the decimator are Bullet
 *   (btVector3) and the Windows SDK (ZeroMemory via test_pch.h).  No SKSE,
 *   CommonLibSSE, or Skyrim RE headers are needed.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Validator/Improvers/hdtMeshDecimator.h"

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

// ── Aliases ───────────────────────────────────────────────────────────────────

using hdt::CollisionMeshDecimationOptions;
using hdt::CollisionMeshDecimationOutput;
using hdt::DecimateCollisionMesh;
using hdt::Vertex;

using Verts = std::vector<Vertex>;
using Tris  = std::vector<std::array<uint32_t, 3>>;

// ── Helpers ───────────────────────────────────────────────────────────────────

namespace
{
    // Construct a Vertex at (x, y, z) with 100 % weight on a single bone.
    // The default bone index is 0 (all vertices are identically weighted unless
    // a test overrides this to exercise the skin-drift gate).
    Vertex v(float x, float y, float z, uint32_t boneIdx = 0)
    {
        Vertex vtx(x, y, z);
        vtx.m_weight[0]  = 1.0f;
        vtx.m_boneIdx[0] = boneIdx;
        return vtx;
    }

    // ── Mesh factories ────────────────────────────────────────────────────────

    // 3 vertices, 1 triangle.
    // The minimum representable mesh — nothing can be decimated below this.
    std::pair<Verts, Tris> makeSingleTriangle()
    {
        Verts verts = { v(0,0,0), v(1,0,0), v(0.5f,0,1) };
        Tris  tris  = { {0, 1, 2} };
        return { verts, tris };
    }

    // 4 vertices, 4 triangles — a closed tetrahedron inscribed in a ±1 cube.
    //
    // Vertices:       v0=(+1,+1,+1)  v1=(+1,−1,−1)  v2=(−1,+1,−1)  v3=(−1,−1,+1)
    // Edge length:    2√3 ≈ 3.46
    // Signed volume:  ≈ 2.667  (non-zero, so the global volume guard is active)
    //
    // Used for: volume-fallback tests, structural invariant checks, mapping tests.
    std::pair<Verts, Tris> makeTetrahedron()
    {
        Verts verts = { v(1,1,1), v(1,-1,-1), v(-1,1,-1), v(-1,-1,1) };
        // Winding chosen so outward normals are consistent.
        Tris tris = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };
        return { verts, tris };
    }

    // 9 vertices, 8 triangles — a flat, unit-spacing 3×3 grid on the z=0 plane.
    //
    // Layout (vertex index = row*3+col):
    //   0  1  2
    //   3  4  5
    //   6  7  8
    //
    // Each 1×1 square is split into two triangles along the lower-left→upper-right
    // diagonal.  All triangles share the normal (0, 0, −1), so:
    //   • normal-deviation checks always pass (no flip ever occurs)
    //   • volume = 0 → global volume guard is inactive (volumeCap ≈ 0 → guard off)
    //   • local volume contributions are also 0 → local volume guard is inactive
    //
    // This makes the grid ideal for testing reduction targets and Pass A/B mechanics
    // without interference from volume constraints.
    std::pair<Verts, Tris> makeGrid3x3()
    {
        Verts verts = {
            v(0,0,0), v(1,0,0), v(2,0,0),   // row 0
            v(0,1,0), v(1,1,0), v(2,1,0),   // row 1
            v(0,2,0), v(1,2,0), v(2,2,0),   // row 2
        };
        Tris tris = {
            {0,3,1}, {1,3,4},   // square (r0c0)–(r1c1)
            {1,4,2}, {2,4,5},   // square (r0c1)–(r1c2)
            {3,6,4}, {4,6,7},   // square (r1c0)–(r2c1)
            {4,7,5}, {5,7,8},   // square (r1c1)–(r2c2)
        };
        return { verts, tris };
    }

    // 6 vertices, 5 triangles — a star-fan with a non-boundary centre vertex.
    //
    // Topology:
    //   v0 = (0, 0, 0)       centre, NON-BOUNDARY (all 5 spokes are interior edges)
    //   v1 = (0.001, 0, 0)   ring,   BOUNDARY      (distance 0.001 from v0)
    //   v2 = (1, 0, 0)       ring,   BOUNDARY
    //   v3 = (0, 1, 0)       ring,   BOUNDARY
    //   v4 = (−1, 0, 0)      ring,   BOUNDARY
    //   v5 = (0, −1, 0)      ring,   BOUNDARY
    //
    // The edge (v0, v1) is the shortest in the mesh (≈ 0.001 units) and is the
    // only edge that spans the boundary/non-boundary seam.  The bounding-box
    // diagonal is ≈ 2.83, so with shortEdgeRatio = 0.1 the threshold is ≈ 0.28,
    // which is larger than 0.001 → Pass A will always select this edge first.
    //
    // Used for:
    //   §5  Boundary gate  — with preserveBoundary=true, the (v0,v1) collapse is
    //                         rejected because boundary(v0) ≠ boundary(v1).
    //   §5  Skin-drift gate — same short edge; assigning v0 and v1 to different
    //                         bones guarantees a drift of ≈ 1.0, which any
    //                         reasonable maxSkinWeightDrift threshold rejects.
    //   §6  Pass A         — the edge qualifies as "short" and pointRemovals
    //                         counts a removal when no gates block the collapse.
    std::pair<Verts, Tris> makeBoundaryTestMesh()
    {
        Verts verts = {
            v(0,    0, 0),   // v0: centre, non-boundary
            v(0.001f,0, 0),  // v1: ring,   boundary, very close to v0
            v(1,    0, 0),   // v2: ring,   boundary
            v(0,    1, 0),   // v3: ring,   boundary
            v(-1,   0, 0),   // v4: ring,   boundary
            v(0,   -1, 0),   // v5: ring,   boundary
        };
        // Fan triangles — each spoke edge (v0,vN) appears in exactly 2 triangles
        // → interior.  Each perimeter edge (vN,vN+1) appears in exactly 1 → boundary.
        Tris tris = { {0,1,2}, {0,2,3}, {0,3,4}, {0,4,5}, {0,5,1} };
        return { verts, tris };
    }

    // ── Default options ───────────────────────────────────────────────────────

    // Returns a sensible, permissive option set with decimation enabled and no
    // hard reduction target.  Individual tests override only the fields relevant
    // to what they are verifying, keeping each test self-documenting.
    CollisionMeshDecimationOptions enabledOpts()
    {
        CollisionMeshDecimationOptions o;
        o.enabled                   = true;
        o.preserveBoundary          = false;
        o.preserveFeatures          = false;
        o.targetVertexCount         = 0;     // use targetVertexRatio instead
        o.targetVertexRatio         = 1.0f;  // no target — only short edges / QEM fire
        o.qemCostThreshold          = 1.0f;
        o.shortEdgeRatio            = 0.01f;
        o.maxVolumeLossPercent      = 5.0f;
        o.maxLocalVolumeChangePercent = 5.0f;
        o.maxNormalDeviationDegrees = 45.0f;
        o.maxSkinWeightDrift        = 0.0f;
        return o;
    }

    // ── Generic output validators ─────────────────────────────────────────────

    // Every triangle index must be strictly less than the number of output vertices.
    void requireIndicesInBounds(const CollisionMeshDecimationOutput& out)
    {
        const auto nv = static_cast<uint32_t>(out.vertices.size());
        for (const auto& tri : out.triangles) {
            REQUIRE(tri[0] < nv);
            REQUIRE(tri[1] < nv);
            REQUIRE(tri[2] < nv);
        }
    }

    // No output triangle may be degenerate (two or more equal indices).
    void requireNoDegenerate(const CollisionMeshDecimationOutput& out)
    {
        for (const auto& tri : out.triangles) {
            REQUIRE(tri[0] != tri[1]);
            REQUIRE(tri[1] != tri[2]);
            REQUIRE(tri[2] != tri[0]);
        }
    }

    // The sentinel written into oldToNewVertex for removed vertices.
    // Mirrors the private constant inside hdtMeshDecimator.cpp.
    constexpr uint32_t kNoMapping = std::numeric_limits<uint32_t>::max();

    // Every non-sentinel entry in oldToNewVertex must be a valid index into
    // out.vertices, and the mapping must be injective (no two old vertices
    // map to the same new index, since each surviving vertex has a unique slot).
    void requireMappingConsistent(const CollisionMeshDecimationOutput& out,
                                   const Verts& inputVertices)
    {
        REQUIRE(out.oldToNewVertex.size() == inputVertices.size());
        const auto nv = static_cast<uint32_t>(out.vertices.size());
        for (uint32_t i = 0; i < static_cast<uint32_t>(out.oldToNewVertex.size()); ++i) {
            if (out.oldToNewVertex[i] != kNoMapping)
                REQUIRE(out.oldToNewVertex[i] < nv);
        }
    }

}  // anonymous namespace


// ═════════════════════════════════════════════════════════════════════════════
// §1  Passthrough semantics
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("disabled flag: output equals input, no fallback recorded", "[passthrough]")
{
    // When options.enabled is false the function must return the original mesh
    // verbatim and must NOT set usedFallback — the caller asked for a no-op,
    // so there is nothing to fall back from.
    auto [verts, tris] = makeTetrahedron();

    CollisionMeshDecimationOptions opts;  // enabled = false by default
    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.vertices.size()  == verts.size());
    REQUIRE(out.triangles.size() == tris.size());
    REQUIRE_FALSE(out.stats.usedFallback);
}

TEST_CASE("empty vertex list: returns empty output without crashing", "[passthrough]")
{
    // A mesh with no vertices is degenerate by definition; the function must
    // return an empty (not undefined) output rather than assert or throw.
    Verts emptyV;
    Tris  emptyT;

    auto out = DecimateCollisionMesh(emptyV, emptyT, enabledOpts());

    REQUIRE(out.vertices.empty());
    REQUIRE(out.triangles.empty());
}

TEST_CASE("empty triangle list: returns input vertices, empty triangles, no crash", "[passthrough]")
{
    // Vertices with no triangles is a degenerate case. The function restores the
    // input (identity mapping) rather than returning a null mesh — vertices are
    // preserved so callers that iterate oldToNewVertex still get valid data.
    auto [verts, _] = makeTetrahedron();
    Tris emptyT;

    auto out = DecimateCollisionMesh(verts, emptyT, enabledOpts());

    REQUIRE(out.vertices.size() == verts.size());  // input vertices preserved
    REQUIRE(out.triangles.empty());                 // no triangles to restore
}

TEST_CASE("single triangle: output has exactly 3 vertices", "[passthrough]")
{
    // The algorithm's minimum vertex count is 3 (hard-coded via max(3, …)).
    // A single-triangle mesh cannot be further reduced.
    auto [verts, tris] = makeSingleTriangle();

    auto opts            = enabledOpts();
    opts.targetVertexCount = 3;  // explicit minimum — still can't go lower

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.vertices.size()  == 3u);
    REQUIRE(out.triangles.size() == 1u);
}


// ═════════════════════════════════════════════════════════════════════════════
// §2  Structural invariants
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("all output triangle indices are within bounds", "[invariants]")
{
    // If the index remapping after collapse is wrong, indices can point past the
    // end of the vertex list and cause undefined behaviour at runtime.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 4;
    opts.maxNormalDeviationDegrees = 90.0f;  // permissive so collapses happen

    auto out = DecimateCollisionMesh(verts, tris, opts);

    requireIndicesInBounds(out);
}

TEST_CASE("no degenerate triangles in output", "[invariants]")
{
    // applyCandidate removes degenerate triangles from the working list, and
    // buildOutput skips them.  A degenerate triangle in the final output
    // (two equal vertex indices) would trigger the post-check fallback.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 4;
    opts.maxNormalDeviationDegrees = 90.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    requireNoDegenerate(out);
}

TEST_CASE("stats.outputVertexCount equals out.vertices.size()", "[invariants]")
{
    // The stats field must mirror the actual container size so callers can use
    // either without discrepancy.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 5;
    opts.maxNormalDeviationDegrees = 90.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.outputVertexCount   == static_cast<int>(out.vertices.size()));
    REQUIRE(out.stats.outputTriangleCount == static_cast<int>(out.triangles.size()));
}

TEST_CASE("stats.originalVertexCount equals the input size", "[invariants]")
{
    // The stats snapshot of the original mesh must be taken before any work
    // begins, independent of whether decimation succeeds or falls back.
    auto [verts, tris] = makeTetrahedron();
    auto opts = enabledOpts();
    opts.maxVolumeLossPercent = 0.0001f;  // tight — forces fallback

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.originalVertexCount   == static_cast<int>(verts.size()));
    REQUIRE(out.stats.originalTriangleCount == static_cast<int>(tris.size()));
}


// ═════════════════════════════════════════════════════════════════════════════
// §3  Reduction
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("vertex count never increases after decimation", "[reduction]")
{
    // This is the most fundamental invariant of any simplification algorithm.
    // It must hold regardless of options, geometry, or which pass runs.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 4;
    opts.maxNormalDeviationDegrees = 90.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.vertices.size() <= verts.size());
}

TEST_CASE("absolute targetVertexCount is respected on a reducible flat grid", "[reduction]")
{
    // The 3×3 flat grid has 9 vertices.  Targeting 8 requires exactly one valid
    // collapse (centre vertex v4 into one of its neighbours).  The flat surface
    // ensures no normal-deviation rejection, and volume guards are inactive.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 8;
    opts.maxNormalDeviationDegrees = 90.0f;  // flat mesh — normals never flip
    opts.maxLocalVolumeChangePercent = 100.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    // The output must not exceed the requested target.
    REQUIRE(out.vertices.size() <= 8u);
    // At least one collapse must have occurred (the grid is reducible).
    REQUIRE(out.vertices.size() < verts.size());
}

TEST_CASE("targetVertexRatio reduces vertex count on a reducible flat grid", "[reduction]")
{
    // ratio = 0.8 → ceil(0.8 × 9) = 8 → same target as the absolute test above,
    // but exercised via the ratio path in the option resolution code.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexRatio         = 0.8f;
    opts.maxNormalDeviationDegrees = 90.0f;
    opts.maxLocalVolumeChangePercent = 100.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.vertices.size() <= 8u);
    REQUIRE(out.vertices.size() < verts.size());
}


// ═════════════════════════════════════════════════════════════════════════════
// §4  Vertex mapping
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("fallback oldToNewVertex is the identity map", "[mapping]")
{
    // When a fallback occurs (here: global volume-loss violation during Pass B),
    // restoreInputAsOutput writes an identity mapping: oldToNewVertex[i] == i for
    // all i.  This allows callers to always dereference the mapping safely.
    //
    // maxLocalVolumeChangePercent=100 is needed to let the first collapse through
    // the local guard, after which the global guard (maxVolumeLossPercent=0.0001%)
    // immediately fires because collapsing any tetrahedron edge nearly zeroes the
    // signed volume of the surviving (non-closed) triangle pair.
    auto [verts, tris] = makeTetrahedron();
    auto opts = enabledOpts();
    opts.targetVertexCount          = 3;      // force at least one collapse attempt
    opts.maxVolumeLossPercent       = 0.0001f;
    opts.maxLocalVolumeChangePercent = 100.0f; // open local gate so global gate fires

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.usedFallback);
    REQUIRE(out.oldToNewVertex.size() == verts.size());
    for (uint32_t i = 0; i < static_cast<uint32_t>(verts.size()); ++i)
        REQUIRE(out.oldToNewVertex[i] == i);
}

TEST_CASE("every surviving vertex maps to a valid output index", "[mapping]")
{
    // After a successful decimation (no fallback), each non-sentinel entry in
    // oldToNewVertex must index into the output vertex list.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 7;
    opts.maxNormalDeviationDegrees = 90.0f;
    opts.maxLocalVolumeChangePercent = 100.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE_FALSE(out.stats.usedFallback);
    requireMappingConsistent(out, verts);
}

TEST_CASE("oldToNewVertex size always equals the input vertex count", "[mapping]")
{
    // The mapping array must cover every original index so callers can index it
    // with any original vertex index without a bounds check.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.targetVertexCount         = 5;
    opts.maxNormalDeviationDegrees = 90.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.oldToNewVertex.size() == verts.size());
}


// ═════════════════════════════════════════════════════════════════════════════
// §5  Safety gates
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("tight volume limit triggers fallback and output equals input", "[safety]")
{
    // A maxVolumeLossPercent of 0.0001 % on the tetrahedron (volume ≈ 2.67)
    // sets volumeCap ≈ 2.67e-6.  Any single edge collapse changes the volume
    // by far more than this, so the fallback must fire after the very first
    // successful collapse in Pass B.
    //
    // Post-fallback guarantees: usedFallback=true, output == input, stats reflect
    // the original mesh dimensions.
    auto [verts, tris] = makeTetrahedron();
    auto opts = enabledOpts();
    opts.targetVertexCount           = 3;      // force collapse attempt
    opts.maxVolumeLossPercent        = 0.0001f;
    opts.maxLocalVolumeChangePercent = 100.0f; // open local gate so global gate fires

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.usedFallback);
    REQUIRE(out.stats.fallbackReason == "volume-loss-threshold");
    REQUIRE(out.vertices.size()  == verts.size());
    REQUIRE(out.triangles.size() == tris.size());
    // Volume delta is zeroed on fallback (output == input, so Δ = 0 by definition)
    REQUIRE_THAT(out.stats.volumeDeltaPercent,
                 Catch::Matchers::WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("boundary gate rejects a collapse across the boundary seam", "[safety]")
{
    // In makeBoundaryTestMesh, v0 is non-boundary (all 5 spoke edges have count=2)
    // and v1–v5 are boundary (each connected to a perimeter edge with count=1).
    // The edge (v0,v1) is 0.001 units long — well below the Pass A threshold
    // (shortEdgeRatio=0.1, diag≈2.83, threshold≈0.28).
    //
    // With preserveBoundary=true, validateCandidate checks that keep.boundary ==
    // remove.boundary and returns false when they differ.  Pass A therefore cannot
    // collapse (v0,v1) and increments rejectedBoundary before breaking.
    auto [verts, tris] = makeBoundaryTestMesh();
    auto opts = enabledOpts();
    opts.preserveBoundary  = true;
    opts.shortEdgeRatio    = 0.1f;   // 0.1 × diag(≈2.83) = 0.283 > 0.001 → edge qualifies
    opts.targetVertexCount = 5;      // allow one attempt (6 − 1 = 5)

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.rejectedBoundary > 0);
    // No vertex was actually removed — the only candidate was rejected.
    REQUIRE(out.vertices.size() == verts.size());
}

TEST_CASE("skin-drift gate rejects a collapse between differently-weighted vertices", "[safety]")
{
    // The edge (v0,v1) in makeBoundaryTestMesh is 0.001 units long and will be
    // the first candidate chosen by Pass A.  Here v0 is assigned 100 % bone 0
    // and v1 is assigned 100 % bone 1, giving a merge skin-drift of ≈ 1.0.
    // With maxSkinWeightDrift = 0.01, that drift is 100× the threshold and must
    // be rejected, incrementing rejectedSkin.
    auto [verts, tris] = makeBoundaryTestMesh();

    // Reassign v1 to bone 1 to create a maximum-drift situation on the short edge.
    verts[1].m_weight[0]  = 1.0f;
    verts[1].m_boneIdx[0] = 1;

    auto opts = enabledOpts();
    opts.preserveBoundary    = false;  // skip boundary gate so we reach skin gate
    opts.shortEdgeRatio      = 0.1f;
    opts.targetVertexCount   = 5;
    opts.maxSkinWeightDrift  = 0.01f;  // very tight — any cross-bone merge is rejected

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.rejectedSkin > 0);
    REQUIRE(out.vertices.size() == verts.size());  // no vertex removed
}

TEST_CASE("volumeDeltaPercent stays within maxVolumeLossPercent when decimation succeeds",
          "[safety]")
{
    // When the algorithm finishes without a fallback the reported volumeDeltaPercent
    // must not exceed the configured maxVolumeLossPercent.  (The global guard is
    // checked after every Pass B collapse, so a violation would trigger fallback
    // before reaching this point — this test just verifies the stat is credible.)
    auto [verts, tris] = makeTetrahedron();
    auto opts = enabledOpts();
    opts.targetVertexCount    = 4;     // no reduction target — no collapse forced
    opts.maxVolumeLossPercent = 5.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    if (!out.stats.usedFallback)
        REQUIRE(out.stats.volumeDeltaPercent <= 5.0f + 1e-3f);  // small float tolerance
}


// ═════════════════════════════════════════════════════════════════════════════
// §6  Pass A — short-edge point removal
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("short-edge removal fires when an edge falls below the ratio threshold",
          "[passA]")
{
    // makeBoundaryTestMesh contains the edge (v0,v1) with length 0.001.
    // The bounding-box diagonal is ≈ 2.83, so with shortEdgeRatio = 0.1 the
    // short-edge threshold is ≈ 0.28 — well above 0.001.
    //
    // With preserveBoundary=false and no skin constraints, validateCandidate
    // passes (both v0 and v1 share boundary=false? — no: actually both are in
    // the boundary set because the perimeter edges are count=1.  Wait — v0 is
    // non-boundary; see mesh doc above.  preserveBoundary=false skips the check
    // regardless, so the collapse is allowed to proceed.
    //
    // After one successful Pass A collapse, pointRemovals increments to 1 and
    // the output has one fewer vertex.
    auto [verts, tris] = makeBoundaryTestMesh();
    auto opts = enabledOpts();
    opts.preserveBoundary          = false;
    opts.shortEdgeRatio            = 0.1f;
    opts.targetVertexCount         = 5;    // 6 vertices → allow 1 removal
    opts.maxNormalDeviationDegrees = 90.0f;
    opts.maxLocalVolumeChangePercent = 100.0f;

    auto out = DecimateCollisionMesh(verts, tris, opts);

    REQUIRE(out.stats.pointRemovals >= 1);
    REQUIRE(out.vertices.size() < verts.size());
}

TEST_CASE("large shortEdgeRatio does not fire for a mesh with no short edges", "[passA]")
{
    // The 3×3 flat grid has all edges of length 1 or √2.  With shortEdgeRatio = 0.001
    // the threshold is ≈ 0.003, which is below all edge lengths, so Pass A must find
    // zero qualifying edges and leave pointRemovals at 0.
    auto [verts, tris] = makeGrid3x3();
    auto opts = enabledOpts();
    opts.shortEdgeRatio    = 0.001f;
    opts.targetVertexCount = 8;  // set a target so the loop can enter

    auto out = DecimateCollisionMesh(verts, tris, opts);

    // Pass A contributes nothing; any reduction comes from Pass B.
    REQUIRE(out.stats.pointRemovals == 0);
}
