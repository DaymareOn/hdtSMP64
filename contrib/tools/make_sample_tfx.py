#!/usr/bin/env python3
"""Generate a reference TressFX .tfx groom for the FSMP strand-wig engine.

This is a worked example of the binary format FSMP's loadTfx() reads, and a known-good asset
authors can test against. It writes a hemispherical scalp cap that drapes downward -- the same
shape FSMP generates procedurally, just baked into a file.

Coordinate convention (what an author must match):
  * positions are OFFSETS FROM THE HEAD ORIGIN, in Skyrim units (a head is ~12 units across),
  * axes are world-up: +Z is up, so the scalp sits near (0, 0, +10) and strands grow toward -Z.
FSMP converts these to head-local at bind time, so the wig follows the head on any actor.

.tfx layout: a 160-byte header, then numStrands*numVerts float4 vertices (xyz = position,
w = inverse mass; w=0 pins vertex 0 of each strand as the fixed root).
"""
import math
import struct
import sys

NUM_STRANDS = 300
NUM_VERTS = 16          # must be a power of two: 4, 8, 16, 32 or 64
BASE_LENGTH = 50.0      # long hair, so an authored groom is visibly distinct from the short default
LENGTH_VAR = 0.25
CAP_RADIUS = 8.0        # head half-width
CAP_TOP = 10.0          # scalp height above the head origin
GOLDEN_ANGLE = 2.399963


def hash01(n: int, k: float) -> float:
    """Deterministic pseudo-random in [0,1): fract(sin(n*k) * 43758.5453). Same as the shader/solver."""
    v = math.sin(n * k) * 43758.5453
    return v - math.floor(v)


def build_positions():
    """Yield numStrands*numVerts (x, y, z, w) tuples, strand-major, root first."""
    for s in range(NUM_STRANDS):
        t = (s + 0.5) / NUM_STRANDS
        zf = 0.15 + 0.85 * t                     # crown bias: skip the forehead ring
        ring = math.sqrt(max(0.0, 1.0 - zf * zf))
        phi = s * GOLDEN_ANGLE
        dir_w = (ring * math.cos(phi), ring * math.sin(phi), zf)

        h1, h2 = hash01(s, 12.9898), hash01(s, 78.233)
        jitter = ((h1 - 0.5) * 0.4, (h2 - 0.5) * 0.4, 0.0)
        grow = [dir_w[i] * 0.35 + jitter[i] for i in range(3)]
        grow[2] -= 0.65                          # bias the growth direction downward
        gl = math.sqrt(sum(c * c for c in grow)) or 1.0
        grow = [c / gl for c in grow]

        length = BASE_LENGTH * (1.0 - LENGTH_VAR + 2.0 * LENGTH_VAR * h1)
        seg = length / (NUM_VERTS - 1)

        pos = [CAP_TOP * 0 + dir_w[0] * CAP_RADIUS * 0.98,
               dir_w[1] * CAP_RADIUS * 0.98,
               CAP_TOP + dir_w[2] * CAP_RADIUS * 0.98]
        for v in range(NUM_VERTS):
            w = 0.0 if v == 0 else 1.0           # pin the root
            yield (pos[0], pos[1], pos[2], w)
            pos = [pos[i] + grow[i] * seg for i in range(3)]


def main(out_path: str):
    header = struct.pack(
        "<fIIIIIII" + "I" * 32,
        4.0,             # version
        NUM_STRANDS,
        NUM_VERTS,
        160,             # offsetVertexPosition (immediately after this header)
        0, 0, 0, 0,      # offsetStrandUV / VertUV / Thickness / Color (unused)
        *([0] * 32),     # reserved
    )
    assert len(header) == 160, len(header)

    body = bytearray()
    for x, y, z, w in build_positions():
        body += struct.pack("<ffff", x, y, z, w)

    with open(out_path, "wb") as f:
        f.write(header)
        f.write(body)
    print(f"wrote {out_path}: {NUM_STRANDS} strands x {NUM_VERTS} verts, {160 + len(body)} bytes")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "sample.tfx")
