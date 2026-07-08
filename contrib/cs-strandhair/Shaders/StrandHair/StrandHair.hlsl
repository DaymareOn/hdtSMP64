// Strand Hair -- vertex + pixel shader (Part B render pass 1: thickness ribbons + occlusion).
// TressFX-style expansion: every bead becomes two vertices (SV_VertexID = beadIndex*2 + side),
// offset perpendicular to both the strand tangent and the view ray, so each segment renders as a
// camera-facing ribbon with root->tip taper. Camera data comes from the game's per-frame cbuffer
// (b12, Common/FrameBuffer.hlsli): camera-relative position, matrix-first mul -- the only correct
// convention. PPLL OIT, self-shadow, and SSS/deferred parity are later Part B milestones.

#include "Common/FrameBuffer.hlsli"
#include "Common/SharedData.hlsli"

struct Bead
{
    float3 pos;
    float t;  // 0 at root .. 1 at tip
};
StructuredBuffer<Bead> Beads : register(t0);

cbuffer StrandCB : register(b0)
{
    float4 g_params;  // x = strand half-width in world units, yzw unused
};

struct VSOut
{
    float4 pos : SV_POSITION;
    float t : TEXCOORD0;
    float3 tangent : TEXCOORD1;
    float3 viewDir : TEXCOORD2;
};

VSOut VSMain(uint id : SV_VertexID)
{
    const uint beadIdx = id >> 1;
    const float side = (id & 1) ? 1.0 : -1.0;
    Bead b = Beads[beadIdx];

    // Tangent from the neighbor along the same strand. Beads are strand-major and t rises
    // monotonically root->tip, so "next bead has larger t" means it continues this strand;
    // otherwise this bead is a tip and the previous bead is the neighbor.
    uint total, strideUnused;
    Beads.GetDimensions(total, strideUnused);
    float3 tangent;
    if (beadIdx + 1 < total && Beads[beadIdx + 1].t > b.t)
        tangent = Beads[beadIdx + 1].pos - b.pos;
    else
        tangent = b.pos - Beads[beadIdx - 1].pos;
    tangent = normalize(tangent + float3(1e-6, 0, 0));

    float3 positionCR = b.pos - FrameBuffer::CameraPosAdjust.xyz;  // camera-relative world position

    // Camera-facing ribbon: offset sideways, perpendicular to the strand and the view ray.
    float3 viewDir = normalize(positionCR);
    float3 offsetDir = normalize(cross(tangent, viewDir) + float3(1e-6, 0, 0));
    float halfWidth = g_params.x * (1.0 - 0.5 * b.t);  // taper toward the tip
    positionCR += offsetDir * (halfWidth * side);

    VSOut o;
    o.pos = mul(FrameBuffer::CameraViewProj, float4(positionCR, 1.0));
    o.t = b.t;
    o.tangent = tangent;
    o.viewDir = viewDir;
    return o;
}

float4 PSMain(VSOut i) : SV_Target
{
    float3 tangent = normalize(i.tangent);
    // Scene sun (direction TO the light, colour x fade x HDR scale), from SharedData (b5).
    float3 L = normalize(SharedData::DirLightDirection.xyz);

    // Kajiya-Kay diffuse: hair scatters around the strand axis, so brightness ~ sin(tangent, light).
    float tDotL = dot(tangent, L);
    float diffuse = sqrt(saturate(1.0 - tDotL * tDotL));

    // Skyrim's directional ambient (DALC) with a camera-facing pseudo-normal: ribbons have no
    // stable surface normal, and facing the camera matches how the eye reads a hair mass.
    float3 pseudoNormal = -normalize(i.viewDir);
    float3 ambient = mul(SharedData::DirectionalAmbient, float4(pseudoNormal, 1.0));

    float3 rootColor = float3(0.05, 0.03, 0.02);
    float3 tipColor = float3(0.16, 0.10, 0.06);
    float3 baseColor = lerp(rootColor, tipColor, i.t);

    float3 lit = baseColor * (ambient + SharedData::DirLightColor.rgb * diffuse);
    float alpha = lerp(0.95, 0.6, i.t);
    return float4(lit, alpha);
}
