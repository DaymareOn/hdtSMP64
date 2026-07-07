// Strand Hair -- PoC vertex + pixel shader.
// The VS reads bead positions from a StructuredBuffer indexed by SV_VertexID (no input layout
// needed). Camera data comes from the game's own per-frame cbuffer (b12) via the canonical
// Common/FrameBuffer.hlsli, using the exact transform pattern every working CS shader uses:
// camera-relative position, matrix-first mul. Shading is a Kajiya-Kay diffuse term with a
// root->tip gradient; thickness quads, self-shadow, and SSS/deferred parity are Part-B upgrades.

#include "Common/FrameBuffer.hlsli"

struct Bead
{
    float3 pos;
    float t;  // 0 at root .. 1 at tip
};
StructuredBuffer<Bead> Beads : register(t0);

cbuffer StrandCB : register(b0)
{
    float4 g_lightDir;    // world-space direction to the light (xyz)
    float4 g_lightColor;
};

struct VSOut
{
    float4 pos : SV_POSITION;
    float t : TEXCOORD0;
    float3 world : TEXCOORD1;
};

VSOut VSMain(uint id : SV_VertexID)
{
    Bead b = Beads[id];
    VSOut o;
    o.world = b.pos;
    float3 positionCR = b.pos - FrameBuffer::CameraPosAdjust.xyz;  // camera-relative world position
    o.pos = mul(FrameBuffer::CameraViewProj, float4(positionCR, 1.0));
    o.t = b.t;
    return o;
}

float4 PSMain(VSOut i) : SV_Target
{
    // No per-vertex tangent on a line list; reconstruct it from screen-space world derivatives.
    float3 tangent = normalize(ddx(i.world) + ddy(i.world) + float3(1e-5, 0, 0));
    float3 L = normalize(g_lightDir.xyz);

    // Kajiya-Kay diffuse: hair scatters around the strand axis, so brightness ~ sin(tangent, light).
    float tDotL = dot(tangent, L);
    float diffuse = sqrt(saturate(1.0 - tDotL * tDotL));

    float3 rootColor = float3(0.05, 0.03, 0.02);
    float3 tipColor = float3(0.16, 0.10, 0.06);
    float3 baseColor = lerp(rootColor, tipColor, i.t);

    float3 lit = baseColor * (0.3 + 0.7 * diffuse) * g_lightColor.rgb;
    float alpha = lerp(0.95, 0.6, i.t);
    return float4(lit, alpha);
}
