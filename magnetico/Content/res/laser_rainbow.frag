#include <metal_stdlib>
#include <simd/simd.h>

uniform float3 iResolution;
float iGlobalTime;

float3 Strand(float2 fragCoord, float3 color, float hoffset, float hscale, float vscale, float timescale) {
    float glow = 0.06 * iResolution.y;
    float twopi = 6.28318530718;
    float curve = 1.0 - abs(fragCoord.y - (sin(mod(fragCoord.x * hscale / 100.0 / iResolution.x * 1000.0 + iGlobalTime * timescale + hoffset, twopi)) * iResolution.y * 0.25 * vscale + iResolution.y / 2.0));
    float i = clamp(curve, 0.0, 1.0);
    i += clamp((glow + curve) / glow, 0.0, 1.0) * 0.4;
    return i * color;
}

fragment float4 fragment_main(VertexOut in [[stage_in]], texture2d<float> tex [[texture(0)]]) {
    float timescale = 4.0;
    float3 c = float3(0, 0, 0);
    c += Strand(in.texCoord, float3(1.0, 0, 0), 0.7934 + 1.0 + sin(iGlobalTime) * 30.0, 1.0, 0.16, 10.0 * timescale);
    c += Strand(in.texCoord, float3(0.0, 1.0, 0.0), 0.645 + 1.0 + sin(iGlobalTime) * 30.0, 1.5, 0.2, 10.3 * timescale);
    c += Strand(in.texCoord, float3(0.0, 0.0, 1.0), 0.735 + 1.0 + sin(iGlobalTime) * 30.0, 1.3, 0.19, 8.0 * timescale);
    c += Strand(in.texCoord, float3(1.0, 1.0, 0.0), 0.9245 + 1.0 + sin(iGlobalTime) * 30.0, 1.6, 0.14, 12.0 * timescale);
    c += Strand(in.texCoord, float3(0.0, 1.0, 1.0), 0.7234 + 1.0 + sin(iGlobalTime) * 30.0, 1.9, 0.23, 14.0 * timescale);
    c += Strand(in.texCoord, float3(1.0, 0.0, 1.0), 0.84525 + 1.0 + sin(iGlobalTime) * 30.0, 1.2, 0.18, 9.0 * timescale);

    // Convert the final color to a normalized RGBA value
    float4 finalColor = float4(c.r, c.g, c.b, 1.0);

    // Apply a red overlay with some transparency
    finalColor += float4(0.6, 0, 0, 0.6);

    return finalColor;
}