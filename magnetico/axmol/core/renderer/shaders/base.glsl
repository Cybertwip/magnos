
#define DIRLIGHT TEXCOORD1
#define POINTLIGHT TEXCOORD2
#define SPOTLIGHT TEXCOORD4
#define SPOTLIGHT_NORM TEXCOORD5

/* follow macro in glslcc too large
    "TANGENT",
    "BINORMAL",
    "BLENDINDICES",
    "BLENDWEIGHT"
*/
#undef TANGENT
#undef BINORMAL
#undef BLENDINDICES
#undef BLENDWEIGHT

#define TANGENT TEXCOORD6
#define BINORMAL TEXCOORD7
#define BLENDINDICES COLOR1
#define BLENDWEIGHT COLOR2
