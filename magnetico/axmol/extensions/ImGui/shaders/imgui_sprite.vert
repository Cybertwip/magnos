#version 310 es

layout(location = POSITION) in vec4 a_position;
layout(location = TEXCOORD0) in vec2 a_texCoord;
layout(location = COLOR0) in vec4 a_color;

layout(location = COLOR0) out vec4 v_color;
layout(location = TEXCOORD0) out vec2 v_texCoord;

layout(std140) uniform vs_ub {
    mat4 u_MVPMatrix;
};

void main()
{
    v_texCoord = a_texCoord;
    v_color = a_color;
    gl_Position = u_MVPMatrix * vec4(a_position.xy, 0.0, 1.0);
}
