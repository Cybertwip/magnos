#version 310 es
precision highp float;
precision highp int;
// http://www.cocos2d-iphone.org


layout(location = TEXCOORD0) in vec2 v_texCoord;

layout(binding = 0) uniform sampler2D u_tex0;

layout(std140) uniform fs_ub {
    vec2 u_screenSize;
};

layout(location = SV_Target0) out vec4 FragColor;

void main(void)
{
	vec4 optColor;
#ifdef METAL
	float fragCoordY = u_screenSize.y - gl_FragCoord.y;
#else
	float fragCoordY = gl_FragCoord.y;
#endif
	int y = int( mod(fragCoordY / 3.0, 10.0 ) );
	if(y == 0)	optColor = vec4(1,0,0,1);
	else if(y == 1) optColor = vec4(0,1,0,1);
	else if(y == 2) optColor = vec4(0,0,1,1);
	else if(y == 3) optColor = vec4(0,1,1,1);
	else if(y == 4) optColor = vec4(1,0,1,1);
	else if(y == 5) optColor = vec4(1,1,0,1);
	else if(y == 6) optColor = vec4(1,1,1,1);
	else if(y == 7) optColor = vec4(1,0.5,0,1);
	else if(y == 8) optColor = vec4(1,0.5,0.5,1);
	else if(y == 9) optColor = vec4(0.5,0.5,1,1);
	
	// inline to prevent "float" loss and keep using lowp
	FragColor = optColor * texture(u_tex0, v_texCoord);
}