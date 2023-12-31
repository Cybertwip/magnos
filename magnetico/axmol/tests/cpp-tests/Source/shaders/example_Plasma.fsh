#version 310 es
precision highp float;
precision highp int;
// Shader from http://www.iquilezles.org/apps/shadertoy/



layout(std140) uniform fs_ub {
    vec2 center;
    vec2 resolution;
    vec2 u_screenSize;
    vec4 u_Time;
};

layout(location = SV_Target0) out vec4 FragColor;

void main(void)
{
#ifdef METAL
	float fragCoordY = u_screenSize.y - gl_FragCoord.y;
#else
	float fragCoordY = gl_FragCoord.y;
#endif
   float time = u_Time[1];
   float x = gl_FragCoord.x - (center.x - resolution.x / 2.0);
   float y = fragCoordY - (center.y - resolution.y / 2.0);
   float mov0 = x+y+cos(sin(time)*2.)*100.+sin(x/100.)*1000.;
   float mov1 = y / resolution.y / 0.2 + time;
   float mov2 = x / resolution.x / 0.2;
   float c1 = abs(sin(mov1+time)/2.+mov2/2.-mov1-mov2+time);
   float c2 = abs(sin(c1+sin(mov0/1000.+time)+sin(y/40.+time)+sin((x+y)/100.)*3.));
   float c3 = abs(sin(c2+cos(mov1+mov2+c2)+cos(mov2)+sin(x/1000.)));
   FragColor = vec4( c1,c2,c3,1.0);
}
