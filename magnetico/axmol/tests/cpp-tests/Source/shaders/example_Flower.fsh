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

//float u( float x ) { return 0.5+0.5*sign(x); }
float u( float x ) { return (x>0.0)?1.0:0.0; }
//float u( float x ) { return abs(x)/x; }

layout(location = SV_Target0) out vec4 FragColor;

void main(void)
{
	float time = u_Time[1];
#ifdef METAL
	vec2 fragCoord = vec2(gl_FragCoord.x, u_screenSize.y - gl_FragCoord.y);
#else
	vec2 fragCoord = gl_FragCoord.xy;
#endif
	vec2 p = 2.0 * (fragCoord - center.xy) / resolution.xy;
	
	float a = atan(p.x,p.y);
	float r = length(p)*.75;

	float w = cos(3.1415927*time-r*2.0);
	float h = 0.5+0.5*cos(12.0*a-w*7.0+r*8.0);
	float d = 0.25+0.75*pow(h,1.0*r)*(0.7+0.3*w);

	float rd = 1.0-r/d;
	float col = 0.0;
	if(rd > 0.0)
	    col = u( d-r ) * sqrt(rd)*r*2.5;	
	col *= 1.25+0.25*cos((12.0*a-w*7.0+r*8.0)/2.0);
	col *= 1.0 - 0.35*(0.5+0.5*sin(r*30.0))*(0.5+0.5*cos(12.0*a-w*7.0+r*8.0));
	FragColor = vec4(
		col,
		col-h*0.5+r*.2 + 0.35*h*(1.0-r),
		col-h*r + 0.1*h*(1.0-r),
		1.0);
}