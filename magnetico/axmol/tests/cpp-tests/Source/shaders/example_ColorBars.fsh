#version 310 es
precision highp float;
precision highp int;
// http://www.cocos2d-iphone.org


layout(location = TEXCOORD0) in vec2 v_texCoord;
layout(binding = 0) uniform sampler2D u_tex0;

vec4 getColorByCoord(int y){
    if(y < 5){
         if(y == 0){
            return vec4(1,0,0,1);
         } else if(y == 1){
            return vec4(0,1,0,1);
         } else if(y == 2){
            return vec4(0,0,1,1);
         } else if(y == 3){
            return vec4(0,1,1,1);
         } else{
            return vec4(1,0,1,1);
         }
     } else {
         if(y == 5){
            return vec4(1,1,0,1);
         } else if(y == 6){
            return vec4(1,1,1,1);
         } else if(y == 7){
            return vec4(1,0.5,0,1);
         } else if(y == 8){
            return vec4(1,0.5,0.5,1);
         }else {
            return vec4(0.5,0.5,1,1);
         }
     }
}

layout(location = SV_Target0) out vec4 FragColor;

void main(void) {
	// inline to prevent "float" loss and keep using lowp
    //int y = int( mod(( (gl_FragCoord.y+gl_FragCoord.x)*mod(AX_Time[0],5.0)) / 10.0, 10.0 ) );
	//int y = int( mod( AX_Time[3] + (gl_FragCoord.y + gl_FragCoord.x) / 10.0, 10.0 ) );
	int y = int( mod(gl_FragCoord.y / 10.0, 10.0 ) );
	FragColor = getColorByCoord(y) * texture(u_tex0, v_texCoord);
}
