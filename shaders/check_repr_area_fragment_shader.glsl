#version 420

in float areaCheck;
in vec4 shadowCoord;

out vec4 color;

uniform sampler2DShadow shadowMap;

void main(){
  float shadowCoeff = textureProj(shadowMap, shadowCoord);
  
  color = vec4(areaCheck,areaCheck,areaCheck,1.0);
}
