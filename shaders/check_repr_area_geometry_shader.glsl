#version 420

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;
 

in vec4 shadowCoordV[];      
in vec4 projectorTexCoordV[];        

out vec4 shadowCoord;
out float areaCheck;

uniform sampler2DShadow shadowMap;
uniform float maxArea;

float orientPoint(vec2 v0, vec2 v1, vec2 p){
  mat2 m;
  m[0][0] = (v1.x - v0.x);  m[0][1] = ( p.x - v0.x);
  m[1][0] = (v1.y - v0.y);  m[1][1] = ( p.y - v0.y);

  return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}


void main(){
  vec2 pt0 = projectorTexCoordV[0].xy;
  vec2 pt1 = projectorTexCoordV[1].xy;
  vec2 pt2 = projectorTexCoordV[2].xy;
  float d0 = length(pt0-pt1);
  float d1 = length(pt0-pt2);
  float d2 = length(pt2-pt1);
  int which;
  int which2;

  if (d0>= d1 && d0 >= d2){
    which = 0;
    which2 = 1;
  }else if (d1>= d0 && d1 >= d2){
    which = 0;
    which2 = 2;
  }else {
    which = 2;
    which2 = 1;
  }

  float area = abs(0.5*orientPoint(pt0, pt1, pt2));
  
  for(int i = 0; i < gl_in.length(); i++){
    float shadowCoeff = textureProj(shadowMap, shadowCoordV[i]);
    gl_Position = gl_in[i].gl_Position;
    shadowCoord = shadowCoordV[i];

    if (area > maxArea && (i == which)||(i == which2)){
      areaCheck = shadowCoeff * 1.0;
    }

    EmitVertex();
  }
}