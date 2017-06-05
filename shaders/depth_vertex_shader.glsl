#version 420

in vec3 position;
in vec3 normal;

uniform mat4 MVP;
uniform float alpha;
uniform vec3 camCenter;

vec3 sweepPosition(vec3 position, float alpha, vec3 normal){
  //simply along the normal
 vec3 position_new = position + alpha * normal;

  //or by considering the current viewing ray
  vec3 camToV1 = (position-camCenter)/length(position-camCenter);
  //float angle1 = atan(length(cross(normal,camToV1)),dot(normal,camToV1)); 

  vec3 x = camToV1;
  vec3 y = normal;

  float angle1 = 2 * atan(length(x * length(y) - length(x) * y), length(x * length(y) + length(x) * y));
 
  float k_1 = 0.0f;
  if (abs(cos(angle1)) >= 0.05){ 
    //do not consider triangles for which the angle of incidence of the current ray is too narrow 
     k_1 = alpha * abs(cos(angle1));
  }
  position_new = position + k_1 * camToV1;
  
  return position_new;
}



void main(){
  vec3 position_new = sweepPosition(position, alpha, normal);
  gl_Position =  MVP * vec4(position_new,1.0);
}

