#version 420

in vec3 position;              

out vec4 shadowCoordV;         
out vec4 projectorTexCoordV; 
out vec4 projectorCoordV;

uniform mat4 MVP;
uniform mat4 camera;

void main(){

    gl_Position = MVP * vec4(position, 1.0);

    mat4 biasMatrix = mat4(
        0.5, 0.0, 0.0, 0.0,
        0.0, 0.5, 0.0, 0.0,
        0.0, 0.0, 0.5, 0.0,
        0.5, 0.5, 0.5, 1.0
        );
    
    shadowCoordV = biasMatrix * MVP * vec4(position, 1.0);
    shadowCoordV.z -= 0.00000000500;
    
    projectorTexCoordV = MVP * vec4(position, 1.0);
    projectorTexCoordV.y = projectorTexCoordV.y;
    projectorTexCoordV = biasMatrix * projectorTexCoordV;
    projectorTexCoordV.x = projectorTexCoordV.x / projectorTexCoordV.w;
    projectorTexCoordV.y = projectorTexCoordV.y / projectorTexCoordV.w;
    projectorTexCoordV.z = projectorTexCoordV.z / projectorTexCoordV.w;
    projectorTexCoordV.w = projectorTexCoordV.w / projectorTexCoordV.w;
    projectorCoordV = vec4(position, 1.0)*camera;
    projectorCoordV.x = projectorCoordV.x / projectorCoordV.z;
    projectorCoordV.y = projectorCoordV.y / projectorCoordV.z;
}

