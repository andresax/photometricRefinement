#version 420

in vec3 position;              

out vec4 shadowCoordV;         
out vec4 projectorTexCoordV;

uniform mat4 MVP;

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
}

