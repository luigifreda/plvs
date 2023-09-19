#version 330 core

// visualization shader 

// first vertex shader which processes one vertex per time 

layout (location = 0) in vec3 position; // input from glEnableVertexAttribArray(0)
layout (location = 1) in uvec3 color;   // input from glEnableVertexAttribArray(1)
layout (location = 2) in vec3 normal;   // input from glEnableVertexAttribArray(2)

// global input from normal code 
uniform mat4 matModelView;

// output towards geometry shader
//out uvec3 vColor;    
out vec3 vPosition;
out vec3 vNormal;

//flat out int vId;

void main()
{
    //vColor  = color;
    vPosition = position;
    vNormal   = normal;
    // vId    = gl_VertexID; // N.B.: a specific point will have a changing ID 
    
    gl_Position = matModelView * vec4(position.xyz, 1.0f);

}
