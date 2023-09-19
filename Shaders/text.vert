#version 330 core


layout(location = 0) in vec3 vertexPosition_modelspace;

out vec3 pos;   // screen position <-1,+1>

void main()
{
    pos=vec3(-1,1,0);
    gl_Position.xyz = vertexPosition_modelspace;
    gl_Position.w = 1.0;
}