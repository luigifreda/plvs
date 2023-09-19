#version 330 core

// visualization shader 

// third (after vertex and geometry shader) comes the fragment shader which processes one vertex color per time

// input from geometry shader  
in vec3 colorGeom;

// final fragment output (must be vec4)
out vec4 colorFrag;

void main()
{
    colorFrag = vec4(colorGeom,1.0f);
}
