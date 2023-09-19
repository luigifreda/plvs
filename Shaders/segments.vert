#version 330 core

// visualization shader 

// first vertex shader which processes one vertex per time 

layout (location = 0) in vec3 position; // input from glEnableVertexAttribArray(0)
layout (location = 1) in uvec2 label;   // input from glEnableVertexAttribArray(1)

// global input from normal code 
uniform mat4 matModelView;

// output towards geometry shader
out uvec3 vColor;    
out vec3 vPosition;
out uint vLabel;
out uint vLabelConfidence; 


const uint ICOLOR = 5u;
const uint DELTACOLOR = (256u/ICOLOR);

const uint minColorVal = 256u/ICOLOR;
const uint rangeColor  = (256u/ICOLOR)*(ICOLOR-1u);

const uint PRIME_NUMBER=997u;

uvec3 decodeColor(uint c)
{
    uint intcolor = c;
    //uint intcolor = (c >> 16) & 0xFFu; 

    if(intcolor == 0u) 
        return uvec3(0u,0u,0u);

    uint numColor = (intcolor*PRIME_NUMBER);

    uvec3 col;
    col.r = minColorVal + uint(numColor%rangeColor);
    numColor = numColor/rangeColor;
    col.g = minColorVal + uint(numColor%rangeColor);
    numColor = numColor/rangeColor;
    col.b = minColorVal + uint(numColor%rangeColor);
    return col;
}

uvec3 decodeColorS(uint c)
{
    if(c == 0u) 
        return uvec3(0u,0u,0u);
    else
        return uvec3(0u,0u,255u);
}

uint decodeLabelConfidence(uint data)
{
    return uint(int(data) & 0xFF); 
}


void main()
{
    vPosition = position;
    vColor    = decodeColor(label[0]);
    vLabel    = label[0]; 
    vLabelConfidence = label[1];

    gl_Position = matModelView * vec4(position.xyz, 1.0f);

}