#version 330 core

// visualization shader 

// second geometry shader which processes vertices (specify the max_vertices per time)

layout(points) in;

//layout(line_strip, max_vertices = 4) out;
layout(points, max_vertices = 4) out;
//layout(triangle_strip, max_vertices = 8) out;


// global input from normal code 
uniform mat4 matModelView;
uniform uint labelConfidenceThreshold; 

// input from geometry shader 
in uvec3 vColor[];
in vec3 vPosition[];
in uint vLabel[];
in uint vLabelConfidence[]; 


// output towards fragment shader 
out vec3 colorGeom;

void main()
{
    int i = 0;
    for(i=0; i < gl_in.length(); i++)
    {    
        if( vLabel[i]>0u && vLabelConfidence[i]>=labelConfidenceThreshold )
        //if( vLabel[i]==139u  && vLabelConfidence[i]>=labelConfidenceThreshold )
        {
            gl_Position = matModelView * vec4(vPosition[i], 1.0);
            colorGeom = vColor[i]/255.f;

            EmitVertex();
            EndPrimitive();
        }
    }
}
