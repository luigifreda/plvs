#version 330 core

// visualization shader 

// second geometry shader which processes vertices (specify the max_vertices per time)

layout(points) in;

layout(line_strip, max_vertices = 4) out;
//layout(points, max_vertices = 8) out;
//layout(triangle_strip, max_vertices = 8) out;


#define N_DOWN_SAMPLE 2u
#define NORMAL_LENGTH 0.1f

// global input from normal code 
uniform mat4 matModelView;

// input from geometry shader 
in uvec3 vColor[];
in vec3 vPosition[];
in vec3 vNormal[];

//flat in int vId[];


// output towards fragment shader 
out vec3 colorGeom;

vec3 decodeColor(float c)
{
    vec3 col;
    col.x = float(int(c) >> 16 & 0xFF) / 255.0f;
    col.y = float(int(c) >> 8 & 0xFF) / 255.0f;
    col.z = float(int(c) & 0xFF) / 255.0f;
    return col;
}

vec3 decodeColorU(uvec3 c)
{
    vec3 col;
    float factor = 0*1/255.0f; 
    col.x = float(c.r) * factor;
    col.y = float(c.g) * factor;
    col.z = float(c.b) * factor;
    return col;
}

void main()
{
    int i = 0;
    float factor = NORMAL_LENGTH; 

    for(i=0; i < gl_in.length(); i++)
    {    
        // this does not work since a specific point will have a changing ID 
        /*if(vId[i] % N_DOWN_SAMPLE == 0)  
        {
            factor = NORMAL_LENGTH; 
        } 
        else
        {
            factor = NORMAL_LENGTH;
        }*/   
        
        gl_Position = matModelView * vec4(vPosition[i], 1.0);
        //colorGeom     = decodeColorU(vColor[i]);
        colorGeom = vec3(1.0f, 1.0f, 0.0f);  
        EmitVertex();

        gl_Position = matModelView * vec4(vPosition[i] + vNormal[i] * factor, 1.0);
        //gl_Position = matModelView * vec4(vPosition[i] + vec3(0,-1,0) * factor, 1.0);
        //colorGeom     = decodeColorU(vColor[i]);
        colorGeom = vec3(1.0f, 0.0f, 0.0f);  
        EmitVertex();

        EndPrimitive();
    }
}
