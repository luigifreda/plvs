#version 330 core

// visualization shader for raw features using Kannala-Brandt distortion model
                        
attribute vec4 position;

// uniforms 
uniform mat4 matModelView;
uniform mat4 matProjection;
uniform vec4 kbIntrinsicsk1k2k3k4; // Kannala-Brandt k1 k2 k3 k4
uniform vec3 color;

// out 
out vec3 color_;
out float distance_;

// Kannala-Brandt distort 3D point represented in camera frame  
vec4 distort(vec4 position_c)
{
    vec2 uvn = position_c.xy /position_c.z; // normalized coords  
    float uvnLength = length(uvn);
    float psi = atan(uvn.x, uvn.y);

    float theta = atan(uvnLength);
    float theta2 = theta * theta;
    float theta4 = theta2 * theta2;
    float theta6 = theta4 * theta2;
    float theta8 = theta4 * theta4;

    float r = theta * (1.0 + kbIntrinsicsk1k2k3k4[0] * theta2 + kbIntrinsicsk1k2k3k4[1] * theta4 + kbIntrinsicsk1k2k3k4[2] * theta6 + kbIntrinsicsk1k2k3k4[3] * theta8);

    // distorted coords 
    float ud = r * sin (psi);
    float vd = r * cos (psi);        
               
    return vec4(ud*position_c.z, vd*position_c.z, position_c.z, position_c.w);
}   

void main()
{
    vec3 camera_position_w = -matModelView[3].xyz * mat3(matModelView);
#if 0
    distance_ = length(position.xz - camera_position_w.xz); // horizontal distance 
#else 
    distance_ = length(position.xyz - camera_position_w.xyz); // 3D distance    
#endif 
    vec3 position_w = position.xyz; // point position in world coordinates   
    vec4 position_c = matModelView * vec4(position_w, 1.0); // point position in camera coordinates  
    gl_Position = matProjection * distort(position_c); // project distorted normalized coords 
    color_ = color;
}
