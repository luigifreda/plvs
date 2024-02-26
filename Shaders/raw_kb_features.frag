#version 330 core

// visualization shader for raw features using Kannala-Brandt distortion model
                        
uniform float minDistanceColor;
uniform float maxDistanceColor;
uniform float maxDistanceRender;
uniform float alpha;

in vec3 color_;
in float distance_;

void main()
{
    float clampedDistance = clamp(distance_, minDistanceColor, maxDistanceColor);
    float normalizedDistance = (clampedDistance - minDistanceColor) / (maxDistanceColor - minDistanceColor);

#if 0
    vec3 force = vec3(1.0, 1.0-normalizedDistance, 1.0);
    vec3 color = color_ * force;
#else 
    // interpolate between input color and red 
    vec3 redColor = vec3(1.0, 0.0, 0.0);
    vec3 color = color_*(1.0-normalizedDistance) + redColor*normalizedDistance;
#endif 

    float actualAlpha = alpha*(1.0 - smoothstep(maxDistanceRender*0.2, maxDistanceRender, distance_));    
    gl_FragColor = vec4(color, actualAlpha);
}
