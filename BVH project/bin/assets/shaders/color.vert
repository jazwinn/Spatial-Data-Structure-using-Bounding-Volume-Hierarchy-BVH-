#version 440 core
    layout(location = 0) in vec3 attr_position;
    layout(location = 0) uniform mat4 uniform_m2w;
    layout(location = 1) uniform mat4 uniform_v;
    layout(location = 2) uniform mat4 uniform_p;

    //out vec4 position;
    
    void main()
    {
        vec4 vertex = vec4(attr_position, 1.0f);
        gl_Position = uniform_p * uniform_v * uniform_m2w * vertex;
    }
