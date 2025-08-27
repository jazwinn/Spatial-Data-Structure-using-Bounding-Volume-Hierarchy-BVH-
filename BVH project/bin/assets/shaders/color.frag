#version 440 core
        out vec4 out_color;
        layout(location = 3) uniform vec4 uniform_color;
    void main()
    {
        out_color = uniform_color;
    }
