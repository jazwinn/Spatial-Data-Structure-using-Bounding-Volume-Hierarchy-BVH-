#include "shader.hpp"
#include <stdexcept>
#include <cstring>

namespace {

    GLuint CompileShader(GLenum shader, char const* src, unsigned int size) {
        unsigned shader_object = glCreateShader(shader);

        // Upload source
        const char* blocks[]      = { src };
        const int   block_sizes[] = { (int)size };
        glShaderSource(shader_object, 1, blocks, block_sizes);

        // Compile it
        glCompileShader(shader_object);

        // Retrieve result
        std::string compile_result;
        GLint       info_len = 0;
        glGetShaderiv(shader_object, GL_INFO_LOG_LENGTH, &info_len);
        if (info_len > 0) {
            compile_result.resize(static_cast<unsigned>(info_len), '\0');
            GLsizei written = 0;
            glGetShaderInfoLog(shader_object, info_len, &written, compile_result.data());
        }

        // Check if compiled
        GLint compiled = 0;
        glGetShaderiv(shader_object, GL_COMPILE_STATUS, &compiled);
        if (compiled == 0) {
            auto err = fmt::format("Error compiling Shader: {}", compile_result);
            std::cerr << err << std::endl;
            throw std::runtime_error(err);
        }
        return shader_object;
    }
}

namespace CS350 {
    Shader::Shader(char const* vtxCode, char const* frgCode) {
        m_program  = glCreateProgram();
        m_vertex   = CompileShader(GL_VERTEX_SHADER, vtxCode, static_cast<unsigned>(std::strlen(vtxCode)));
        m_fragment = CompileShader(GL_FRAGMENT_SHADER, frgCode, static_cast<unsigned>(std::strlen(frgCode)));

        glAttachShader(m_program, m_vertex);
        glAttachShader(m_program, m_fragment);
        glLinkProgram(m_program);

        { // Retrieve result
            std::string link_result;
            GLint       info_len = 0;
            glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &info_len);
            if (info_len > 0) {
                link_result.resize(static_cast<unsigned>(info_len), '\0');
                GLsizei written = 0;
                glGetProgramInfoLog(m_program, info_len, &written, link_result.data());
            }

            // Check if linked
            GLint linked = 0;
            glGetProgramiv(m_program, GL_LINK_STATUS, &linked);
            if (linked == 0) {
                auto err = fmt::format("Error linking Shader: {}", link_result);
                std::cerr << err << std::endl;
                throw std::runtime_error(err);
            }
        }
    }

    Shader::~Shader() {
        if (m_program != 0) {
            glDeleteProgram(m_program);
        }
        if (m_fragment != 0) {
            glDeleteShader(m_fragment);
        }
        if (m_vertex != 0) {
            glDeleteShader(m_vertex);
        }
    }

    void Shader::use() {
        glUseProgram(m_program);
    }

    template <>
    void set_uniform<int>(GLint location, int const& v) { glUniform1i(location, v); }

    template <>
    void set_uniform<float>(GLint location, float const& v) { glUniform1f(location, v); }

    template <>
    void set_uniform<vec2>(GLint location, vec2 const& v) { glUniform2fv(location, 1, &v[0]); }

    template <>
    void set_uniform<vec3>(GLint location, vec3 const& v) { glUniform3fv(location, 1, &v[0]); }

    template <>
    void set_uniform<vec4>(GLint location, vec4 const& v) { glUniform4fv(location, 1, &v[0]); }

    template <>
    void set_uniform<mat3>(GLint location, mat3 const& v) { glUniformMatrix3fv(location, 1, GL_FALSE, &v[0][0]); }

    template <>
    void set_uniform<mat4>(GLint location, mat4 const& v) { glUniformMatrix4fv(location, 1, GL_FALSE, &v[0][0]); }
}
