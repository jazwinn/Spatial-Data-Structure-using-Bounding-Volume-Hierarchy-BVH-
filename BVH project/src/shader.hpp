#ifndef SHADER_HPP
#define SHADER_HPP

#include "opengl.hpp"
#include "math.hpp"
#include "utils.hpp"

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <iostream> // Debug

namespace CS350 {
    class Shader {
      private:
        GLuint m_program;
        GLuint m_vertex;
        GLuint m_fragment;

      public:
        Shader& operator=(Shader&&) = delete;
        Shader(char const* vtxCode, char const* frgCode);
        ~Shader();
        Shader(Shader const&)            = delete;
        Shader& operator=(Shader const&) = delete;

        void use();
    };

    template <typename T>
    void set_uniform(GLint location, T const& v);

}

#endif // SHADER_HPP
