#ifndef PRIMITIVE_HPP
#define PRIMITIVE_HPP

#include "opengl.hpp"
#include "math.hpp"
#include <vector>

namespace CS350 {

    class Primitive {
      private:
        GLuint*  m_vbos         = {};
        unsigned m_vbo_count    = {};
        GLuint   m_vao          = {};
        GLsizei  m_vertex_count = {};
        GLsizei  m_stride       = {};

      public:
        Primitive(void const* vboData, unsigned vboSize);
        ~Primitive();
        Primitive(Primitive const& rhs)            = delete;
        Primitive& operator=(Primitive&&)          = delete;
        Primitive& operator=(Primitive const& rhs) = delete;
        void       draw(GLenum mode);
        GLsizei    vertexCount() const { return m_vertex_count; }

      protected:
        void create_buffers(void const* vboData, unsigned vboSize);
    };
}
#endif // PRIMITIVE_HPP
