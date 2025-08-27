#include "primitive.hpp"
#include "opengl.hpp"
#include <cstring>

namespace CS350 {

    Primitive::Primitive(void const* vboData, unsigned int vboSize) {
        // Compute stride
        GLsizei stride = 0;
        stride += sizeof(float) * 3;

        // VBO
        m_vbo_count = 1;
        m_vbos     = new GLuint[m_vbo_count];
        auto& vbo = m_vbos[0];
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vboSize, vboData, GL_STATIC_DRAW);

        // VAO
        glGenVertexArrays(1, &m_vao);
        glBindVertexArray(m_vao);
        // ATTRIBUTES
        long offset = 0;
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)(offset));
        offset += sizeof(float) * 3;
        glBindVertexArray(0);

        // Vertex count
        m_vertex_count = static_cast<GLsizei>(vboSize) / stride;
        assert((vboSize % static_cast<unsigned>(stride)) == 0);
        m_stride = stride;
    }

    Primitive::~Primitive() {
        if (m_vao != 0) {
            glDeleteVertexArrays(1, &m_vao);
        }
        if (m_vbos != nullptr) {
            glDeleteBuffers(m_vbo_count, m_vbos);
            delete[] m_vbos;
        }
    }

    void Primitive::draw(GLenum mode) {
        glBindVertexArray(m_vao);
        glDrawArrays(mode, 0, m_vertex_count);
        glBindVertexArray(0);
    }
}
