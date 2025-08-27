#include "debug_renderer.hpp"
#include "shader.hpp"
#include "primitive.hpp"

namespace CS350 {

    namespace {
        constexpr uint32_t cUniformMvp   = 0;
        constexpr uint32_t cUniformColor = 1;

        char const* c_vertex_shader = R"(
        #version 440 core
        layout(location = 0) in vec3 attr_position;
        layout(location = 0) uniform mat4 uniform_mvp;
        void main()
        {
            vec4 vertex = vec4(attr_position, 1.0f);
            gl_Position = uniform_mvp * vertex;
        }
        )";

        char const* c_fragment_shader = R"(
        #version 440 core
        out vec4 out_color;
        layout(location = 1) uniform vec4 uniform_color;
        void main()
        {
            out_color = uniform_color;
        }
        )";
    }

    DebugRenderer::DebugRenderer() {
        m_shader = new Shader(c_vertex_shader, c_fragment_shader);

        { // Create point
            vec3 const vbo_data[] = { { 0, 0, 0 } };
            m_mesh_point          = new Primitive(vbo_data, sizeof(vbo_data));
        }

        { // Create segment
            vec3 const vbo_data[] = { { 0, 0, 0 }, { 1, 1, 1 } };
            m_mesh_segment        = new Primitive(vbo_data, sizeof(vbo_data));
        }

        { // Create AABB
            vec3 const vbo_data_unindexed[] = {
                { -0.5, -0.5, -0.5 },
                { -0.5, -0.5, 0.5 },
                { -0.5, 0.5, -0.5 },
                { -0.5, 0.5, 0.5 },
                { 0.5, -0.5, -0.5 },
                { 0.5, -0.5, 0.5 },
                { 0.5, 0.5, -0.5 },
                { 0.5, 0.5, 0.5 },
            };
            vec3 const vbo_data[] = {
                vbo_data_unindexed[0],
                vbo_data_unindexed[6],
                vbo_data_unindexed[4],
                vbo_data_unindexed[0],
                vbo_data_unindexed[2],
                vbo_data_unindexed[6],
                vbo_data_unindexed[0],
                vbo_data_unindexed[3],
                vbo_data_unindexed[2],
                vbo_data_unindexed[0],
                vbo_data_unindexed[1],
                vbo_data_unindexed[3],
                vbo_data_unindexed[2],
                vbo_data_unindexed[7],
                vbo_data_unindexed[6],
                vbo_data_unindexed[2],
                vbo_data_unindexed[3],
                vbo_data_unindexed[7],
                vbo_data_unindexed[4],
                vbo_data_unindexed[6],
                vbo_data_unindexed[7],
                vbo_data_unindexed[4],
                vbo_data_unindexed[7],
                vbo_data_unindexed[5],
                vbo_data_unindexed[0],
                vbo_data_unindexed[4],
                vbo_data_unindexed[5],
                vbo_data_unindexed[0],
                vbo_data_unindexed[5],
                vbo_data_unindexed[1],
                vbo_data_unindexed[1],
                vbo_data_unindexed[5],
                vbo_data_unindexed[7],
                vbo_data_unindexed[1],
                vbo_data_unindexed[7],
                vbo_data_unindexed[3]
            };
            m_mesh_aabb = new Primitive(vbo_data, sizeof(vbo_data));
        }

        { // Create AABB
            vec3 const vbo_data_unindexed[] = {
                { -0.5, -0.5, -0.5 },
                { -0.5, -0.5, 0.5 },
                { -0.5, 0.5, -0.5 },
                { -0.5, 0.5, 0.5 },
                { 0.5, -0.5, -0.5 },
                { 0.5, -0.5, 0.5 },
                { 0.5, 0.5, -0.5 },
                { 0.5, 0.5, 0.5 },
            };
            unsigned const LBN = 0;
            unsigned const LBF = 1;
            unsigned const LTN = 2;
            unsigned const LTF = 3;
            unsigned const RBN = 4;
            unsigned const RBF = 5;
            unsigned const RTN = 6;
            unsigned const RTF = 7;

            vec3 const vbo_data[] = {
                vbo_data_unindexed[LBN],
                vbo_data_unindexed[LBF],
                vbo_data_unindexed[LBN],
                vbo_data_unindexed[LTN],
                vbo_data_unindexed[LBN],
                vbo_data_unindexed[RBN],
                vbo_data_unindexed[LTF],
                vbo_data_unindexed[LTN],
                vbo_data_unindexed[LTF],
                vbo_data_unindexed[LBF],
                vbo_data_unindexed[LTF],
                vbo_data_unindexed[RTF],
                vbo_data_unindexed[RBF],
                vbo_data_unindexed[RBN],
                vbo_data_unindexed[RBF],
                vbo_data_unindexed[RTF],
                vbo_data_unindexed[RBF],
                vbo_data_unindexed[LBF],
                vbo_data_unindexed[RTN],
                vbo_data_unindexed[RTF],
                vbo_data_unindexed[RTN],
                vbo_data_unindexed[RBN],
                vbo_data_unindexed[RTN],
                vbo_data_unindexed[LTN],
            };
            m_mesh_aabb_lines = new Primitive(vbo_data, sizeof(vbo_data));
        }

        { // Create plane
            vec3 const vbo_data[] = {
                { -0.5, 0.5, 0.0 },
                { 0.5, -0.5, 0.0 },
                { -0.5, -0.5, 0.0 },
                { -0.5, 0.5, 0.0 },
                { 0.5, 0.5, 0.0 },
                { 0.5, -0.5, 0.0 },
            };
            m_mesh_plane = new Primitive(vbo_data, sizeof(vbo_data));
        }

        { // Create triangles
            vec3 const vbo_data[] = {
                { 1, 0, 0 },
                { 0, 1, 0 },
                { 0, 0, 1 }
            };
            m_mesh_triangle = new Primitive(vbo_data, sizeof(vbo_data));
        }

        { // Create Disc
            vec3 vbo_data[361] = {};
            for (int i = 0; i <= 360; ++i) {
                float x     = glm::sin(glm::radians<float>(static_cast<float>(i)));
                float y     = glm::cos(glm::radians<float>(static_cast<float>(i)));
                vbo_data[i] = { x, y, 0 };
            }
            m_mesh_disc = new Primitive(vbo_data, sizeof(vbo_data));
        }
    }

    DebugRenderer::~DebugRenderer() {
        delete m_shader;
        delete m_mesh_point;
        delete m_mesh_segment;
        delete m_mesh_triangle;
        delete m_mesh_aabb;
        delete m_mesh_aabb_lines;
        delete m_mesh_plane;
        delete m_mesh_disc;
    }

    void DebugRenderer::draw_point(mat4 const& vp, vec3 const& pt, vec4 const& color, float pointSize) {
        auto const& mvp = vp * glm::translate(pt);
        glPointSize(pointSize);
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_point->draw(GL_POINTS);
    }

    void DebugRenderer::draw_segment(mat4 const& vp, vec3 const& s, vec3 const& e, vec4 const& color) {
        auto const& mvp = vp * glm::translate(s) * glm::scale(e - s);
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_segment->draw(GL_LINES);
    }

    void DebugRenderer::draw_triangle(mat4 const& vp, vec3 const& a, vec3 const& b, vec3 const& c, vec4 const& color) {
        mat4 transform  = mat4(1);
        transform[0]    = vec4(a, 0);
        transform[1]    = vec4(b, 0);
        transform[2]    = vec4(c, 0);
        auto const& mvp = vp * transform;
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);

        glCullFace(GL_BACK);
        m_mesh_triangle->draw(GL_TRIANGLES);

        glCullFace(GL_FRONT);
        set_uniform(cUniformColor, color * 0.5f);
        m_mesh_triangle->draw(GL_TRIANGLES);
    }

    void DebugRenderer::draw_aabb(mat4 const& vp, vec3 const& c, vec3 const& size, vec4 const& color) {
        auto const& mvp = vp * glm::translate(c) * glm::scale(size);
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_aabb->draw(GL_TRIANGLES);

        // Lines
        auto lines_color = vec4(1, 1, 1, 1) - color; // Complementary color
        lines_color      = {};                       // Override to black
        lines_color.w    = 1.0f;

        draw_aabb_wireframe(vp, c, size, lines_color);
    }

    void DebugRenderer::draw_aabb_wireframe(mat4 const& vp, vec3 const& c, vec3 const& size, vec4 const& color) {
        auto const& mvp = vp * glm::translate(c) * glm::scale(size);
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_aabb_lines->draw(GL_LINES);
    }

    void DebugRenderer::draw_plane(mat4 const& vp, vec3 const& pt, vec3 const& n, float scale, vec4 const& color) {
        // Compute orientation
        vec3 up = { 0, 1, 0 };
        if (glm::epsilonEqual(glm::abs(glm::dot(up, n)), 1.0f, 0.001f)) {
            up = { 1, 0, 0 };
        }

        // Billboard approach
        mat4 rot{};
        vec3 look  = glm::normalize(-n);
        vec3 right = glm::normalize(glm::cross(look, up));
        up         = glm::normalize(glm::cross(look, right));
        rot[0]     = vec4(right, 0);
        rot[1]     = vec4(up, 0);
        rot[2]     = vec4(look, 0);
        rot[3]     = vec4(0, 0, 0, 1);

        auto mvp = vp * glm::translate(pt) * rot * glm::scale(vec3{ scale });
        glCullFace(GL_BACK);
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_plane->draw(GL_TRIANGLES);

        // Draw normal
        draw_segment(vp, pt, pt + n, { 1, 1, 1, 1 });

        //
        rot[0] = vec4(right, 0);
        rot[1] = vec4(up, 0);
        rot[2] = vec4(-look, 0);
        rot[3] = vec4(0, 0, 0, 1);
        mvp    = vp * glm::translate(pt) * rot * glm::scale(vec3{ scale });
        glCullFace(GL_FRONT);
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color * 0.5f);
        m_mesh_plane->draw(GL_TRIANGLES);
    }

    void DebugRenderer::draw_sphere(mat4 const& vp, vec3 cameraPosition, vec3 const& c, float r, glm::vec4 const& color) {
        { // Base
            auto mvp = vp * glm::translate(c) * glm::scale(vec3(r));
            set_uniform(cUniformMvp, mvp);
            set_uniform(cUniformColor, color);
            m_mesh_disc->draw(GL_LINE_STRIP);
        }

        { // One rotation
            auto mvp = vp * glm::translate(c) * glm::rotate(glm::half_pi<float>(), vec3{ 1, 0, 0 }) * glm::scale(vec3(r));
            set_uniform(cUniformMvp, mvp);
            set_uniform(cUniformColor, color);
            m_mesh_disc->draw(GL_LINE_STRIP);
        }

        { // One rotation
            auto mvp = vp * glm::translate(c) * glm::rotate(glm::half_pi<float>(), vec3{ 0, 1, 0 }) * glm::scale(vec3(r));
            set_uniform(cUniformMvp, mvp);
            set_uniform(cUniformColor, color);
            m_mesh_disc->draw(GL_LINE_STRIP);
        }

        { // Horizon disc
            auto  diff                = cameraPosition - c;
            float d                   = glm::length(diff);
            float l                   = glm::sqrt(d * d - r * r);
            float horizon_disc_radius = r * l / d;
            float z                   = glm::sqrt(r * r - horizon_disc_radius * horizon_disc_radius);
            vec3  c_moved             = c + z * diff / d;

            // Billboard approach
            mat4 R{};
            vec3 up    = { 0, 1, 0 };
            vec3 look  = glm::normalize(-diff);
            vec3 right = glm::normalize(glm::cross(look, up));
            up         = glm::normalize(glm::cross(look, right));
            R[0]       = vec4(right, 0);
            R[1]       = vec4(up, 0);
            R[2]       = vec4(look, 0);
            R[3]       = vec4(0, 0, 0, 1);

            auto mvp = vp * glm::translate(c_moved) * R * glm::scale(vec3(horizon_disc_radius));
            set_uniform(cUniformMvp, mvp);
            set_uniform(cUniformColor, color);

            m_mesh_disc->draw(GL_LINE_STRIP);
        }
    }

    void DebugRenderer::draw_frustum(mat4 const& vp, mat4 const& frustumVp, vec4 const& color) {
        auto const& mvp = vp * glm::inverse(frustumVp) * glm::scale(vec3(2.0f)); // Since aabb points are [-0.5, 0.5]
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_aabb->draw(GL_TRIANGLES);
        draw_frustum_wireframe(vp, frustumVp, vec4(0, 1, 0, 1));
    }

    void DebugRenderer::draw_frustum_wireframe(mat4 const& vp, mat4 const& frustumVp, vec4 const& color) {
        glDisable(GL_BLEND);
        auto const& mvp = vp * glm::inverse(frustumVp) * glm::scale(vec3(2.0f)); // Since aabb points are [-0.5, 0.5]
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        m_mesh_aabb_lines->draw(GL_LINES);
        glEnable(GL_BLEND);
    }
    void DebugRenderer::draw_primitive(mat4 const& vp, mat4 const& m2w, Primitive* primitive, vec4 const& color) {
        auto const& mvp = vp * m2w;
        set_uniform(cUniformMvp, mvp);
        set_uniform(cUniformColor, color);
        primitive->draw(GL_TRIANGLES);
    }

    void DebugRenderer::draw_begin() {
        m_shader->use();
    }
}
