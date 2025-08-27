#ifndef DEBUGRENDERER_HPP
#define DEBUGRENDERER_HPP

#include "math.hpp"

namespace CS350 {
    class Shader;
    class Primitive;

    /**
     * @brief
     *  Controls the debug resources and is able to debug draw
     */
    class DebugRenderer {
      private:
        Shader*    m_shader;
        Primitive* m_mesh_point;
        Primitive* m_mesh_segment;
        Primitive* m_mesh_triangle;
        Primitive* m_mesh_aabb;
        Primitive* m_mesh_aabb_lines;
        Primitive* m_mesh_plane;
        Primitive* m_mesh_disc;

      public:
        DebugRenderer();
        ~DebugRenderer();

        DebugRenderer(DebugRenderer const&)            = delete;
        DebugRenderer& operator=(DebugRenderer const&) = delete;

        void draw_point(mat4 const& vp, vec3 const& pt, vec4 const& color, float pointSize = 1.0f);
        void draw_segment(mat4 const& vp, vec3 const& s, vec3 const& e, vec4 const& color);
        void draw_triangle(mat4 const& vp, vec3 const& a, vec3 const& b, vec3 const& c, vec4 const& color);
        void draw_aabb(mat4 const& vp, vec3 const& c, vec3 const& size, vec4 const& color);
        void draw_aabb_wireframe(mat4 const& vp, vec3 const& c, vec3 const& size, vec4 const& color);
        void draw_plane(mat4 const& vp, vec3 const& pt, vec3 const& n, float scale, vec4 const& color);
        void draw_sphere(mat4 const& vp, vec3 cameraPosition, vec3 const& c, float r, glm::vec4 const& color);
        void draw_frustum(mat4 const& vp, mat4 const& frustumVp, vec4 const& color);
        void draw_frustum_wireframe(mat4 const& vp, mat4 const& frustumVp, vec4 const& color);
        void draw_primitive(mat4 const& vp, mat4 const& m2w, Primitive* primitive, vec4 const& color);
        void draw_begin();
    };
}

#endif // DEBUGRENDERER_HPP
