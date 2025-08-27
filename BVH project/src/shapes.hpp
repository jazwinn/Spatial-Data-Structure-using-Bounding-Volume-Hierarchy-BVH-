#ifndef __SHAPES_HPP__
#define __SHAPES_HPP__

#include "math.hpp"
#include <vector>
#include <array>

// Forward declarations
namespace CS350 {
    struct Aabb;
    struct Frustum;
    struct Sphere;
    struct Triangle;
    struct Line;
    struct Plane;
    struct Segment;
    struct Ray;

    enum SideResult {
        eINSIDE       = -1,
        eINTERSECTING = 0,
        eOUTSIDE      = 1
    };

}

namespace CS350 {
    /**
     * @brief
     */
    struct Line {
        vec3 start;
        vec3 dir;

        Line() = default;
        Line(vec3 const& _start, vec3 const& _dir);
        vec3 evaluate(float t) const;
    };
    static_assert(std::is_trivial<Line>());
    static_assert(std::is_standard_layout<Line>());

    /**
     * @brief
     * 	A ray class, defined by a point and a direction.
     * 	Note that the direction may or not be normalized
     */
    struct Ray {
        vec3 start;
        vec3 dir;

        Ray() = default;
        Ray(vec3 const& _start, vec3 const& _dir);
        vec3  at(float t) const;
        float intersect(Plane const& plane) const;
        float intersect(Aabb const& aabb) const;
        float intersect(Sphere const& sphere) const;
        float intersect(Triangle const& triangle) const;
    };

    static_assert(std::is_trivial<Ray>());
    static_assert(std::is_standard_layout<Ray>());

    /**
     * @brief
     * 	Describes a segment by two points, start and End.
     * 	Points put in array for conveniences.
     */
    struct Segment {
        std::array<vec3, 2> points;

        Segment() = default;
        Segment(vec3 const& start, vec3 const& end);

        vec3        closest(Segment const& rhs) const;
        float       distance(vec3 const& pt) const;
        vec3        at(float t) const;
        vec3        dir() const;
        vec3 const& operator[](int index) const;
        vec3&       operator[](int index);
    };
    static_assert(std::is_trivial<Segment>());
    static_assert(std::is_standard_layout<Segment>());

    /**
     * @brief
     * 	Defines a plane, storing the normal and the dot(n, p).
     * 	Note that when retrieving the point, different point may be returned, as it's a new computation
     */
    struct Plane {
        vec3  normal;
        float dot_result;

        Plane() = default;
        explicit Plane(Triangle const& triangle);
        Plane(vec3 const& point, vec3 const& _normal);
        Plane(vec3 const& _normal, float d);

        vec3 get_point() const;

        float      distance(vec3 const& pt) const;
        vec3       closest_point(vec3 const& pt) const;
        SideResult classify(vec3 const& pt) const;
        SideResult classify(Triangle const& triangle) const;
        SideResult classify(Aabb const& aabb) const;
        SideResult classify(Sphere const& sphere) const;
    };
    static_assert(sizeof(Plane) == sizeof(float) * 4);
    static_assert(std::is_trivial<Plane>());
    static_assert(std::is_standard_layout<Plane>());

    /**
     * @brief
     *  A basic triangle class. Defined by an array of three points
     */
    struct Triangle {
        std::array<vec3, 3> points;

        Triangle() = default;
        Triangle(vec3 const& a, vec3 const& b, vec3 const& c);
        vec3 closest(vec3 const& pt) const;

        void        invert();
        vec3        normal() const;
        vec3 const& operator[](int index) const;
        vec3&       operator[](int index);
        float       area() const;
    };
    static_assert(std::is_trivial<Triangle>());
    static_assert(std::is_standard_layout<Triangle>());

    /**
     * @brief
     *  A basic sphere class. Defined by a point and a radius
     */
    struct Sphere {
        vec3  center;
        float radius;

        Sphere() = default;
        Sphere(vec3 const& center, float radius);
        bool contains(const vec3&  pt) const;
        bool intersects(Sphere const& rhs) const;

        static Sphere centroid(vec3 const* points, size_t count);
        static Sphere centroid(vec3 const* points, size_t count, mat4 const& transformMatrix);
        static Sphere ritters(vec3 const* points, size_t count);
        static Sphere ritters(vec3 const* points, size_t count, mat4 const& transformMatrix);
        static Sphere iterative(vec3 const* points, size_t count, size_t iteration_count, float shrink_ratio);
        static Sphere iterative(vec3 const* points, size_t count, size_t iteration_count, float shrink_ratio, mat4 const& transform);
    };
    static_assert(std::is_trivial<Sphere>());
    static_assert(std::is_standard_layout<Sphere>());

    /**
     * @brief
     *  A basic Axis Aligned Bounding Box.
     *  Defined by min-max points.
     */
    struct Aabb {
        vec3 min;
        vec3 max;

        Aabb() = default;
        Aabb(vec3 const* points, size_t count, mat4 const& transformMatrix);
        Aabb(vec3 const* points, size_t count);
        Aabb(vec3 const& min_point, vec3 const& max_point);
        Aabb(Aabb const& bv, mat4 const& _transform);
        Aabb(Aabb const& lhs, Aabb const& rhs);

        Aabb  transform(mat4 const& m2w) const;
        bool  intersects(vec3 const& pt) const;
        bool  intersects(Aabb const& rhs) const;
        float surface_area() const;
        float volume() const;
        vec3  get_center() const;
        vec3  get_extents() const;
        int   longest_axis() const;
    };
    static_assert(std::is_trivial<Aabb>());
    static_assert(std::is_standard_layout<Aabb>());

    /**
     * @brief
     *  A basic frustum implementation.
     */
    struct Frustum {

        Frustum() = default;
        explicit Frustum(mat4 const& viewProj);
        Frustum( std::array<vec3, 6> const& normals,  std::array<float, 6> const& dists);

        SideResult classify(Sphere const& sphere) const;
        SideResult classify(Aabb const& aabb) const;

        /**
         * @brief
         *  Retrieves a plane on the frustum
         * @param index
         *  Index of each plane on the frustum
         * @return
         *  Returns the plane
         */
        Plane& operator[](const size_t index) {
            return planes[index];
        }

        /**
         * @brief
         *  Retrieves a plane on the frustum
         * @param index
         *  Index of each plane on the frustum
         * @return
         *  Returns the plane
         */
        Plane operator[](const size_t index) const {
            return planes[index];
        }


        std::array<Plane, 6> planes;

    };
    static_assert(std::is_trivial<Frustum>());
    static_assert(std::is_standard_layout<Frustum>());
}

#endif // __SHAPES_HPP__
