#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <iostream>
#include "math.hpp"
#include "shapes.hpp"

namespace glm {
    std::istream& operator>>(std::istream& is, ivec2& v);
    std::ostream& operator<<(std::ostream& os, ivec2 const& v);
    std::istream& operator>>(std::istream& is, vec2& v);
    std::ostream& operator<<(std::ostream& os, vec2 const& v);
    std::istream& operator>>(std::istream& is, vec3& v);
    std::ostream& operator<<(std::ostream& os, vec3 const& v);
    std::istream& operator>>(std::istream& is, vec4& v);
    std::ostream& operator<<(std::ostream& os, vec4 const& v);
    std::istream& operator>>(std::istream& is, mat3& v);
    std::ostream& operator<<(std::ostream& os, mat3 const& v);
    std::istream& operator>>(std::istream& is, mat4& v);
    std::ostream& operator<<(std::ostream& os, mat4 const& v);
}

namespace CS350 {
    std::istream& operator>>(std::istream& is, Line& v);
    std::ostream& operator<<(std::ostream& os, Line const& l);
    std::istream& operator>>(std::istream& is, Ray& v);
    std::ostream& operator<<(std::ostream& os, Ray const& ray);
    std::istream& operator>>(std::istream& is, Segment& segment);
    std::ostream& operator<<(std::ostream& os, Segment const& segment);
    std::istream& operator>>(std::istream& is, Plane& v);
    std::ostream& operator<<(std::ostream& os, Plane const& plane);
    std::istream& operator>>(std::istream& is, Triangle& v);
    std::ostream& operator<<(std::ostream& os, Triangle const& tri);
    std::istream& operator>>(std::istream& is, Sphere& v);
    std::ostream& operator<<(std::ostream& os, Sphere const& sphere);
    std::istream& operator>>(std::istream& is, Aabb& aabb);
    std::ostream& operator<<(std::ostream& os, Aabb const& aabb);
    std::istream& operator>>(std::istream& is, Frustum& frustum);
    std::ostream& operator<<(std::ostream& os, Frustum const& frustum);
}

#include <fmt/format.h>
#include <fmt/ostream.h>
#if FMT_VERSION >= 90000
// Fix for windows
template <> struct fmt::formatter<vec2> : ostream_formatter {};
template <> struct fmt::formatter<vec3> : ostream_formatter {};
template <> struct fmt::formatter<vec4> : ostream_formatter {};
template <> struct fmt::formatter<CS350::Sphere> : ostream_formatter {};
template <> struct fmt::formatter<CS350::Aabb> : ostream_formatter {};
#endif

#endif // LOGGING_HPP
