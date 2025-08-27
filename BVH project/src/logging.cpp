#include "logging.hpp"
#include <iostream>
namespace {
    template <typename T>
    std::istream& GenericVecRead(std::istream& is, T& v)
    {
        for (int i = 0; i < T::length(); ++i) {
            is >> v[i];
            if (i + 1 != T::length()) {
                is.ignore(1, ',');
            }
        }
        return is;
    }

    template <typename T>
    std::ostream& GenericVecWrite(std::ostream& os, T const& v)
    {
        for (int i = 0; i < T::length(); ++i) {
            os << v[i];
            if (i + 1 != T::length()) {
                os << ", ";
            }
        }
        return os;
    }
}

namespace glm {
    std::istream& operator>>(std::istream& is, vec2& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, vec2 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, vec3& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, vec3 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, vec4& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, vec4 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, ivec2& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, ivec2 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, ivec3& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, ivec3 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, ivec4& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, ivec4 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, mat3& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, mat3 const& v) { return GenericVecWrite(os, v); }
    std::istream& operator>>(std::istream& is, mat4& v) { return GenericVecRead(is, v); }
    std::ostream& operator<<(std::ostream& os, mat4 const& v) { return GenericVecWrite(os, v); }
}

namespace CS350 {
    std::istream& operator>>(std::istream& is, Line& v)
    {
        is >> v.start;
        is >> v.dir;
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Line const& l)
    {
        os << l.start << " " << l.dir;
        return os;
    }

    std::istream& operator>>(std::istream& is, Ray& v)
    {
        is >> v.start;
        is >> v.dir;
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Ray const& ray)
    {
        os << ray.start << " " << ray.dir;
        return os;
    }

    std::istream& operator>>(std::istream& is, Segment& segment)
    {
        is >> segment[0] >> segment[1];
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Segment const& segment)
    {
        os << segment[0] << " " << segment[1];
        return os;
    }

    std::istream& operator>>(std::istream& is, Plane& v)
    {
        glm::vec3 point{};
        glm::vec3 normal{};
        is >> point;
        is >> normal;
        v = Plane(point, normal);
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Plane const& plane)
    {
        os << plane.get_point() << " " << plane.normal;
        return os;
    }

    std::istream& operator>>(std::istream& is, Triangle& v)
    {
        is >> v[0];
        is >> v[1];
        is >> v[2];
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Triangle const& tri)
    {
        os << tri.points[0] << " " << tri.points[1] << " " << tri.points[2];
        return os;
    }

    std::istream& operator>>(std::istream& is, Sphere& v)
    {
        is >> v.center;
        is >> v.radius;
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Sphere const& sphere)
    {
        os << sphere.center << " " << sphere.radius;
        return os;
    }

    std::istream& operator>>(std::istream& is, Aabb& Aabb)
    {
        is >> Aabb.min;
        is >> Aabb.max;
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Aabb const& Aabb)
    {
        os << Aabb.min << ", " << Aabb.max;
        return os;
    }

    std::istream& operator>>(std::istream& is, Frustum& frustum)
    {
        for (int j = 0; j < 6; ++j) {
            vec3 pt;
            is >> pt >> frustum[j].normal;
            frustum[j].dot_result = glm::dot(pt, frustum[j].normal);
        }
        return is;
    }

    std::ostream& operator<<(std::ostream& os, Frustum const& frustum)
    {
        for (int j = 0; j < 6; ++j) {
            os << frustum[j].normal << " " << frustum[j].dot_result;
        }
        return os;
    }
}
