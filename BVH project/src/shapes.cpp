/**
 * @file
 *  shapes.cpp
 * @author
 *  Jaz Winn Ng, 670001224, jazwinn.ng@digipen.edu
 * @date
 *  2025/05/31
 * @brief
 *  Provides the definition of all geometry and spatial operations
 * @copyright
 *  Copyright (C) 2025 DigiPen Institute of Technology.
 */

#include "stats.hpp"
#include "shapes.hpp"
#include <iostream>
#include <random>
#include <algorithm>


namespace {
    constexpr float cEpsilon = 1e-5f;
}

namespace CS350 {

    Line::Line(vec3 const& _start, vec3 const& _dir) :
        start{ _start },
        dir{ _dir }
    {}

    vec3 Line::evaluate(float t) const {
        return start + t * dir;
    }

    Aabb::Aabb(vec3 const& min_point, const vec3& max_point) :
        min{ glm::min(min_point.x, max_point.x),
            glm::min(min_point.y, max_point.y),
            glm::min(min_point.z, max_point.z) },
        max{ glm::max(min_point.x, max_point.x),
            glm::max(min_point.y, max_point.y),
            glm::max(min_point.z, max_point.z) }
    {}

    Aabb::Aabb(const vec3* points, size_t count):
        min{*points },
        max{*points }
    {
        for (size_t n{}; n < count; n++) {
            min.x = glm::min(min.x, points[n].x);
            min.y = glm::min(min.y, points[n].y);
            min.z = glm::min(min.z, points[n].z);

            max.x = glm::max(max.x, points[n].x);
            max.y = glm::max(max.y, points[n].y);
            max.z = glm::max(max.z, points[n].z);
        }
    }

    Aabb::Aabb(const vec3* points, size_t count, const mat4& transformMatrix) :
        Aabb(points, count)
    {
        *this = transform(transformMatrix);
    }

    
    Aabb::Aabb(Aabb const& bv, mat4 const& _transform) :
        min{ bv.min },
        max{ bv.max }
    {
        *this = transform(_transform);
    }

    Aabb::Aabb(Aabb const& lhs, Aabb const& rhs):
        min{lhs.min},
        max{lhs.max}
    {
        min.x = glm::min(lhs.min.x, rhs.min.x);
        min.y = glm::min(lhs.min.y, rhs.min.y);
        min.z = glm::min(lhs.min.z, rhs.min.z);

        max.x = glm::max(lhs.max.x, rhs.max.x);
        max.y = glm::max(lhs.max.y, rhs.max.y);
        max.z = glm::max(lhs.max.z, rhs.max.z);
    }

    Aabb Aabb::transform( mat4 const& m2w) const {

        std::array<vec3, 8> corners{
            min,
            vec3{min.x, max.y, max.z},
            vec3{min.x, min.y, max.z},
            vec3{min.x, max.y, min.z},
            max,
            vec3{max.x, max.y, min.z},
            vec3{max.x, min.y, min.z},
            vec3{max.x, min.y, max.z}
        };

        
        vec3 point = vec3{ m2w * vec4{corners[0], 1.f} };
        Aabb aabb(point,point);

        // transform all corners and get the min and max
        for (size_t n{1}; n < corners.size(); n++) {

            vec3 transformedPoint = vec3{ m2w * vec4{corners[n], 1.f}};
            
            aabb.min = glm::min(aabb.min, transformedPoint);
            aabb.max = glm::max(aabb.max, transformedPoint);

        }
           


        return aabb;
    }

    bool Aabb::intersects( Aabb const& rhs) const {
        //Checks if both AABB intersects
        return (rhs.min.x <= max.x && rhs.max.x >= min.x) &&
            (rhs.min.y <= max.y && rhs.max.y >= min.y) &&
            (rhs.min.z <= max.z && rhs.max.z >= min.z);
    }

    bool Aabb::intersects( vec3 const& pt) const {
        //Checks if points is within AABB
        return (pt.x >= min.x && pt.x <= max.x) &&
            (pt.y >= min.y && pt.y <= max.y) &&
            (pt.z >= min.z && pt.z <= max.z);
    }

    float Aabb::surface_area() const {
        vec3 extend = get_extents();

        return (extend.x * extend.y + extend.x * extend.z + extend.y * extend.z) * 2.f;

    }

    float Aabb::volume() const {
        vec3 extend = get_extents();

        return extend.x * extend.y * extend.z;
    }

    vec3 Aabb::get_extents() const {
        
        return max - min;
    }

    vec3 Aabb::get_center() const{
        return (min + max) * 0.5f;
    }

    int Aabb::longest_axis() const {
        vec3 extend = get_extents();

        // 0 - Xaxis, 1 - Yaxis, 2 - Zaxis

        float longest = glm::max(extend.x, extend.y, extend.z);
        
        if (longest == extend.x) {
            
            return 0;
        }
        
        if (longest == extend.y) {
            return 1;
        }

        return 2;
        
    }

    Triangle::Triangle(vec3 const& a, vec3 const& b, vec3 const& c) :
        points{ a, b, c }
    {}

    vec3 Triangle::closest(vec3 const& pt) const {
        vec3 p1pt = pt - points[1];
        vec3 triangleNormal = normal();
        
        float p1ptLength = glm::length(p1pt);
        float normalLength = glm::length(triangleNormal);
        //find angle between p0p0' and p1
        float cosTheta = glm::dot(p1pt, triangleNormal) / (p1ptLength * normalLength);

        float lengthPtPtprime = p1ptLength * cosTheta;

        vec3 ptPtPrime = -(lengthPtPtprime) * (triangleNormal / normalLength);

        Ray ray(pt, ptPtPrime);
        float t = ray.intersect(*this);


        return ray.start + t * ray.dir;
    }

    void Triangle::invert() {
        std::swap(points[0], points[1]);
    }

    vec3 Triangle::normal() const {
        return glm::cross(points[1] - points[0], points[2] - points[0]);
    }

    vec3 const& Triangle::operator[](int index) const {
        return points[index];
    }

    vec3& Triangle::operator[](int index) {
        return points[index];
    }

    float Triangle::area() const {
        return glm::length(glm::cross(points[1] - points[0], points[2] - points[0]));
    }

    Sphere::Sphere(const vec3& _center, float _radius) :
        center{ _center },
        radius{ _radius }
    {
    }

    bool Sphere::contains(vec3 const& pt) const {
        // Calcualte distance between point and sphere center

        vec3 line = center - pt;
        double sqrDistance = line.x * line.x + line.y * line.y + line.z * line.z;

        // If distance is less than equal to radius, point is inside sphere
        return sqrDistance <= (radius * radius);

    }


    bool Sphere::intersects(Sphere const& rhs) const {

        // Calculate distance between both sphere center
        vec3 line = center - rhs.center;
        double sqrDistance = line.x * line.x + line.y * line.y + line.z * line.z;

        // if distance is less than equal to the addition of both sphere's radius, spheres are intersecting
        return sqrDistance <= (radius * radius);
    }

    Sphere Sphere::centroid( vec3 const* points, size_t count){
        vec3 sumOfPostions{};

        for (size_t n{}; n < count; n++) {
            sumOfPostions += points[n];
        }

        vec3 averagePoint = sumOfPostions / static_cast<float>(count);


        float sqrRadius{};

        for (size_t n{}; n < count; n++) {

            float sqrDistance = glm::distance2(averagePoint, points[n]);

            sqrRadius = glm::max(sqrRadius, sqrDistance);
        }


        return Sphere(averagePoint, glm::sqrt(sqrRadius));
    }

    Sphere Sphere::centroid( vec3 const* points, size_t count, const mat4& transformMatrix) {
        
        std::vector<vec3> transformedPoints;
        transformedPoints.reserve(count);
        for (size_t n{}; n < count; n++) {
            transformedPoints.emplace_back(transformMatrix * vec4{ points[n], 1.f });
        }


        return centroid(transformedPoints.data() , count);
    }

    Sphere Sphere::ritters( vec3 const* points, size_t count) {

        //first for min point, second for max point
        std::pair<vec3, vec3> xLine{ points[0], points[0]};
        std::pair<vec3, vec3> yLine{ points[0], points[0] };
        std::pair<vec3, vec3> zLine{ points[0], points[0] };

        for (size_t n{}; n < count ; n++) {

            const vec3& position = points[n];

            if (position.x < xLine.first.x) {
                xLine.first = position;
            }
            else if (position.x > xLine.second.x) {
                xLine.second = position;
            }

            if (position.y < yLine.first.y) {
                yLine.first = position;
            }
            else if (position.y > yLine.second.y) {
                yLine.second = position;
            }

            if (position.z < yLine.first.z) {
                yLine.first = position;
            }
            else if (position.z > yLine.second.z) {
                yLine.second = position;
            }

        }

        //pick pair with greatest distance
        vec3 sqrAxisDistance{};
        sqrAxisDistance.x = glm::distance2(xLine.first, xLine.second);
        sqrAxisDistance.y = glm::distance2(yLine.first, yLine.second);
        sqrAxisDistance.z = glm::distance2(zLine.first, zLine.second);

        float sqrGreatestDistance = glm::max(sqrAxisDistance.x, sqrAxisDistance.y, sqrAxisDistance.z);

        vec3 circleCenter{};
        //Create circle from the longest axis
        if (sqrGreatestDistance == sqrAxisDistance.x) {

            circleCenter = (xLine.first + xLine.second) * 0.5f;
        }
        else if (sqrGreatestDistance == sqrAxisDistance.y) {
            circleCenter = (yLine.first + yLine.second) * 0.5f;
        }
        else {
            circleCenter = (zLine.first + zLine.second) * 0.5f;
        }

        float radius = glm::sqrt(sqrGreatestDistance) * 0.5f;
        Sphere sphere{ circleCenter, radius };

        //Second Pass
        // If vertex is outside the circle, expand
        for (size_t n{}; n < count; n++) {

            const glm::vec3& point = points[n];

            float sqrDistance = glm::distance2(sphere.center, point);

            // set new gfx sphere if > radius
            if (sqrDistance > (sphere.radius * sphere.radius)) {
                //Calculate center to point
                glm::vec3 vector = (point - sphere.center) / glm::sqrt(sqrDistance);

                //Get point from other side of the sphere
                glm::vec3 pPrime = sphere.center - (sphere.radius * vector);

                sphere.center = (pPrime + point) * 0.5f;
                sphere.radius = glm::distance(sphere.center, point);
            }


        }


        return sphere;
    }

    Sphere Sphere::ritters( vec3 const* points, size_t count, const mat4& transformMatrix) {

        std::vector<vec3> transformedPoints;
        transformedPoints.reserve(count);
        for (size_t n{}; n < count; n++) {
            transformedPoints.emplace_back(transformMatrix * vec4{ points[n], 1.f });
        }

        return ritters(transformedPoints.data(), count);
    }

    Sphere Sphere::iterative( vec3 const* points, size_t count, size_t iteration_count, float shrink_ratio) {
        Sphere finalSphere = ritters(points, count);


        // create a container from 0 to count - 1
        std::vector<int> indices(count);
        for (size_t n{}; n < count; n++) {
            indices[n] = static_cast<int>(n);
        }
            
        std::random_device rd;
        std::mt19937 rng(rd());
       
        for (size_t n{}; n < iteration_count; n++) {
            Sphere newSphere{ finalSphere.center , finalSphere.radius * shrink_ratio };

            // shuffle the container
            std::shuffle(indices.begin(), indices.end(), rng);

            for (int index : indices) {
                const glm::vec3& point = points[index];

                float sqrDistance = glm::distance2(newSphere.center, point);

                // set new gfx sphere if > radius
                if (sqrDistance > (newSphere.radius * newSphere.radius)) {
                    glm::vec3 u = (point - newSphere.center) / glm::sqrt(sqrDistance);
                    glm::vec3 pPrime = newSphere.center - (newSphere.radius * u);

                    glm::vec3 newcenter = (pPrime + point) * 0.5f;

                    newSphere.radius = glm::distance(newcenter, point);

                    newSphere.center = newcenter;
                }

            }

            if (newSphere.radius < finalSphere.radius) {
                finalSphere = newSphere;
            }

        }

        return finalSphere;
    }

    Sphere Sphere::iterative( vec3 const* points, size_t count, size_t iteration_count, float shrink_ratio, const mat4& transformMatrix) {
        std::vector<vec3> transformedPoints;
        transformedPoints.reserve(count);
        for (size_t n{}; n < count; n++) {
            transformedPoints.emplace_back(transformMatrix * vec4{ points[n], 1.f });
        }

        return iterative(transformedPoints.data(), count, iteration_count, shrink_ratio);
    }


    Segment::Segment( vec3 const& start,  vec3 const& end) :
        points{ start, end }
    {}

    vec3 Segment::closest(Segment const& rhs) const {


        vec3 v = points[1] - points[0];
        vec3 w = rhs[1] - rhs[0];
        vec3 k = points[0] - rhs[0];


        // calculate dot product of the paremeter
        float a = glm::dot(v, v);
        float b = glm::dot(v, w);
        float c = glm::dot(w, w);
        float d = glm::dot(v, k);
        float e = glm::dot(w, k);

        float denom = a * c - b*b;

        // current segment
        float t{ 0.f };

        // other segment
        float s{ 0.f };


        // lines are parellel
        if (denom < cEpsilon) {
            if (a > cEpsilon) {
                t = glm::clamp(-d/a,0.f,1.f);
            }
        }
        else{

            t = (b * e - c * d) / denom;
            t = glm::clamp(t, 0.f, 1.f);

            // calculate s using the result in t
            s = (b * t + e) / c;


            // recalcualte t if s is outside [0,1]
            if (s < cEpsilon) {
                t = glm::clamp(-d / a, 0.f, 1.f);
            }
            else if (s > 1.f) {
                t = glm::clamp((b - d) / a, 0.f, 1.f);
            }

        }

        // calculate closest point of the segment
        vec3 closestPoint1 = points[0] + t * v;

        return closestPoint1;
    }

    float Segment::distance(vec3 const& pt) const {
        //Get line from segment start point to pointM
        vec3 segmentPointM = pt - points[0];
        vec3 segmentDir = dir();

        float segmentLengthSquare = glm::dot(segmentDir , segmentDir);

        // return if segment length is 0
        if (segmentLengthSquare < cEpsilon) {
            return glm::length(segmentPointM);
        }

        //apply orthogonal projection
        float t = glm::dot(segmentPointM, segmentDir) / segmentLengthSquare;
        t = glm::clamp(t, 0.f, 1.f);


        // calculate closest point of the segment
        vec3 closestPoint = points[0] + t * segmentDir;

        return glm::length(pt - closestPoint);

    }

    vec3 Segment::at(float t) const {
        return points[0] + dir() * t;
    }

    vec3 Segment::dir() const {
        return points[1] - points[0];
    }

    vec3 const& Segment::operator[](int index) const {
        return points[index];
    }

    vec3& Segment::operator[](int index) {
        return points[index];
    }

    Plane::Plane(Triangle const& triangle):
        //retrieve triangle normal through cross product
        Plane(triangle[0], triangle.normal())
    {}

    Plane::Plane( vec3 const& _normal, float d) :
        normal{_normal},
        dot_result{ d }
    {}

    Plane::Plane( vec3 const& point,  vec3 const& _normal):
        normal{_normal},
        dot_result{glm::dot(_normal, point)}
    {}

    float Plane::distance(vec3 const& pt) const {
        return glm::dot(normal, pt) - dot_result;
    }

    vec3 Plane::closest_point( vec3 const& pt) const {
        vec3 segmentPointM = pt - (dot_result * normal);
        float shortestDistance = glm::dot(segmentPointM, normal);
        return pt - (normal * shortestDistance);

    }

    SideResult Plane::classify( vec3 const& pt) const {

        float distance = glm::dot(normal, pt) - dot_result;


        if (distance < -cEpsilon) {
            return eINSIDE;
        }
        
        if (distance > cEpsilon) {
            return eOUTSIDE;
        }

        return eINTERSECTING;

    }
    SideResult Plane::classify(const Triangle& triangle) const {

        bool isInside = false;
        bool isOutside = false;
        const size_t triangleSides = 3;

        //test plane with all points of triangle
        for (size_t n{}; n < triangleSides; n++) {
            SideResult result = classify(triangle[static_cast<int>(n)]);

            if (result == eINSIDE) {
                isInside = true;
            }
            else if (result == eOUTSIDE) {
                isOutside = true;
            }

            if (isInside && isOutside) {
                return eINTERSECTING;
            }
        }

        return isInside? eINSIDE:eOUTSIDE;
    }
    SideResult Plane::classify(const Aabb& aabb)const {

        
        float halfWidth = (aabb.max.x - aabb.min.x) * 0.5f;
        float halfHeight = (aabb.max.y - aabb.min.y) * 0.5f;
        float halfTickness = (aabb.max.z - aabb.min.z) * 0.5f;

        float radius = halfWidth * glm::abs(normal.x) + halfHeight * glm::abs(normal.y) + halfTickness * glm::abs(normal.z);
        vec3 aabbCenter = (aabb.max + aabb.min) * 0.5f;

        return classify(Sphere(aabbCenter, radius));


    }
    SideResult Plane::classify(const Sphere& sphere) const {


        // Signed distance from the sphere center to the plane
        float distanceToPlane = distance(sphere.center);

        if (distanceToPlane < -sphere.radius) {
            return eINSIDE;
        }
        
        if (distanceToPlane > sphere.radius) {
            return eOUTSIDE;
        }

        return eINTERSECTING;
    }

    vec3 Plane::get_point() const{
        return normal * dot_result;
    }

    Frustum::Frustum(std::array<vec3, 6> const& normals, std::array<float, 6> const& dists):
        planes{ Plane(normals[0], dists[0]), Plane(normals[1], dists[1]),
                 Plane(normals[2], dists[2]), Plane(normals[3], dists[3]),
                 Plane(normals[4], dists[4]), Plane(normals[5], dists[5]) }
     {}

    Frustum::Frustum(mat4 const& viewProj):
        planes{ Plane(vec3(viewProj[0][3] + viewProj[0][0], viewProj[1][3] + viewProj[1][0], //Left
                            viewProj[2][3] + viewProj[2][0]),viewProj[3][3] + viewProj[3][0]),
                 Plane(vec3(viewProj[0][3] - viewProj[0][0], viewProj[1][3] - viewProj[1][0], //Right
                            viewProj[2][3] - viewProj[2][0]),viewProj[3][3] - viewProj[3][0]),
                 Plane(vec3(viewProj[0][3] + viewProj[0][1], viewProj[1][3] + viewProj[1][1], //Bottom
                            viewProj[2][3] + viewProj[2][1]),viewProj[3][3] + viewProj[3][1]),
                 Plane(vec3(viewProj[0][3] - viewProj[0][1], viewProj[1][3] - viewProj[1][1], //Top
                            viewProj[2][3] - viewProj[2][1]),viewProj[3][3] - viewProj[3][1]),
                 Plane(vec3(viewProj[0][3] + viewProj[0][2], viewProj[1][3] + viewProj[1][2],//Near 
                            viewProj[2][3] + viewProj[2][2]),viewProj[3][3] + viewProj[3][2]), 
                 Plane(vec3(viewProj[0][3] - viewProj[0][2], viewProj[1][3] - viewProj[1][2], //Far
                            viewProj[2][3] - viewProj[2][2]),viewProj[3][3] - viewProj[3][2])}
    {
        for (Plane& plane : this->planes) {
            //add "-" to invert normal
            plane.normal *= -1;
        }
        
    }

    SideResult Frustum::classify(Sphere const& sphere)const {

        bool isInside = true;
        SideResult result{};

        // test Sphere to every plane on the frustrum
        for (const Plane& plane : this->planes) {

            result = plane.classify(sphere);

            if (result == eOUTSIDE) {
                return eOUTSIDE;
            }
            if (result == eINTERSECTING) {
                isInside = false;
            }
        }

        return isInside ? eINSIDE : eINTERSECTING;
    }


    SideResult Frustum::classify(Aabb const& aabb) const {

        //update stats
		CS350::Stats::Instance().frustumVsAabb++;

        bool isInside = true;
        SideResult result{};

        // test AABB to every plane on the frustrum
        for (const Plane& plane : this->planes) {

            result = plane.classify(aabb);

            if (result == eOUTSIDE) {
                return eOUTSIDE;
            }
            
            if (result == eINTERSECTING) {
                isInside = false;
            }
        }

        return isInside ? eINSIDE : eINTERSECTING;
    }

    Ray::Ray(vec3 const& _start, vec3 const& _dir) :
        start{ _start },
        dir{ _dir }
     {}

    vec3 Ray::at(float t) const {
        return start + t * dir;
    }

    float Ray::intersect(Plane const& plane) const{



        float denominator = glm::dot(plane.normal, dir);

        //No intersection if denominator < 0
        if (glm::abs(denominator) < cEpsilon) {
             return -1.f;
        }
        

        vec3 planePoint = plane.normal * plane.dot_result;
        float t = glm::dot(planePoint - start, plane.normal) / denominator;

        // -1 means no collision
        return t < cEpsilon ? -1.f : t;
    }

    float Ray::intersect(Aabb const& aabb)const {

        //update stats
        CS350::Stats::Instance().rayVsAabb++;

        if (aabb.intersects(start)) {
            return 0.f;
        }


        // retrieve slabs intersection ranges for all axis
        float tx1 = (aabb.min.x - start.x) / dir.x;
        float tx2 = (aabb.max.x - start.x) / dir.x;

        float ty1 = (aabb.min.y - start.y) / dir.y;
        float ty2 = (aabb.max.y - start.y) / dir.y;

        float tz1 = (aabb.min.z - start.z) / dir.z;
        float tz2 = (aabb.max.z - start.z) / dir.z;

        //test if ranges overlaps
        float minRange = glm::max(glm::max(glm::min(tx1, tx2), glm::min(ty1,ty2)), glm::min(tz1,tz2));
        float maxRange = glm::min(glm::min(glm::max(tx1, tx2), glm::max(ty1, ty2)), glm::max(tz1, tz2));


        //AABB is behind or ray does not interact
        if (maxRange < cEpsilon || minRange > maxRange) {
            return -1.f;
        }

        return minRange;
    }

    float Ray::intersect(Sphere const& sphere) const {

        // checks if ray start inside sphere
        if (sphere.contains(start)) {
            return 0.f;
        }


        //Calculate paremeters to compute discriminant
        float a = glm::dot(dir, dir);
        float b = 2.f * (glm::dot(dir, start - sphere.center));
        float c = glm::dot(start - sphere.center, start - sphere.center) - (sphere.radius * sphere.radius);

        float sqrtDiscriminant = (b*b) - (4.f * a * c);


        // no solution
        if (sqrtDiscriminant < cEpsilon) {
            return -1.f;
        }

        // 1 solution
        if (glm::abs(sqrtDiscriminant) < cEpsilon) {
            return -b / (2.f * a);
        }
        
        //2 solutions
        float discriminant = glm::sqrt(sqrtDiscriminant);
        float t1 = (-b + discriminant) / (2.f * a);
        float t2 = (-b - discriminant) / (2.f * a);

        if (t1 > cEpsilon && t2 > cEpsilon) {
            return glm::min(t1, t2);
        }
        if (t1 > cEpsilon) {
            return t1;
        }
        if (t2 > cEpsilon) {
            return t2;
        }

        // no intersection
        return -1.f; 
    }

    float Ray::intersect(Triangle const& triangle) const {
        
        // test intersection of triangle plane
        vec3 triangleNormal = glm::cross(triangle[1] - triangle[0], triangle[2] - triangle[0]);
        Plane trianglePlane(triangle[0], triangleNormal);

        float planeIntersection = intersect(trianglePlane);

        // If ray does not intersect triangle plane, return -1
        if (planeIntersection < cEpsilon) {
            return -1.f;
        }

        //find intersection point
        vec3 intersectionPoint = start + planeIntersection * dir;

        //Calcualte vector for 2 lines on the triangle
        vec3 v = triangle[1] - triangle[0];
        vec3 w = triangle[2] - triangle[0];

        //Calculate vector between intersection point and point on triangle
        vec3 u = intersectionPoint - triangle[0];

        //Calculate paremeter for equations
        float a = glm::dot(v, v);
        float b = glm::dot(w, v);
        float c = glm::dot(v, w);
        float d = glm::dot(w, w);
        float e = glm::dot(u, v);
        float f = glm::dot(u, w);

        // Apply Cramer's rule
        float denom = glm::determinant(glm::mat2x2(a, c, b, d));

        // Ray start point is on the triangle
        if (denom < cEpsilon) {
            return 0.f;
        }

        float s = glm::determinant(glm::mat2x2(e, f, b, d)) / denom;
        float t = glm::determinant(glm::mat2x2(a, c, e, f)) / denom;

        if (s >= 0.f && t >= 0.f) {
            if ((s + t) >= 0.f && (s + t) <= 1.f) {
                return planeIntersection;
            }
        }

        // point is outside the triangle
        return -1;

    }

}