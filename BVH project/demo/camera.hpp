#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "math.hpp"
#include "shapes.hpp"
struct Camera {
    glm::vec3 camera_position = { 5, 5, 5 };
    glm::vec3 camera_dir      = glm::vec3(-1, -1, -1);
    glm::vec2 cursor_pos      = {};
    float     near            = {};
    float     far             = {};
    glm::mat4 v               = {};
    glm::mat4 p               = {};
    glm::mat4 vp              = {};
    float     fov_deg         = 70.0f;
    float     display_w       = 0.0f;
    float     display_h       = 0.0f;

    void compute_matrices() {
        p  = glm::perspective(glm::radians(fov_deg), display_w / display_h, near, far);
        v  = glm::lookAt(camera_position, camera_position + camera_dir, glm::vec3(0, 1, 0));
        vp = p * v;
    }

    CS350::Ray cursor_ray() const {
        vec3 ray_origin = camera_position;
        vec4 ray_target = glm::inverse(vp) * glm::vec4(cursor_pos.x / display_w * 2.0f - 1.0f, -(cursor_pos.y / display_h * 2.0f - 1.0f), 1.0f, 1.0f);
        ray_target /= ray_target.w;
        vec3 ray_dir = glm::normalize(vec3(ray_target) - ray_origin);
        return CS350::Ray(ray_origin, ray_dir);
    }
};

#endif // CAMERA_HPP
