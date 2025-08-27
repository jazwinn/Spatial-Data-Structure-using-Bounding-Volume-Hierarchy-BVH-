#include "DemoScene.hpp"
#include "ImGui.hpp"
#include "utils.hpp"
#include "opengl.hpp"
#include "window.hpp"
#include "camera.hpp"
#include <chrono>

namespace {
    // Camera controls
    float cCameraSpeed = 10.0f;
    void  ScrollCallback(GLFWwindow* /*window*/, double /*xoffset*/, double yoffset) {
        if (!ImGui::GetIO().WantCaptureMouse) {
            cCameraSpeed += (float)yoffset * 4.0f;
        }
    }

    void update_camera(GLFWwindow* window, float dt, Camera& camera) {
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        double cursor_x = 0.0;
        double cursor_y = 0.0;
        glfwGetCursorPos(window, &cursor_x, &cursor_y);

        // Speed change
        float speed = cCameraSpeed;

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) != 0) {
            auto side = glm::normalize(glm::cross(camera.camera_dir, { 0, 1, 0 }));
            auto up   = glm::normalize(glm::cross(camera.camera_dir, side));

            if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) != 0) {
                speed *= 2.0f;
            }
            // Move
            if (glfwGetKey(window, GLFW_KEY_W) != 0) {
                camera.camera_position += glm::normalize(camera.camera_dir) * dt * speed;
            }
            if (glfwGetKey(window, GLFW_KEY_S) != 0) {
                camera.camera_position -= glm::normalize(camera.camera_dir) * dt * speed;
            }
            if (glfwGetKey(window, GLFW_KEY_A) != 0) {
                camera.camera_position -= glm::normalize(side) * dt * speed;
            }
            if (glfwGetKey(window, GLFW_KEY_D) != 0) {
                camera.camera_position += glm::normalize(side) * dt * speed;
            }
            if (glfwGetKey(window, GLFW_KEY_SPACE) != 0) {
                camera.camera_position -= up * dt * speed;
            }
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) != 0) {
                camera.camera_position += up * dt * speed;
            }
            // View
            glm::vec2 cursor_delta   = { (float)cursor_x - camera.cursor_pos.x, (float)cursor_y - camera.cursor_pos.y };
            float     rotation_speed = 0.01f;
            camera.camera_dir        = glm::vec3(glm::vec4(camera.camera_dir, 0) * glm::rotate(cursor_delta.y * rotation_speed, side));
            camera.camera_dir        = glm::vec3(glm::vec4(camera.camera_dir, 0) * glm::rotate(cursor_delta.x * rotation_speed, glm::vec3(0, 1, 0)));
        }
        camera.display_w = static_cast<float>(display_w);
        camera.display_h = static_cast<float>(display_h);
        //Do this to prevent crash when applicaation is minisimized
        if (camera.display_h > 0.f) {
            camera.compute_matrices();
        }

        camera.cursor_pos = { cursor_x, cursor_y };
    }
}

int main() {
    CS350::ChangeWorkdir("bin");
    CS350::Window::initialize_system();
    CS350::Window w({ 1920, 1080 });
    ImGuiInitialize(w.handle());
    glfwSetScrollCallback(w.handle(), ScrollCallback);
    glfwSwapInterval(0); // Disable VSync
    {                    // Scene
        CS350::DemoScene scene;
        // Camera setup
        auto& cam           = scene.camera();
        cam.camera_position = { 0, 0, 10 };
        cam.fov_deg         = 60.0f;
        cam.display_w       = static_cast<float>(w.size().x);
        cam.display_h       = static_cast<float>(w.size().y);
        cam.near            = 0.01f;
        cam.far             = 99999.0f;

        auto& auxCam = scene.AuxCamera();
        auxCam.camera_position = { 0, 0, 10 };
        auxCam.fov_deg = 60.0f;
        auxCam.display_w = static_cast<float>(w.size().x);
        auxCam.display_h = static_cast<float>(w.size().y);
        auxCam.near = 0.01f;
        auxCam.far = 99999.0f;
        // mCamera.SetPosition({ -12.063, -52.3396, -25.3407 });
        // mCamera.SetTarget({ 0.479789, -3.10705, -3.16863 });

        // cam.SetPosition({ -9.13937, -20.5272, -41.7185 });
        // cam.SetTarget({ -9.608, 4.99564, 0.156429 });
        // cam.SetProjection(50.0f, { 1920.0f, 1080.0f }, 0.01f, 1000.0f);

        while (!w.should_exit()) {
            // dt
            static auto lastTime = std::chrono::high_resolution_clock::now();
            float       dt       = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - lastTime).count();
            lastTime             = std::chrono::high_resolution_clock::now();

            w.update();

            // General rendering states
            ivec2 windowSize = w.size();
            glViewport(0, 0, windowSize.x, windowSize.y);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClearDepth(1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Camera
            if (scene.AuxCameraMain()) {
                update_camera(w.handle(), dt, auxCam);
            }
            else {
                update_camera(w.handle(), dt, cam);
            }
            
            


            scene.Update();

            // Debug
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            scene.PassDebug();
            ImGuiNewFrame();
            if (ImGui::Begin("Options")) {
                scene.ImguiOptions(dt);
            }
            ImGui::End();
            ImGuiEndFrame();
        }

    }

    CS350::Window::destroy_system();
    return 0;
}
