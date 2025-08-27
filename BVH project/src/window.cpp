#include "window.hpp"
#include "utils.hpp"

#include <GLFW/glfw3.h>
#include <iostream>
#include <stdexcept>
#include <cassert>

/**
 * OpenGL callback for debugging
 */
void APIENTRY openglCallbackFunction(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei /* length */, const GLchar* message, const void* /* userParam */) {
    // Ignore notifications
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) {
        return;
    }

    std::cout << "{" << std::endl;
    std::cout << "\tsource: " << source << std::endl;
    std::cout << "\tmessage: " << message << std::endl;
    std::cout << "\ttype: ";
    switch (type) {
        case GL_DEBUG_TYPE_ERROR:
            std::cout << "ERROR";
            break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            std::cout << "DEPRECATED_BEHAVIOR";
            break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            std::cout << "UNDEFINED_BEHAVIOR";
            break;
        case GL_DEBUG_TYPE_PORTABILITY:
            std::cout << "PORTABILITY";
            break;
        case GL_DEBUG_TYPE_PERFORMANCE:
            std::cout << "PERFORMANCE";
            break;
        case GL_DEBUG_TYPE_OTHER:
            std::cout << "OTHER";
            break;
        default: break;
    }
    std::cout << std::endl;

    std::cout << "\tid: " << id << std::endl;
    std::cout << "\tseverity: ";
    switch (severity) {
        case GL_DEBUG_SEVERITY_LOW:
            std::cout << "LOW";
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            std::cout << "MEDIUM";
            break;
        case GL_DEBUG_SEVERITY_HIGH:
            std::cout << "HIGH";
            break;
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            std::cout << "NOTIFICATION";
            break;
        default: break;
    }
    std::cout << std::endl
              << "}" << std::endl;
    assert(type != GL_DEBUG_TYPE_ERROR);
}

/**
 * Enables OpenGL callbacks. This callbacks will be useful to intercept errors.
 */
void enable_callbacks() {
    // Debug
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(openglCallbackFunction, nullptr);
    GLuint unused_ids = 0;
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, &unused_ids, GL_TRUE);
}

namespace CS350 {

    Window::Window(ivec2 size, bool visible) {
        // Create window
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        m_window = glfwCreateWindow(size.x, size.y, "CS300", nullptr, nullptr);
        if (m_window == nullptr) {
            char const* desc = nullptr;
            int         code = glfwGetError(&desc);
            throw std::runtime_error(fmt::format("Could not create window: {} ({})", code, desc));
        }

        // Needs to setup the context of the window to initialize Glad
        glfwMakeContextCurrent(m_window);

        // Initialize Glad (in the current context)
        if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0) {
            throw std::runtime_error("Could not load GLAD");
        }

        if (GLAD_GL_VERSION_4_4 == 0) {
            throw std::runtime_error("Incorrect OpenGL version");
        }
        enable_callbacks();

        // To have valid size from the beginning
        glfwGetWindowSize(m_window, &m_size.x, &m_size.y);
    }

    Window::~Window() {
        glfwDestroyWindow(m_window);
    }

    void Window::update() {
        glfwPollEvents();
        glfwGetWindowSize(m_window, &m_size.x, &m_size.y);
        glfwSwapBuffers(m_window);
    }

    bool Window::should_exit() {
        return glfwWindowShouldClose(m_window) != 0 || glfwGetKey(m_window, GLFW_KEY_ESCAPE);
    }

    void Window::initialize_system() {
        if (glfwInit() == GLFW_NO_ERROR) {
            char const* desc = nullptr;
            int         code = glfwGetError(&desc);
            throw std::runtime_error(fmt::format("Could not create window: {} ({})", code, desc));
        }
    }

    void Window::destroy_system() {
        glfwTerminate();
    }
}
