cmake_minimum_required(VERSION 3.11)
project(cs350-engine)

# Engine library
add_library(
        ${PROJECT_NAME}
        bvh.hpp
        bvh.inl
        logging.cpp
        logging.hpp
        math.hpp
        shapes.hpp
        shapes.cpp
        utils.cpp
        utils.hpp
        debug_renderer.hpp
        debug_renderer.cpp
        opengl.hpp
        opengl.cpp
        primitive.hpp
        primitive.cpp
        shader.cpp
        shader.hpp
        window.hpp
        window.cpp
        cs350_loader.hpp
        cs350_loader.cpp
        stats.hpp
        PRNG.cpp
        PRNG.h)
target_include_directories(${PROJECT_NAME} PUBLIC .)

# GLM
find_package(glm CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC glm::glm)

# fmt
find_package(fmt CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC fmt::fmt)

# ImGui
find_package(imgui CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui::imgui)

# ImGuizmo
find_package(imguizmo CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE imguizmo::imguizmo)

# GLFW3
find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC glfw)

# GLAD
find_package(glad CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC glad::glad)
