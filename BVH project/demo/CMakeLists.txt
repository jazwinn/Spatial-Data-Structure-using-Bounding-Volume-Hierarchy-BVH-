cmake_minimum_required(VERSION 3.8)
project(cs350-demo)

# Demo
add_executable(${PROJECT_NAME}       
        main.cpp
        DemoScene.hpp DemoScene.cpp
        ImGui.hpp ImGui.cpp
        camera.cpp
        camera.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC .)

# Libraries
target_link_libraries(${PROJECT_NAME} PUBLIC cs350-engine)

# ImGui
find_package(imgui CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui::imgui)

# ImGuizmo
find_package(imguizmo CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE imguizmo::imguizmo)