cmake_minimum_required(VERSION 3.20)
project(Collisions LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glm CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(sdl2-gfx CONFIG REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/App.cpp
    src/Object.cpp
    src/Grid.cpp
    src/CSpaceRefiner.cpp
    src/imgui_backends/imgui_impl_sdl2.cpp
    src/imgui_backends/imgui_impl_sdlrenderer2.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include src/imgui_backends)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    glm::glm
    imgui::imgui
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    SDL2::SDL2_gfx
)
