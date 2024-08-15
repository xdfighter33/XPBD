cmake_minimum_required(VERSION 3.20)
set(PROJECT_NAME 3D-Verlet)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(FetchContent)

# Function to declare content with local fallback
function(declare_local_or_remote NAME LOCAL_DIR REPO_URL TAG)
    if(EXISTS ${LOCAL_DIR})
        FetchContent_Declare(
            ${NAME}
            SOURCE_DIR ${LOCAL_DIR}
        )
    else()
        message(STATUS "${NAME} not found locally, fetching from GitHub")
        FetchContent_Declare(
            ${NAME}
            GIT_REPOSITORY ${REPO_URL}
            GIT_TAG ${TAG}
        )
    endif()
endfunction()

# Declare libraries
declare_local_or_remote(glfw ${CMAKE_CURRENT_SOURCE_DIR}/LIBS/glfw https://github.com/glfw/glfw.git 3.3.8)
declare_local_or_remote(glad ${CMAKE_CURRENT_SOURCE_DIR}/LIBS/glad https://github.com/Dav1dde/glad.git v0.1.36)
declare_local_or_remote(glm ${CMAKE_CURRENT_SOURCE_DIR}/LIBS/glm https://github.com/g-truc/glm.git 0.9.9.8)
declare_local_or_remote(assimp ${CMAKE_CURRENT_SOURCE_DIR}/LIBS/assimp https://github.com/assimp/assimp.git v5.2.5)
declare_local_or_remote(imgui ${CMAKE_CURRENT_SOURCE_DIR}/LIBS/imgui https://github.com/ocornut/imgui.git v1.89.2)

FetchContent_MakeAvailable(glfw glad glm assimp imgui)

# Add an executable
add_executable(${PROJECT_NAME} 
   src/main.cpp
   src/Model_loading/Mesh.hpp
   src/Model_loading/Models.hpp
   src/Shader/shader_loader.h
   src/Camera/camera.hpp
   src/Physics/points.hpp
   src/Physics/SoftBodyPBD.hpp
   src/Physics/SoftBodyXPBD.hpp
)

# Include ImGui
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

# Add ImGui source files to the target
target_sources(${PROJECT_NAME} PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp 
    ${imgui_SOURCE_DIR}/imgui_demo.cpp 
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

# Copy shader files to the build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/SHADER DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
get_filename_component(SHADER_ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/SHADER" ABSOLUTE)
target_compile_definitions(${PROJECT_NAME} PRIVATE SHADER_DIR="${SHADER_ABSOLUTE_PATH}")

# Copy model files to the build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
get_filename_component(MODELS_ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models" ABSOLUTE)
target_compile_definitions(${PROJECT_NAME} PRIVATE MODELS_DIR="${MODELS_ABSOLUTE_PATH}")

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE glfw glad glm::glm assimp)