cmake_minimum_required(VERSION 3.1)
project(Realsense_camera)

find_package(OpenCV REQUIRED)
add_definitions(-w)

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

#需要glfw插件
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

#指定cmake使用的OpenGL 版本
if (POLICY CMP0072)
  set(OpenGL_GL_PREFERENCE LEGACY)
endif()

#设置可执行文件保存路径 
# SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Debug)

SET(PROJECT_ROOT_PATH "${PROJECT_SOURCE_DIR}")
# SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")
link_directories(${PROJECT_BINARY_DIR}/lib/)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/rs_camera)
add_subdirectory(rs_camera)
aux_source_directory(./rs_camera CAMERA_SRCS)

#添加opengl，glfw的目录
include_directories(
        ${OPENGL_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIRS}
)

add_executable(
  Realsense_camera
  example/main.cpp
  ${CAMERA_SRCS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(Realsense_camera PUBLIC ${OpenCV_LIBS} -lpthread -lm)

set(DEPENDENCIES realsense2 )
target_link_libraries(Realsense_camera PUBLIC  ${DEPENDENCIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})
