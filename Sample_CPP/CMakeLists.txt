cmake_minimum_required(VERSION 3.1.3)
project(FZMotionReceiveDLL_Sample)

# C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# 添加SDK头文件和库目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/LuMoSDK/include
)
link_directories(${PROJECT_SOURCE_DIR}/LuMoSDK/lib)

# ROS（Noetic）查找与头文件目录
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs)
include_directories(${catkin_INCLUDE_DIRS})

# 可执行文件源码（如果在 src 目录下，注意路径）
add_executable(LuMoSDK_Sample src/FZMotion_Receive_Sample.cpp)

# 链接库
target_link_libraries(LuMoSDK_Sample
    LuMoSDK
    ${catkin_LIBRARIES}
)

link_directories(${PROJECT_SOURCE_DIR}/LuMoSDK/lib)

# rpath设置（可选，便于运行时找动态库）
set_target_properties(LuMoSDK_Sample PROPERTIES
    BUILD_RPATH "$ORIGIN/../LuMoSDK/lib"
)

