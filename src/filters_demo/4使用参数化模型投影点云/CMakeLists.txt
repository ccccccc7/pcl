cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(project_inliers)

find_package(PCL 1.12 REQUIRED)

# VTK相关
find_package(VTK REQUIRED)
#不加初始化会出错
include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (project_inliers project_inliers.cpp) # 生成可执行文件 （工程名 cpp文件名）
target_link_libraries (project_inliers ${PCL_LIBRARIES}) # 工程名