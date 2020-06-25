cmake_minimum_required(VERSION 3.5)
project(useEigen)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
include_directories("/usr/include/eigen3")
add_executable(eigenMatrix eigenMatrix.cpp)
