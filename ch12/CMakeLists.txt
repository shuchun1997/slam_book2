cmake_minimum_required(VERSION 3.5)
project(ch12)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")

#find_package(OpenCV 3 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})

#include_directories("/usr/include/eigen3")

#add_executable(dense_mapping dense_mapping.cpp)
#target_link_libraries(dense_mapping ${OpenCV_LIBS} ${Sophus_LIBRARIES})

add_subdirectory(dense_mono)
add_subdirectory(dense_RGBD)
