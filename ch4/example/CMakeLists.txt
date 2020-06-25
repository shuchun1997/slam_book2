cmake_minimum_required( VERSION 3.5 )
project(trajectoryError)
set(CMAKE_CXX_FLAGS  "-std=c++11 ")
set(CMAKE_BUILD_TYPE "Debug")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
