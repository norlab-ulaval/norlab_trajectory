cmake_minimum_required(VERSION 3.8)
project(norlab_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package (steam)
find_package(Boost COMPONENTS system REQUIRED)
find_package(pybind11 2.5.0 REQUIRED)

include_directories(include)

add_library(norlab_trajectory src/Trajectory.cpp)
ament_target_dependencies(norlab_trajectory Eigen3 steam Boost)

pybind11_add_module(pypointmatcher python/bindings.cpp)

add_executable(main src/main.cpp)
target_link_libraries (main norlab_trajectory)

ament_package()
