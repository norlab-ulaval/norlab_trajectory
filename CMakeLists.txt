cmake_minimum_required(VERSION 3.8)
project(norlab_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

include_directories(include)

add_executable(main src/main.cpp src/Trajectory.cpp)
target_link_libraries (main Eigen3::Eigen)

ament_package()
