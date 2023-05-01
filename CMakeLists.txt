cmake_minimum_required(VERSION 3.16)
project(norlab_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -march=native)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -fPIC")
set(PYTHON_EXECUTABLE /usr/bin/python3.10)

find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package (steam REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(norlab_trajectory src/Trajectory.cpp)
ament_target_dependencies(norlab_trajectory Eigen3 steam Boost)

add_executable(main src/main.cpp)
target_link_libraries (main norlab_trajectory OpenMP::OpenMP_CXX)
add_subdirectory(python)

install(TARGETS norlab_trajectory main
        DESTINATION lib/${PROJECT_NAME})

ament_package()