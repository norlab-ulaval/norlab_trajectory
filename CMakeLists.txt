cmake_minimum_required(VERSION 3.8)
project(norlab_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -march=native)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(PYTHON_EXECUTABLE /usr/bin/python3.10)

find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package (steam REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pybind11 2.5.0 REQUIRED)
find_package (Python3 COMPONENTS Interpreter Development)

include_directories(include)

add_library(norlab_trajectory src/Trajectory.cpp)
ament_target_dependencies(norlab_trajectory steam)
target_link_libraries(norlab_trajectory Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_export_dependencies(norlab_trajectory steam)

add_executable(main src/main.cpp)
target_link_libraries (main norlab_trajectory)
install(TARGETS norlab_trajectory main
  DESTINATION lib/${PROJECT_NAME}
)

if (pybind11_FOUND)
  ament_python_install_package(${PROJECT_NAME})
  pybind11_add_module(pynorlab_trajectory src/bindings.cpp)
  target_link_libraries(pynorlab_trajectory PUBLIC norlab_trajectory)
  install(TARGETS pynorlab_trajectory
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
else ()
  message(FATAL_ERROR "pybind11 is required!")
endif ()

ament_package()
