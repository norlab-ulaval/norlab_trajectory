include_directories(/home/effie/ros2_ws/install/steam/include)
include_directories(/home/effie/ros2_ws/install/lgmath/include)

find_package(pybind11 2.5.0 REQUIRED)
find_package (Python3 COMPONENTS Interpreter Development)
find_package (steam REQUIRED)

if (pybind11_FOUND)
    pybind11_add_module(trajectory bindings.cpp)
    add_dependencies(trajectory norlab_trajectory)
    target_link_libraries(trajectory PUBLIC /home/effie/ros2_ws/install/lgmath/lib/liblgmath.a)
    target_link_libraries(trajectory PUBLIC /home/effie/ros2_ws/install/steam/lib/libsteam.a)
    target_link_libraries(trajectory PUBLIC ${CMAKE_BINARY_DIR}/libnorlab_trajectory.a)
    target_link_libraries(trajectory PUBLIC OpenMP::OpenMP_CXX)
else ()
    message(FATAL_ERROR "pybind11 is required! Please follow the \"Compiling \
norlab_trajectory with Python\" instructions from the official norlab_trajectory documentation.")
endif ()
