if("${PYTHON_INSTALL_TARGET}" STREQUAL "")
    message(STATUS "PYTHON_INSTALL_TARGET variable is not set, setting a default value...")
    execute_process(COMMAND "python3" "-c" "import site; print(site.getsitepackages()[0])"
            OUTPUT_VARIABLE PYTHON_INSTALL_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

set(PYBIND11_SOURCES
        bindings.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(/home/effie/ros2_ws/install/steam/include)
include_directories(/home/effie/ros2_ws/install/lgmath/include)

find_package(pybind11 2.5.0 REQUIRED)
find_package (Python3 COMPONENTS Interpreter Development)
find_package (steam REQUIRED)
find_package(OpenMP)

if (pybind11_FOUND)
    pybind11_add_module(trajectory ${PYBIND11_SOURCES})
    target_link_libraries(trajectory PUBLIC /home/effie/ros2_ws/install/lgmath/lib/liblgmath.a)
    target_link_libraries(trajectory PUBLIC /home/effie/ros2_ws/install/steam/lib/libsteam.a)
    target_link_libraries(trajectory PUBLIC /home/effie/ros2_ws/install/norlab_trajectory/lib/norlab_trajectory/libnorlab_trajectory.a)
    target_link_libraries(trajectory PUBLIC OpenMP::OpenMP_CXX)
else ()
    message(FATAL_ERROR "pybind11 is required! Please follow the \"Compiling \
norlab_trajectory with Python\" instructions from the official norlab_trajectory documentation.")
endif ()