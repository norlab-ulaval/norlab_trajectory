#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include "Trajectory.h"

namespace py = pybind11;

PYBIND11_MODULE(pynorlab_trajectory, trajectory_module_handle)
{
    trajectory_module_handle.doc() = "Python bindings of Trajectory";

    py::class_<Trajectory>(trajectory_module_handle, "Trajectory")
            .def(py::init<std::vector<std::pair<float, Eigen::Matrix4f>>>(), py::arg("poses"))
            .def("getPose", &Trajectory::getPose, py::arg("time"))
            .def("getPoseCovariance", &Trajectory::getPoseCovariance, py::arg("time"));
}