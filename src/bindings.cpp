#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include "Trajectory.h"

namespace py = pybind11;

PYBIND11_MODULE(pynorlab_trajectory, trajectory_module_handle)
{
    trajectory_module_handle.doc() = "Python bindings of Trajectory";

    py::class_<norlab_trajectory::Trajectory>(trajectory_module_handle, "Trajectory")
            .def(py::init<std::vector<double>, std::vector<Eigen::Matrix4f>, std::vector<Eigen::Matrix<float, 6, 6>>>(), py::arg("timeStamps"), py::arg("poses"),
                 py::arg("covariances"))
            .def("getPose", &norlab_trajectory::Trajectory::getPose, py::arg("time"))
            .def("getPoseCovariance", &norlab_trajectory::Trajectory::getPoseCovariance, py::arg("time"));
}