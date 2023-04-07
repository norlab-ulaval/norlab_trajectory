#include "trajectory.h"
#include "Trajectory.h"
#include <pybind11/pybind11.h>

void pybindMatches()
{
    pybind11::class_<Trajectory> pyMatches("Trajectory");

    pyMatches.doc() = R"pbdoc(
Result of the data-association step (Matcher::findClosests), before outlier rejection.

This class holds a list of associated reference identifiers, along with the corresponding \e squared distance, for all points in the reading.
A single point in the reading can have one or multiple matches.
)pbdoc";
    pyMatches
            .def(pybind11::init<>()).def(pybind11::init<std::pair <float, Eigen::Matrix4f>>(), pybind11::arg("poses"))
            .def("getPose", &Trajectory::getPose, pybind11::arg("time"))
            .def("getPoseCovariance", &Trajectory::getPoseCovariance, pybind11::arg("time"));
}
