#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

PYBIND11_MODULE(pypointmatcher, module)
{
module.doc() = "Python bindings of libpointmatcher";

python::modules::pybindPointMatcherModule(module);
}


void pybindPointMatcher(py::module& p_module)
{
    py::class_<PM> pyPointmatcher(p_module, "PointMatcher");

    pyPointmatcher
            .doc() = "Functions and classes that are dependant on scalar type are defined in this templatized class";

    pybindMatches(pyPointmatcher);
}
