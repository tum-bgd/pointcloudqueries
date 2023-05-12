#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include<boost/geometry/geometry.hpp>

#include "mpcl.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;



PYBIND11_MODULE(pointcloudqueries, m) {
  
  pybind11::class_<mpcl::pointcloud>(m, "pointcloud")
        .def(pybind11::init())
    .def("add", [](mpcl::pointcloud &self, py::array_t<double, py::array::c_style | py::array::forcecast> pcl)
    {
     auto r = pcl.unchecked<2>();
      for (py::ssize_t i = 0; i < r.shape(0); i++)
        
	{
	  if (r.shape(1) != 3) throw(std::runtime_error("Expect Nx3 matrix"));
	  self.cloud.push_back(point(r(i, 0),r(i,  1),r(i,  2)));
	}
    })
    .def("size",[](const mpcl::pointcloud &self) {
      return self.cloud.size();
    })
    .def("index",[](mpcl::pointcloud &self){
      self.buildIndex();
    })
    .def("nndistancemap",[](mpcl::pointcloud &self){
      auto result = self.nndistancemap(); // this needs to be refined to fit your query
      
      return py::array_t<double>(result.size(), result.data());
    })
    ;
    m.doc() = R"pbdoc(
        pointcloudqieries
        -----------------------

        .. currentmodule:: python_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
