#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "mpcl2.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

template<typename pointcloud_type, typename module_type>
void register_class_with_name(module_type m, std::string name)
{
 pybind11::class_<pointcloud_type>(m, name.c_str())
    .def(pybind11::init())
    .def("add", [](pointcloud_type &self, py::array_t<double, py::array::c_style | py::array::forcecast> pcl)
    {
     auto r = pcl.unchecked<2>();
      for (py::ssize_t i = 0; i < r.shape(0); i++)
        
	{
	  if (r.shape(1) != 3) throw(std::runtime_error("Expect Nx3 matrix"));
	  self.cloud.push_back(typename pointcloud_type::point_type(r(i, 0),r(i,  1),r(i,  2)));
	}
    })
    .def("size",[](const pointcloud_type &self) {
      return self.cloud.size();
    })
    .def("index",[](pointcloud_type &self){
      self.buildIndex();
    })
    .def("knn_radius",[](pointcloud_type &self, std::string name, int k){
        auto ret = self.computeKNNradius(name,k);
    })
    .def ("get_attrib",[](pointcloud_type &self, std::string name){
       auto result = self.get_attrib_by_name(name);
       return py::array_t<double>(result.size(), result.data()); 

    })
    .def("eigenfeatures",[](pointcloud_type &self, std::string prefix, int k){
	self.map_kNN_features(k, prefix);
    })


    ; // keep on line to make new methods easier to add.
 

};


PYBIND11_MODULE(pointcloudqueries, m) {
    register_class_with_name<pointcloud<double,3>>(m,"pointcloud3d");
    register_class_with_name<pointcloud<double,4>>(m,"pointcloud4d");
    register_class_with_name<pointcloud<double,5>>(m,"pointcloud5d");

    register_class_with_name<pointcloud<float,3>>(m,"pointcloud3f");
    register_class_with_name<pointcloud<float,4>>(m,"pointcloud4f");
    register_class_with_name<pointcloud<float,5>>(m,"pointcloud5f");

/*  .def("nndistancemap",[](mpcl::pointcloud &self){
      auto result = self.nndistancemap(); // this needs to be refined to fit your query
      
      return py::array_t<double>(result.size(), result.data());
    })
    .def("compute_structuretensor_features", [](mpcl::pointcloud &self){
      auto result = self.compute_structuretensorfeatures(); // this needs to be refined to fit your query
      
      return py::array_t<double>(result.size(), result.data());
    
    }
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

    */
    /*#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
    #endif*/
}
