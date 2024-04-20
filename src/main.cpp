#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "mpcl2.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;



/*
     auto r = pcl.unchecked<2>();
      for (py::ssize_t i = 0; i < r.shape(0); i++)
        
	{
	  if (r.shape(1) != 3) throw(std::runtime_error("Expect Nx3 matrix"));
	  self.cloud.push_back(typename pointcloud_type::point_type(r(i, 0),r(i,  1),r(i,  2)));
	}

*/

template<typename classtype, typename pointcloud_type>
void add_generic_api_functions(classtype cls)
{
    cls.def("size",[](const pointcloud_type &self) {
      return self.cloud.size();
    })
    .def("index",[](pointcloud_type &self){
      self.buildIndex();
    })
      .def("drop_cloud",[](pointcloud_type &self){
	self.cloud.clear();
      })
    .def("knn_radius",[](pointcloud_type &self, std::string name, int k){
        auto ret = self.computeKNNradius(name,k);
    })
    .def ("get_attrib",[](pointcloud_type &self, std::string name){
       auto result = self.get_attrib_by_name(name);
       return py::array_t<double>(result.size(), result.data()); 

    })
    .def("eigenfeatures_knn",[](pointcloud_type &self, std::string prefix, int k){
	self.map_kNN_features(k, prefix);
    })
    .def("eigenfeatures_range",[](pointcloud_type &self, std::string prefix, double r){
	self.map_3drange_features(r, prefix);
    })
    .def("attributes",[](const pointcloud_type &self){
        std::string attributes;
	for (const auto & s: self.named_attributes)
	   attributes +"=" + s.first;
	return attributes;
    })


    ; // keep on line to make new methods easier to add.
 

}


template<typename pointcloud_type, typename module_type>
void register_class_with_name3(module_type m, std::string name)
{
    auto cls = pybind11::class_<pointcloud_type>(m, name.c_str())
    .def(pybind11::init())
    .def("add", [](pointcloud_type &self, py::array_t<double, py::array::c_style | py::array::forcecast> pcl)
    {
     auto r = pcl.unchecked<2>();
      for (py::ssize_t i = 0; i < r.shape(0); i++)
        
	{
	  if (r.shape(1) != 3) throw(std::runtime_error("Expect Nx3 matrix"));
	  self.cloud.push_back(typename pointcloud_type::point_type(r(i, 0),r(i,  1),r(i,  2)));
	}
    });
    add_generic_api_functions<decltype(cls),pointcloud_type>(cls);


};

template<typename pointcloud_type, typename module_type>
void register_class_with_name4(module_type m, std::string name)
{
    auto cls = pybind11::class_<pointcloud_type>(m, name.c_str());
    // special API
    cls
    .def(pybind11::init())
    .def("add", [](pointcloud_type &self, py::array_t<double, py::array::c_style | py::array::forcecast> pcl)
    {
     auto r = pcl.unchecked<2>();
      for (py::ssize_t i = 0; i < r.shape(0); i++)
        
	{
	  if (r.shape(1) != 4) throw(std::runtime_error("Expect Nx4 matrix"));
	  typename pointcloud_type::point_type p(r(i, 0),r(i,  1),r(i,  2));
	  bg::set<3>(p,r(i,3));
	  self.cloud.push_back(p);
	}
    })
    .def("boxfilter_4d",[](pointcloud_type &self, std::string prefix, double radius, double time_window){
       boxfilter_4d(self, prefix, radius, time_window);
       
       
    })
    
    ;
    add_generic_api_functions<decltype(cls),pointcloud_type>(cls);

};

#include "zcurve.hpp"

template<class T>
py::array_t<T> create_matrix(size_t width, size_t height, T* data_ptr  = nullptr)
{
 return py::array_t<T>(
        	py::buffer_info(
	               data_ptr,
		       sizeof(T), //itemsize
		       py::format_descriptor<T>::format(),
		       2, // ndim
		       std::vector<size_t> { width, height }, // shape
		       std::vector<size_t> {height * sizeof(T), sizeof(T)} // strides
           )
    );
}


auto mix3 = [](py::array_t<uint32_t,py::array::c_style > points) {
	
	auto r = points.unchecked<2>();
        auto a = create_matrix<uint64_t>(r.shape(0),1);
        auto view = a.mutable_unchecked<2>();
	for (py::ssize_t i = 0; i < r.shape(0); i++)
	{
	  if (r.shape(1) != 3) throw(std::runtime_error("Expect Nx3 matrix"));
	  const int BIT_PER_DIMENSION=16;
	  //std::cout << "Accessing "  << i << std::endl;
	  view(i,0) = static_cast<uint64_t>(mix(
	    std::bitset<BIT_PER_DIMENSION> (r(i,0)),
	    std::bitset<BIT_PER_DIMENSION> (r(i,1)),
	    std::bitset<BIT_PER_DIMENSION> (r(i,2))
	    ));
	}
       return a;
    };

auto unmix3 = [](py::array_t<uint64_t,py::array::c_style > indices) {
  // given z curve indices, unmix to the integer cell
	auto r = indices.unchecked<2>();
        auto a = create_matrix<uint32_t>(r.shape(0),3);
        auto view = a.mutable_unchecked<2>();
	for (py::ssize_t i = 0; i < r.shape(0); i++)
	{
	  if (r.shape(1) != 1) throw(std::runtime_error("Expect Nx1 matrix"));
	  const int BIT_PER_DIMENSION=16;
	  std::bitset<BIT_PER_DIMENSION> x,y,z;
	  demix_from(std::bitset<64> (r(i,0)),x,y,z);
	  //      return std::make_pair(a.to_ulong(),b.to_ulong());

	  //std::cout << "Accessing "  << i << std::endl;
	  view(i,0) = static_cast<uint32_t>(x.to_ulong());
	  view(i,1) = static_cast<uint32_t>(y.to_ulong());
	  view(i,2) = static_cast<uint32_t>(z.to_ulong());
	}
       return a;
    };

PYBIND11_MODULE(pointcloudqueries, m) {
    register_class_with_name3<pointcloud<double,3>>(m,"pointcloud3d");
    register_class_with_name4<pointcloud<double,4>>(m,"pointcloud4d");
    register_class_with_name3<pointcloud<double,5>>(m,"pointcloud5d");

    register_class_with_name3<pointcloud<float,3>>(m,"pointcloud3f");
    register_class_with_name4<pointcloud<float,4>>(m,"pointcloud4f");
    register_class_with_name3<pointcloud<float,5>>(m,"pointcloud5f");

    m
      .def("mix3", mix3)
      .def("unmix3", unmix3)
      ;

}
