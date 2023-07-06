cmake_minimum_required(VERSION 3.4...3.18)
project(cmake_example)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -march=native -fopenmp")

set(CMAKE_VERBOSE_MAKEFILE ON)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(boost-cmake)
add_subdirectory(pybind11)

add_library(pointcloudqueries MODULE src/main.cpp)

target_link_libraries(pointcloudqueries PRIVATE pybind11::module pybind11::lto pybind11::windows_extras Boost::boost Eigen3::Eigen)

pybind11_extension(pointcloudqueries)
if(NOT MSVC AND NOT ${CMAKE_BUILD_TYPE} MATCHES Debug|RelWithDebInfo)
    # Strip unnecessary sections of the binary on Linux/macOS
    pybind11_strip(pointcloudqueries)
endif()

set_target_properties(pointcloudqueries PROPERTIES CXX_VISIBILITY_PRESET "hidden"
                                         CUDA_VISIBILITY_PRESET "hidden")





# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
#target_compile_definitions(pointcloudqueries
#                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
			   
