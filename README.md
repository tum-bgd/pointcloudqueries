# Point Cloud Queries

A Python module for selected point cloud queries

This code is for educational purposes

License is CC-BY-SA unless or except those parts that have a different license.
Such licenses come from

- Boost and its documentation
- Eigen and its documentation (though Eigen is currently commented out)
- HDF5 and its documentation

It is a NFDI4Earth Educational Resource, more information on https://www.bgd.ed.tum.de/en/projects/nfdi4earth/

# Installation

## The plan

When we have fixed the usability and add some documentation, we will build binary distributions for Linux,  Windows and Python >= 3.7 and push it to pypi,
such that you just need to

```
> pip install pointcloudqueries # This does not yet work
```

## At the moment

### On Windows

- Setup a C++ compilation environment
  - Install Visual Studio, and use the VS installer to install *Desktop development with C++* module.
  - Install [CMake](https://cmake.org/download/)
- Install Python
  - preferably from python.org, for example, `https://www.python.org/ftp/python/3.11.3/python-3.11.3-amd64.exe` and let it upgrade the py launcher.
  - Try that in a command line `py` runs python3
- Configure Eigen3 using CMake
  - First, go to a directory as your preference, then
    ```
    > git clone https://gitlab.com/libeigen/eigen.git
    > cd eigen && mkdir build && cd build && cmake ..
    ```
  - These steps will take care of environmental variables for Eigen compilation
- Install dependencies for Python
  ```
  > py -3.X -m pip install wheel tqdm h5py scipy
  ```

  - matplotlib and open3d also needed if visualization required
- Compile and install the `pointcloudqueries` library
  ```
  > py -3.X setup.py bdist_wheel
  > py -3.X -m pip install ./dist/*.whl
  ```
- Run benchmark
  ```
  > py -3.X ./benchmark/benchmark.py
  ```

### On Linux

We go for manylinux with

```
$ docker run -it --rm  -v $PWD:/io quay.io/pypa/manylinux2014_x
# cd io/
# /opt/python/cp37-cp37m/bin/python setup.py bdist_wheel
# auditwheel repair pointcloudqueries-0.0.1-cp38-cp38-linux_x86_64.whl
```

The wheelhouse contains a manylinux-wheel

```
> docker run -it --rm mwernerds/pointcloudqueries python3 benchmark/benchmark.py
```
