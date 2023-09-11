# Point Cloud Queries

A Python module for selected point cloud queries

This code is for educational purposes

## Installation

### Future Plans

When we have fixed the usability and add some documentation, we will build binary distributions for Linux,  Windows and Python >= 3.7 and push it to pypi, such that you just need to execute

```bash
pip install pointcloudqueries # This does not yet work
```
in oreder to install the python package. For now we need to build it ourselves.

### On Linux

easy.

### On Windows
- Install Visual Studio
- Download Boost (boost.org) 1.82 to `c:\boost`, and unpack it and make sure that `C:\boost\boost_1_82_0\boost_1_82_0\boost\geometry" is the directory containing the Boost Geometry Library
- If you end up with Boost somewhere else, update include_dirs in setup.py
- Install Python (preferably from python.org, for example, `https://www.python.org/ftp/python/3.11.3/python-3.11.3-amd64.exe` and let it upgrade the py launcher.
- Try that in a command line `py` runs python3
- py -m pip install numpy # numpy is a dependency for our package
- py setup.py develop
- py test.py

## Docker Build

We go for manylinux with

```bash
$ docker run -it --rm  -v $PWD:/io quay.io/pypa/manylinux2014_x
# cd io/
# /opt/python/cp37-cp37m/bin/python setup.py bdist_wheel
# auditwheel repair pointcloudqueries-0.0.1-cp38-cp38-linux_x86_64.whl
```
The wheelhouse contains a manylinux-wheel
docker run -it --rm mwernerds/pointcloudqueries python3 benchmark/benchmark.py

## Citation

Teuscher, B., Geißendörfer, O., Luo, X., Li, H., Anders, K., Holst, C., & Werner, M. (2023). Efficient In-Memory Point Cloud Query Processing. 18th International 3DGeoInfo Conference 2023. <https://www.bgd.ed.tum.de/pdf/2023_pointcloudqueries_Teuscher.pdf>

## Acknowledgement

This work has been funded by the German Research Foundation (NFDI4Earth, DFG project no. 460036893, https://www.nfdi4earth.de/).

## Contact

Balthasar Teuscher: balthasar.teuscher@tum.de

Technische Universität München, Professur für Big Geospatial Data Management, Lise-Meitner-Str. 9, 85521 Ottobrunn

## Licence

License is CC-BY-SA unless or except those parts that have a different license.
Such licenses come from

- Boost and its documentation
- Eigen and its documentation (though Eigen is currently commented out)
- HDF5 and its documentation

It is a NFDI4Earth Educational Resource, more information on https://www.bgd.ed.tum.de/en/projects/nfdi4earth/
