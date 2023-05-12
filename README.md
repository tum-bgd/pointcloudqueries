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
pip install pointcloudqueries # This does not yet work
```

## At the moment
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


