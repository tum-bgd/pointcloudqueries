# Run evaluation system

The Kijkduin evaluation is programmed to find the kijkduin point cloud in /data and
the current directory in /code. The following docker command run from this directory is fine for our infrastructure. Note that a local copy of kijkduin-flat.h5 changes the picture.

```
docker run -it -d --name pclbenchmark -v $PWD:/code -v /data/share/datasets/pointcloud/:/data mwernerds/pointcloudqueries
```

# Plots

## Benchmark-Eigenfeatures and Benchmark-knn

On oakland
These plots hold total transformation times for indexing all data, but querying only subsets of points.
It shows that this 1,6 million point point cloud keeps below 2 seconds for all operations and interestingly that it is not compute bound. That is, the evaluation of geometric features comes practically for free.

On eth


