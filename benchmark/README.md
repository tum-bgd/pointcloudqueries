# Run evaluation system

The Kijkduin evaluation is programmed to find the kijkduin point cloud in /data and
the current directory in /code. The following docker command run from this directory is fine for our infrastructure. Note that a local copy of kijkduin-flat.h5 changes the picture.

```
docker run -it -d --name pclbenchmark -v $PWD:/code -v /data/share/datasets/pointcloud/:/data mwernerds/pointcloudqueries
```