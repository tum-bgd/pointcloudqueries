# The core benchmark
import urllib.request
from tqdm import tqdm
import h5py
import pointcloudqueries
import time
import numpy as np
import sys
import os
from pathlib import Path

benchmark_dir = Path(__file__).parent.absolute()

dataset="oakland_3d"
if len(sys.argv) > 1:
    dataset=sys.argv[1]


class DownloadProgressBar(tqdm):
    def update_to(self, b=1, bsize=1, tsize=None):
        if tsize is not None:
            self.total = tsize
        self.update(b * bsize - self.n)

def download_url(url, output_path):
    print("Downloading %s" % (url))
    with DownloadProgressBar(unit='B', unit_scale=True,
                             miniters=1, desc=url.split('/')[-1]) as t:
        urllib.request.urlretrieve(url, filename=output_path, reporthook=t.update_to)

# todo: update eth to flat
if __name__=="__main__":
    dataset_path = benchmark_dir.joinpath(f"../datasets/{dataset}.h5").absolute()
    
    if not os.path.exists(dataset_path):
        print("Remote download")
        download_url(f"https://api.bgd.ed.tum.de/datasets/pointclouds/{dataset}.h5", dataset_path)
    else:
        print("Loading locally")
    
    
    points = h5py.File(dataset_path)["coords"]
    print(points.shape)

    x = pointcloudqueries.pointcloud3d()

    start = time.time()
    x.add(points) # note this needs to be packed, sometimes you might to use numpy copy when you have sliced from some unaligned data
    print("Adding: %.3fs" %(time.time()-start))

    start = time.time()
    x.index()
    print("Indexing (first time): %.3fs" %(time.time()-start))

    start = time.time()
    distancemap = x.knn_radius("knn30", 30)
    print("kNN Radius: %.3fs" %(time.time()-start))

    radii = x.get_attrib("knn30")
    print(radii)
    default_range = np.nanmean(x.get_attrib("knn30"))
    print("Default range for 30 points is %.3f" % (default_range))

    start = time.time()
    distancemap = x.eigenfeatures_range("test", default_range);print("")
    print("Eigenfeatures: %.3fs" %(time.time()-start))
    
    if os.environ.get('PCL_BENCHMARK_NOVISUALIZATION') is None:
        import scipy.stats
        import matplotlib
        
        data = x.get_attrib("test_planarity")
        data[np.isnan(data)] = 0  # impute nans
        feature_values = data
        print(scipy.stats.describe(feature_values))
        
        cmap = matplotlib.colormaps.get_cmap('viridis')
        colorizer = lambda x: cmap(x)
        colorizer_func = np.vectorize(colorizer)
        
        feature_values = feature_values / np.max(feature_values)
        feature_values = colorizer_func(feature_values)
        feature_colors = np.stack([feature_values[0],feature_values[1],feature_values[2]],axis=-1)
        print(feature_colors.shape)
        
        import open3d as o3d
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(feature_colors)
        #pcd.normals = o3d.utility.Vector3dVector(normals)
        o3d.visualization.draw_geometries([pcd])
