# The core benchmark
import urllib.request
from tqdm import tqdm
import h5py
import pointcloudqueries
import time
import numpy as np
import sys
import os

class DownloadProgressBar(tqdm):
    def update_to(self, b=1, bsize=1, tsize=None):
        if tsize is not None:
            self.total = tsize
        self.update(b * bsize - self.n)

def download_url(url, output_path):
    with DownloadProgressBar(unit='B', unit_scale=True,
                             miniters=1, desc=url.split('/')[-1]) as t:
        urllib.request.urlretrieve(url, filename=output_path, reporthook=t.update_to)


if __name__=="__main__":
    download_url("https://api.bgd.ed.tum.de/datasets/pointclouds/eth.h5", "eth.h5")
    points = h5py.File("eth.h5")["eth"]["coords"][:100000]
    print(points.shape)

    x = pointcloudqueries.pointcloud3d()

    start = time.time()
    x.add(points) # note this needs to be packed, sometimes you might to use numpy copy when you have sliced from some unaligned data
    print("Adding: %s" %(str(time.time()-start)))

    start = time.time()
    x.index();
    print("Indexing (1time): %s" %(str(time.time()-start)))

    start = time.time()
    distancemap = x.knn_radius("knn7", 7);
    print("kNN Radius: %s" %(str(time.time()-start)))

    radii = x.get_attrib("knn7")
    print(radii)

    start = time.time()
    distancemap = x.eigenfeatures("test", 7);
    print("Eigenfeatures: %s" %(str(time.time()-start)))
    print(x.get_attrib("test_linearity"))

    if os.environ.get('PCL_BENCHMARK_NOVISUALIZATION') is None:
        import matplotlib.cm
        
        feature="linearity"
        
        cmap = matplotlib.cm.get_cmap('Spectral')
        feature_values = x.get_attrib("test_planarity")
        feature_values = feature_values / np.max(feature_values)
        colorizer = lambda x: cmap(x)
        colorizer_func = np.vectorize(colorizer)
        
        feature_values = colorizer_func(feature_values)
        feature_colors = np.stack([feature_values[0],feature_values[1],feature_values[2]],axis=-1)
        print(feature_colors.shape)
        
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(feature_colors)
        #pcd.normals = o3d.utility.Vector3dVector(normals)
        o3d.visualization.draw_geometries([pcd])
