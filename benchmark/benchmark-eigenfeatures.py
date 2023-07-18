# The queryscalable benchmark

#
# Adding all point cloud,
# deleting the cloud
# putting in a certain size of query cloud (random of the global cloud)
# only runnable for smaller point clouds
# creating pairs of size/querytime values for a line chart like this.

import urllib.request
from tqdm import tqdm
import h5py
import pointcloudqueries
import time
import numpy as np
import sys
import os
import json
from matplotlib import pyplot as plt
from pathlib import Path

benchmark_dir = Path(__file__).parent.absolute()

class cfg:
    plot_pts = 5

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

def evaluate_dataset_knn(dataset, knn):
    dataset_path = benchmark_dir.joinpath(f"../datasets/{dataset}.h5").absolute()
    
    if not os.path.exists(dataset_path):
        print("Remote download")
        download_url("https://api.bgd.ed.tum.de/datasets/pointclouds/%s.h5"%(dataset), dataset_path)
    else:
        print("Loading locally")
    
    points = h5py.File(dataset_path)["coords"]
    print(points.shape)

    # Plan the plot slices
    N = points.shape[0]
    Ns = [int(1/(cfg.plot_pts-1) * i * N -0.5) for i in range(cfg.plot_pts)]
    slices = [slice(0,j) for j in Ns if 0 < j and j < N]
    
    x = pointcloudqueries.pointcloud3d()

    start = time.time()
    x.add(points) # note this needs to be packed, sometimes you might to use numpy copy when you have sliced from some unaligned data
    print("Adding: %.3fs" %(time.time()-start))

    start = time.time()
    x.index()
    print("Indexing (first time): %.3fs" %(time.time()-start))
    print("Calibrating to about %d points" %(knn))
    distancemap = x.knn_radius("knn", knn)
    radii = x.get_attrib("knn")
    print(radii)
    default_range = np.nanmean(x.get_attrib("knn"))
    print("Default range for %d points is %.3f" % (knn,default_range))
    
    plot=[]
    #for iteration in range(20):
    for s in tqdm(slices):
        x.drop_cloud(); # drop the cloud
        x.add (points[s,:]) # add points
        start = time.time()
        distancemap = x.eigenfeatures_range("test", default_range)
        plot = plot+[[s.stop,time.time()-start]]

    plot=np.array(plot)
#    print(plot)
 #   plt.plot(plot[:,0],plot[:,1])
#    plt.show()
    np.save(benchmark_dir.joinpath("result-%s-eigenrange-%04d.json" % (dataset,knn)),plot)
    return plot
    

if __name__=="__main__":
    plt.rcParams.update({'font.size': 14})
    plot_data = [(knn,evaluate_dataset_knn(sys.argv[1],knn)) for knn in [5,25,50,75]]
    for knn, data in plot_data:
        plt.plot(data[:,0],data[:,1],label="knn=%d" % (knn))
    plt.legend()
    plt.xlabel("query point cloud size")
    plt.ylabel("time [s]")
    plt.savefig(benchmark_dir.joinpath("%s_eigen30.png" %(sys.argv[1])))
