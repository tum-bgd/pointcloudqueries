# The core benchmark
import urllib.request
from tqdm import tqdm
import h5py
import pointcloudqueries
import time


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
    points = h5py.File("eth.h5")["eth"]["coords"][:]
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

