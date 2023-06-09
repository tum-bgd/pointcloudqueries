import numpy as np
import pointcloudqueries
import time
import sys
class cfg:
    N=5000000
    
    

start = time.time()
pcl = np.random.rand(cfg.N,3)
pcl4 = np.random.rand(cfg.N,4)
print(pcl.shape)
print(pcl[:10,:])
print("Random Generation: %s" %(str(time.time()-start)))


x = pointcloudqueries.pointcloud3d()
x.add(pcl);
print("4D")
y = pointcloudqueries.pointcloud4d()
y.add(pcl4)
y.index()
start = time.time()
y.boxfilter_4d("box",0.1, 0.4);
print("Boxfilter: %s" %(str(time.time()-start)))

print("Attributes: " ,y.attributes());
#import scipy.stats
#radii = y.get_attrib("box_boxfilter4d")
#from matplotlib import pyplot as plt
#plt.scatter(pcl4[:,3], radii)
#plt.savefig("/var/www/html/plot.png")


print(scipy.stats.describe(radii))
sys.exit(0)

start = time.time()
x.add(pcl) # note this needs to be packed, sometimes you might to use numpy copy when you have sliced from some unaligned data
print("Adding: %s" %(str(time.time()-start)))

start = time.time()
x.index();
print("Indexing (1time): %s" %(str(time.time()-start)))

start = time.time()
distancemap = x.knn_radius("knn7", 7);
print("kNN Radius: %s" %(str(time.time()-start)))

radii = x.get_attrib("knn7")
print(radii)

# Eigenfeatures
start = time.time()
distancemap = x.eigenfeatures_knn("test", 7);
print("Eigenfeatures: %s" %(str(time.time()-start)))
print(x.get_attrib("test_linearity"))
