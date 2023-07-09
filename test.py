import numpy as np
import pointcloudqueries
import time
class cfg:
    N=500000
    
    

start = time.time()
pcl = np.random.rand(cfg.N,3)
print(pcl.shape)
print(pcl[:10,:])
print("Random Generation: %s" %(str(time.time()-start)))


x = pointcloudqueries.pointcloud3d()


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
