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


x = pointcloudqueries.pointcloud()


start = time.time()
x.add(pcl) # note this needs to be packed, sometimes you might to use numpy copy when you have sliced from some unaligned data
print("Adding: %s" %(str(time.time()-start)))

start = time.time()
x.index();
print("Indexing (1time): %s" %(str(time.time()-start)))

start = time.time()
distancemap = x.nndistancemap();
print("Distance Map: %s" %(str(time.time()-start)))


