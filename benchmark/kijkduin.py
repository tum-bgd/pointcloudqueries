import h5py
import pointcloudqueries
import numpy as np
from tqdm import tqdm
# The Kijkduin is a 4D point cloud with really heavy footprint.
# This is the classical density function evaluation and not more, but
# we already load time data for estimating the runtime

#
# next step is to allow 4D point clouds in library
# followed by running the spatiotemporal range query as specified.
#

import time

class cfg:
    chunks=int(10e6)
    debug=True
    
if __name__=="__main__":
    x = pointcloudqueries.pointcloud3d()
    print(cfg.chunks)
    f = h5py.File("/data/kijkduin-flat.h5") #"#/data/share/pointclouds/kijkduin.h5")
    start = time.time()
    print("Reading")
    cloud = f["coords"][:]
    print("reading: %s" %(str(time.time()-start)))

    x.add(cloud);
    
#    for i, sliced in tqdm(enumerate(f["time"].iter_chunks())):
#        time_slice = f["time"][sliced]
#        points_slice = f["coords"][sliced[0],:]
#        
#        x.add(points_slice) # note this needs to be packed, sometimes you might to use numpy copy when you have sliced from some unaligned data    
#        if i > cfg.chunks:
#            break
#print("Adding: %s" %(str(time.time()-start)))
        
        
    
    
