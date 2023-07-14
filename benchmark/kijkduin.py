import h5py
import pointcloudqueries
import numpy as np
from tqdm import tqdm
import sys
# The Kijkduin is a 4D point cloud with really heavy footprint.
# This is the classical density function evaluation and not more, but
# we already load time data for estimating the runtime

#
# next step is to allow 4D point clouds in library
# followed by running the spatiotemporal range query as specified.
#

import time
    
if __name__=="__main__":
    pcl = pointcloudqueries.pointcloud4d()
    f = h5py.File("/data/kijkduin-flat.h5")
    print([x for x in f])
    
    start = time.time()
    print("Reading")
    cloud = np.hstack([f["coords"][:100,:],f["time"][:100]])
    print(cloud.shape)
    print("reading: %s" %(str(time.time()-start)))
    start=time.time()
    pcl.add(cloud);
    print("adding: %s" %(str(time.time()-start))),
    start=time.time()
    pcl.index()
    print("indexing: %s" %(str(time.time()-start))),
    start=time.time()
    pcl.boxfilter_4d("filter",0.5, int(168*24*60*60/2));
    print("boxfilter: %s" %(str(time.time()-start))),
    start=time.time()
    print("attributes", pcl.attributes())    
        
    
    
