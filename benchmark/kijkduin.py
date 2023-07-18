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
import json

if __name__=="__main__":
    report={
        "file": "kijkduin-flat.h5",
        "query": "boxfilter4d"
        }
    
    pcl = pointcloudqueries.pointcloud4d()
    f = h5py.File("/data/kijkduin-flat.h5")
    
    report["numpts"] = np.random.randint(0,f["coords"].shape[0])
    select = slice(0, report["numpts"],1)
    print("Selection: %s " % (str(select)))
    start = time.time()
    print("Reading")
    cloud = np.hstack([f["coords"][select,:],f["time"][select]])
    report["time_read"] = time.time()-start
    print("PARTIAL:", [x + str(report[x]) for  x in report])
    start=time.time()
    pcl.add(cloud)
    report["time_add"] = time.time()-start
    print("PARTIAL:", [x + str(report[x]) for  x in report])
    print("adding: %s" %(time.time()-start)),
    start=time.time()
    pcl.index()
    report["time_index"] = time.time()-start
    print("PARTIAL:", [x + str(report[x]) for  x in report])
    start=time.time()
    pcl.boxfilter_4d("filter",0.5, int(168*24*60*60/2))
    report["time_boxfilter"] = time.time()-start
    print("PARTIAL:", [x + str(report[x]) for  x in report])
    start=time.time()
    result =  pcl.get_attrib("filter_boxfilter4d")
    report["time_extract"] = time.time()-start
    print("COMPLETE:", [x + str(report[x]) for  x in report])
    print("result.shape", result.shape)
    print("result.head():" , result[:10])
    with open("benchmark.json","a") as f:
        print(json.dumps(report),file=f)
        
    
    
