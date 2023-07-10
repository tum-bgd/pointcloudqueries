from plyfile import PlyData
from tqdm import tqdm;
import glob;
import numpy as np
import h5py

def append_dataset(ds, data):
    ds.resize(ds.shape[0]+ data.shape[0], axis=0)
    ds[-data.shape[0]:,:] = data


f = h5py.File("hannover.h5","w")
ds = f.create_dataset('coords', shape=(0,3), maxshape=(None,3), compression="gzip", chunks=True)

data = glob.glob("ply/*.ply")
allcoords = None
for i,x in tqdm(enumerate(data)):
    plydata = PlyData.read(x)
    coords = np.stack([plydata["vertex"]["x"],plydata["vertex"]["y"],plydata["vertex"]["z"].astype(float)], axis=-1);
    append_dataset(ds,coords)
      
