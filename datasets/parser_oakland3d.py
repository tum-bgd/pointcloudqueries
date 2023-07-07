import numpy as np
import h5py;
import os

# This reader takes the oakland_3d dataset and brings it into our required format
# x y z label confidence
files = ["oakland_part2_ac.xyz_label_conf",
"oakland_part2_ad.xyz_label_conf",
"oakland_part2_ae.xyz_label_conf",
"oakland_part2_ag.xyz_label_conf",
"oakland_part2_ah.xyz_label_conf",
"oakland_part2_ai.xyz_label_conf",
"oakland_part2_aj.xyz_label_conf",
"oakland_part2_ak.xyz_label_conf",
"oakland_part2_al.xyz_label_conf",
"oakland_part2_ao.xyz_label_conf",
"oakland_part3_aj.xyz_label_conf",
"oakland_part3_ak.xyz_label_conf",
"oakland_part3_al.xyz_label_conf",
"oakland_part3_am.xyz_label_conf",
"oakland_part3_an.xyz_label_conf",
"oakland_part3_ao.xyz_label_conf",
"oakland_part3_ap.xyz_label_conf"]

data = [np.loadtxt(fname) for fname in files]
data = np.vstack(data)
print (data.shape)
with h5py.File("oakland.h5", "w") as f:
    f.create_dataset("points",data=data[:,:3])
    f.create_dataset("labels",data=data[:,3])
    f.create_dataset("confidence",data=data[:,4])
    
    
                     
