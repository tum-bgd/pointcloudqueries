import pointcloudqueries
import numpy as np
from itertools import product
def test1():
    
    x = np.random.uniform(size=99).reshape(-1,3)
    x *=10
    x = x.astype(np.uint32)
    print(x.shape)
    y = pointcloudqueries.mix3(x)#np.array([[1,2],[1,2]], dtype=np.uint32))
    print(y)
#print(x)


def test2d(n=12):
    g = np.array([[x,y,z] for x,y,z in  product(range(n),range(n), range(n))],dtype=np.uint32)
    print(g.shape)
    zorder = pointcloudqueries.mix3(g)

test2d(24)
