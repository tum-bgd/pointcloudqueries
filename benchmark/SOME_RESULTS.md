Running the following computation:
hannover.h5: 5.65GB [00:15, 376MB/s]                                          
(1480418498, 3)
Adding: 144.2806897163391
Indexing (1time): 420.03332710266113
kNN Radius: 349.203195810318
[0.38091313 0.40031019 0.3915664  ... 0.11279412 0.13002064 0.0712204 ]
Default range for 30 points is 0.03
1480290000/1480418498(0.999906)
Eigenfeatures: 1224.8112082481384

In about 20 minutes, we were able to compute eigenfeatures for the Hanover point cloud. That is

For our largest point cloud, we reach the safetly limit of our server. I guess there is (a) a memory leak
and (b) the amplification is too large (cloud is not deleted, numpy still holds the whole cloud).

But at least some queries run reasonably:
Loading locally
(6038738935, 3)
Adding: 285.383793592453
Indexing (1time): 2507.761243581772
kNN Radius: 2926.569240808487
[236.39697701 799.16440759 105.52003921 ... 209.50832913 227.19701835
 195.91522178]
Default range for 30 points is 288.52
121/6Killed935(1.98717e-08)
