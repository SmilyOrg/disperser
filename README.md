## RayBatch

* position
* direction
* t


## Steps

### Initial
* generate ray positions and directions

### Iterate
* floor positions
* get morton codes


## Sparse Octree?

0 0 0 0  0 0 0 0
0 0 1 0  0 0 0 0
0 0 0 0  0 0 0 0
0 0 0 0  0 0 0 0

0 0 0 0  0 0 0 0
0 0 0 0  0 0 0 0
0 0 0 0  0 0 0 0
0 0 0 0  0 0 0 0

2 1
1 0
0 1



0 1  0 0
0 0  2 0

0 0  0 0
0 1  0 0

1 0  0 0
0 0  0 0