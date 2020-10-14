# point-cloud-experiments
Just some prototyping for QGIS point clouds implementation


## A quick 2D rendering test

Built EPT from http://maps.zh.ch/download/hoehen/2014/lidar/26850_12580.laz (81 MB LAZ, ~15M points)

2D Rendering of ~366K points by starting at node 1-0-0-0 and traversing down to level 3 (in total 7 octree nodes):

| Data type | Total run time | Total EPT index size |
|-------|-------:|------:|
| Zstandard | 50 ms | 213 MB |
| Binary | 38 ms | 655 MB |
| Laszip | 510 ms | 82 MB |

(Out of the total run time, rendering itself took ~8ms, the rest was spent on loading data)
