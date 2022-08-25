# hole_detection

### Description
* Filters point cloud data and finds holes in the surface of the object. Written
in C++ and CMake.

### Dependencies:
* PCL Library
  * https://github.com/PointCloudLibrary/pcl
  * brew install pcl

### Run
* navigate to the build folder (if it isn't there, simply make one) and run the following commands:
* cmake ..
* make
* ./hole_detection [filename].pcd

### Result
* Boundary points will be appended to boundary.pcd which will need to have its
  POINTS and WIDTH values set to the number of points it contains
* Any .pcd files can be viewed using pcl_viewer which should be available after
  downloading pcl

Written by Aiden Cullo(cullo7)
