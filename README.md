# facetracking
Opencv face tracking in ROS (Haar wavelets + KF tracking using Opencv 3, ROS K), processes images streamed on ros image topic. See 

> Burke, M. G. (2015). Fast upper body pose estimation for human-robot interaction (doctoral thesis). https://doi.org/10.17863/CAM.203

for details.

* In a rosbuild workspace
```
rosmake facetracking
rosrun facetracking faceTracker
```
