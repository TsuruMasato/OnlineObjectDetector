Online Object Detector

input1 : RGB-D colored PointCloud for scene
input2 : non-colored .PCD (point cloud data) of target object

output : 6DoF estimation result in various ros style (tf flame, Pose, etc...).


How to use :

1. roslaunch openni2_launch openni2.launch
2. rosrun online_object_detector

and you can see the estimation result in Rviz, at camera flame.