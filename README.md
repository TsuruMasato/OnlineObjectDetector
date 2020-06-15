Online Object Detector

input1 : RGB-D colored PointCloud for scene
input2 : non-colored .PCD (point cloud data) of target object

output : 6DoF estimation result in various ros style (tf flame, Pose, etc...).

How to build :
in your catkin workspace,

git clone https://gite.lirmm.fr/chappellet/online_object_detector
cd online_object_detector
catkin build (or catkin_make)


How to run :

1. roslaunch openni2_launch openni2.launch
2. rosparam set target_object_pcd_path **your object's PCD file path**
3. rosrun online_object_detector


and you can see the estimation result in Rviz, at camera flame.

I will make roslaunch file to make more convenient.