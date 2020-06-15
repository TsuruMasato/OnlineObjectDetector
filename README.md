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
2. rosparam set target_object_pcd_path **your_object's_PCD_file**
3. rosrun online_object_detector online_object_detector


If you don't have PCD file, you can use PCD generator.


1. rosrun online_object_detector pcd_generator **your_3D_model** **output_file_name**


and you can see the estimation result in Rviz, at camera flame.

I will make roslaunch file to make more convenient.