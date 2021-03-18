# Online Object Detector

videos are abailable :

https://www.youtube.com/watch?v=agjsPMJshy0

https://youtu.be/2rr84jnez4k

# Abstruct

input1 : RGB-D colored PointCloud for scene
input2 : non-colored .PCD (point cloud data) of target object

output : 6DoF estimation result in various ros style (tf flame, Pose, etc...).

# How to build 
in your catkin workspace,

`git clone https://github.com/TsuruMasato/OnlineObjectDetector`

`cd online_object_detector`

`catkin build (or catkin_make)`


# How to run 

1. roslaunch openni2_launch openni2.launch
2. rosparam set target_object_pcd_path **your_object's_PCD_file**
3. rosrun online_object_detector online_object_detector


If you don't have PCD file, you can use PCD generator.


1. rosrun online_object_detector pcd_generator **your_3D_model** **output_file_name**


and you can see the estimation result in Rviz, at camera flame.

I will make roslaunch file to make more convenient.

# Citation

if you use this module for your research, business, or some other things, please cite our paper.

"Online Object Searching by a Humanoid Robot in an Unkown Environment"
M.Tsuru et al. ICRA-RAL, 2021.
https://ieeexplore.ieee.org/document/9361266
