#include <ros/ros.h>
#include "ex_segmentor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "online_object_detector");
  ros::NodeHandle nh;

  // Get params
  double frequency;
  nh.param("subscribe_frequency", frequency, 1.0);
  segmentor::ex_segmentor ex_segmentor(nh);

  // Main loop
  ros::Rate rate(frequency);
  ros::WallTime startTime = ros::WallTime::now();
  while (ros::ok())
  {
    ros::spinOnce();
    ex_segmentor.run();
    rate.sleep();
  }
  return 0;
}