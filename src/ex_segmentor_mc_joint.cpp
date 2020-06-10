#include "ex_segmentor.h"

namespace segmentor
{

void ex_segmentor::config(ros::NodeHandle &nh_private)
{
  //LOG_INFO("start config")
  nh_->param("remove_plane_enable", remove_plane_enable_, true);
  nh_->param("region_limit_enable", region_limit_enable_, true);
  nh_->param("voxel_filter_enable", voxel_filter_enable_, true);
  nh_->param("icp_bottom_th_", icp_bottom_th_, 0.001050);

  nh_->param("camera_odom_topic", camera_odom_topic_, std::string("/rtabmap/odom"));
  nh_->param("target_object_pcd_path", target_object_pcd_path_, std::string("/home/masato/src/catkin_ws/grabo.pcd"));
  nh_->param("point_cloud_subscribe_topic", point_cloud_subscribe_topic_, std::string("/camera/depth_registered/points"));
  nh_->param("publish_best_result", publish_best_result_, true);
  nh_->param("publish_result_as_tf", publish_result_as_tf_, false);
  nh_->param("publish_result_as_pose", publish_result_as_pose_, true);
  nh_->param("publish_result_as_original_msg", publish_result_as_original_msg_, true);
  nh_->param("publish_result_as_PC2", publish_result_as_PC2_, false);
  nh_->param("debug_mode", debug_mode_, true);
  //LOG_INFO("point_cloud_subscribe_topic_ :" << point_cloud_subscribe_topic_);
}

void ex_segmentor::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  //LOG_INFO("point cloud callback")
  //std::lock_guard<std::mutex> lock(mutex_point_cloud_);
  pcl::fromROSMsg(*input, *scene_);
  //LOG_ERROR(camera_flame_)
  try
  {
    pcl::PointCloud<PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<PointXYZRGB>);
    latest_tf_map_camera_ = tfBuffer_.lookupTransform("map", input->header.frame_id, ros::Time(0));
    pcl_ros::transformPointCloud(*scene_, *transformed_cloud_ptr, latest_tf_map_camera_.transform);
    set_scene(transformed_cloud_ptr);
    point_cloud_ready_ = true;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    //continue;
  }
}

void ex_segmentor::publish_all_reults()
{

  if (publish_best_result_ && best_result_.icp_score > 0.0f)
  {
    tfBroadcaster_.sendTransform(convert_result_TF(best_result_));
    if (is_best_result_updated() /* && is_grobal_mode() */)
    {
      //std::cout << "now add score to the highest pos" << std::endl;
      pub_original_msg_.publish(convert_result_original_msg(best_result_));
    }
  }
  if (results_vector_.size())
  {
    for (size_t i = 0; i < results_vector_.size(); i++)
    {
      if (publish_result_as_tf_)
      {
        tfBroadcaster_.sendTransform(convert_result_TF(results_vector_.at(i)));
      }
      if (publish_result_as_pose_)
      {
        pub_pose_.publish(convert_result_Pose(results_vector_.at(i)));
      }
      if (publish_result_as_PC2_)
      {
        pub_pc2_.publish(convert_result_PC2msg(results_vector_.at(i).transform));
      }
      if (publish_result_as_original_msg_ /* && point_cloud_subscribe_topic_ == map_cloud_subscribe_topic_ */)
      {
        pub_original_msg_.publish(convert_result_original_msg(results_vector_.at(i)));
      }
    }
  }
  results_vector_.clear();
  if (debug_mode_)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*debug_cloud_, cloud_msg);
    cloud_msg.header.frame_id = get_camera_flame_id(); //now fixed as "map"
    pub_debug_.publish(cloud_msg);
  }

}


}