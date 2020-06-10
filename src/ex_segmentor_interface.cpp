#include "ex_segmentor.h"

namespace segmentor
{

bool ex_segmentor::set_object_from_PCD_file(std::string &target_object_pcd_path)
{
  pcl::PointCloud<pcl::PointXYZ> temp_object;
  if (pcl::io::loadPCDFile(target_object_pcd_path, temp_object) == 0)
  //if (pcl::io::loadPCDFile("/home/masato/src/catkin_ws/grabo.pcd", temp_object) == 0)
  {
    ROS_INFO("path is correct");
    pcl::copyPointCloud(temp_object, *object_);
    return true;
  }
  ROS_ERROR("Object PCD file is not found. Please check file path or directory");
  return false;
}

void ex_segmentor::save_result(const Eigen::Matrix4f &input, const double &icp_score)
{
  ResultInformationSet result;
  result.transform = input;
  result.generate(icp_score);
  results_vector_.push_back(result);
}

void ex_segmentor::update_best_result(const Eigen::Matrix4f &input, const double &icp_error)
{
  std::lock_guard<std::mutex> lock(best_result_mutex_);
  minimum_error_ = icp_error;
  best_result_.transform = input;
  best_result_.generate(icp_bottom_th_ - icp_error);
  ROS_WARN("best_result_score:%f", best_result_.icp_score);
}

void ex_segmentor::update_best_result(const Eigen::Matrix4f &input)
{
  std::lock_guard<std::mutex> lock(best_result_mutex_);
  best_result_.transform = input;
  best_result_.generate(best_result_.icp_score);
}

void ex_segmentor::reset_best_result()
{
  ROS_ERROR("reset the best result");
  std::lock_guard<std::mutex> lock(best_result_mutex_);
  best_result_cloud_->clear();
  minimum_error_ = 100.0f;
  //first_minimum_error_ = 1000.0f;
  best_result_.transform = Eigen::Matrix4f::Identity();
  best_result_.generate(-100.0f);
  best_result_updated_ = false;
}

//extrude the stacked results in this instance. and clear these stacks.
int ex_segmentor::get_results_vector(std::vector<ResultInformationSet> &output)
{
  output = results_vector_;
  int num_results = results_vector_.size();
  results_vector_.clear();
  return num_results;
}

geometry_msgs::TransformStamped ex_segmentor::convert_result_TF(const Eigen::Matrix4f &result, const std::string &flame_from, const std::string &flame_to)
{
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = flame_from;
  tf_msg.child_frame_id = flame_to;

  Eigen::Vector3f result_pos(result(0, 3), result(1, 3), result(2, 3));
  Eigen::Matrix3f result_rot = result.topLeftCorner(3, 3);
  Eigen::Quaternionf quat(result_rot);

  tf_msg.transform.translation.x = result_pos.x();
  tf_msg.transform.translation.y = result_pos.y();
  tf_msg.transform.translation.z = result_pos.z();
  tf_msg.transform.rotation.w = quat.w();
  tf_msg.transform.rotation.x = quat.x();
  tf_msg.transform.rotation.y = quat.y();
  tf_msg.transform.rotation.z = quat.z();

  return tf_msg;
}

geometry_msgs::TransformStamped ex_segmentor::convert_result_TF(const Eigen::Matrix4f &result)
{
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = camera_flame_id_;
  //tf_msg.header.frame_id = "/map";
  tf_msg.child_frame_id = "/target_object";

  Eigen::Vector3f result_pos(result(0, 3), result(1, 3), result(2, 3));
  Eigen::Matrix3f result_rot = result.topLeftCorner(3, 3);
  Eigen::Quaternionf quat(result_rot);

  tf_msg.transform.translation.x = result_pos.x();
  tf_msg.transform.translation.y = result_pos.y();
  tf_msg.transform.translation.z = result_pos.z();
  tf_msg.transform.rotation.w = quat.w();
  tf_msg.transform.rotation.x = quat.x();
  tf_msg.transform.rotation.y = quat.y();
  tf_msg.transform.rotation.z = quat.z();

  return tf_msg;
}

geometry_msgs::TransformStamped ex_segmentor::convert_result_TF(const ResultInformationSet &result)
{
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = camera_flame_id_;
  //tf_msg.header.frame_id = "/map";
  tf_msg.child_frame_id = "/target_object";

  tf_msg.transform.translation.x = result.pos.x();
  tf_msg.transform.translation.y = result.pos.y();
  tf_msg.transform.translation.z = result.pos.z();
  tf_msg.transform.rotation.w = result.quat.w();
  tf_msg.transform.rotation.x = result.quat.x();
  tf_msg.transform.rotation.y = result.quat.y();
  tf_msg.transform.rotation.z = result.quat.z();

  return tf_msg;
}

geometry_msgs::PoseStamped ex_segmentor::convert_result_Pose(const Eigen::Matrix4f &result)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = camera_flame_id_;

  Eigen::Vector3f result_pos(result(0, 3), result(1, 3), result(2, 3));
  Eigen::Matrix3f result_rot = result.topLeftCorner(3, 3);
  Eigen::Quaternionf quat(result_rot);
  pose_msg.pose.position.x = result_pos.x();
  pose_msg.pose.position.y = result_pos.y();
  pose_msg.pose.position.z = result_pos.z();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();

  return pose_msg;
}

geometry_msgs::PoseStamped ex_segmentor::convert_result_Pose(const ResultInformationSet &result)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = camera_flame_id_;

  pose_msg.pose.position.x = result.pos.x();
  pose_msg.pose.position.y = result.pos.y();
  pose_msg.pose.position.z = result.pos.z();
  pose_msg.pose.orientation.x = result.quat.x();
  pose_msg.pose.orientation.y = result.quat.y();
  pose_msg.pose.orientation.z = result.quat.z();
  pose_msg.pose.orientation.w = result.quat.w();

  return pose_msg;
}

sensor_msgs::PointCloud2 ex_segmentor::convert_result_PC2msg(const Eigen::Matrix4f &result, const std::string &flame, uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = flame;
  cloud_msg.header.stamp = ros::Time::now();

  pcl::PointCloud<PointXYZRGB> transformed_pc;
  pcl::transformPointCloud(*object_, transformed_pc, result);
  transformed_pc.is_dense = true;
  for (size_t i = 0; i < transformed_pc.size(); i++)
  {
    transformed_pc.points[i].r = color_r;
    transformed_pc.points[i].g = color_g;
    transformed_pc.points[i].b = color_b;
  }

  pcl::toROSMsg(transformed_pc, cloud_msg);
  return cloud_msg;
}

sensor_msgs::PointCloud2 ex_segmentor::convert_result_PC2msg(const Eigen::Matrix4f &result, uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = camera_flame_id_;
  cloud_msg.header.stamp = ros::Time::now();

  pcl::PointCloud<PointXYZRGB>::Ptr transformed_pc(new pcl::PointCloud<PointXYZRGB>);
  pcl::transformPointCloud(*object_, *transformed_pc, result);
  transformed_pc->is_dense = true;
  for (size_t i = 0; i < transformed_pc->size(); i++)
  {
    transformed_pc->points[i].r = color_r;
    transformed_pc->points[i].g = color_g;
    transformed_pc->points[i].b = color_b;
  }
  pcl::toROSMsg(*transformed_pc, cloud_msg);

  return cloud_msg;
}

mc_slam_project_msgs::objpos_viewpos ex_segmentor::convert_result_original_msg(const ResultInformationSet &result)
{
  mc_slam_project_msgs::objpos_viewpos output;

  output.header.frame_id = "map";
  output.header.stamp = ros::Time::now();
  output.obj_x = result.pos.x();
  output.obj_y = result.pos.y();
  output.obj_z = result.pos.z();
  output.roll = result.rpy.x();
  output.pitch = result.rpy.y();
  output.yaw = result.rpy.z();
  output.icp_result = result.icp_score;

  output.camera_x = latest_camera_pos_.pose.pose.position.x;
  output.camera_y = latest_camera_pos_.pose.pose.position.y;
  output.camera_z = latest_camera_pos_.pose.pose.position.z;

  return output;
}

void ex_segmentor::add_candidate_to_debug_cloud(const Eigen::Matrix4f &result, uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
  pcl::PointCloud<PointXYZRGB> transformed_object;
  pcl::transformPointCloud(*object_, transformed_object, result);
  for (size_t point = 0; point < transformed_object.size(); point++)
  {
    transformed_object.points[point].r = color_r;
    transformed_object.points[point].g = color_g;
    transformed_object.points[point].b = color_b;
  }
  *debug_cloud_ += transformed_object;
}

void ex_segmentor::add_pointcloud_to_debug_cloud(pcl::PointCloud<PointXYZRGB> target, uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
  for (size_t point = 0; point < target.size(); point++)
  {
    target.points[point].r = color_r;
    target.points[point].g = color_g;
    target.points[point].b = color_b;
  }
  *debug_cloud_ += target;
}

void ex_segmentor::update_camera_pos(const nav_msgs::OdometryConstPtr &input)
{
  LOG_INFO("camera callback")
  latest_camera_pos_ = *input;
  camera_pos_ready_ = true;
}

void ex_segmentor::calc_sight_center()
{
  double roll, pitch, yaw;
  tf::Quaternion quat_tmp;

  tf::quaternionMsgToTF(latest_camera_pos_.pose.pose.orientation, quat_tmp);
  tf::Matrix3x3(quat_tmp).getRPY(roll, pitch, yaw);

  centerX_ = latest_camera_pos_.pose.pose.position.x + gaze_length_ * cos(yaw);
  centerY_ = latest_camera_pos_.pose.pose.position.y + gaze_length_ * sin(yaw);
  centerZ_ = latest_camera_pos_.pose.pose.position.z - gaze_length_ * sin(pitch);
}

void ex_segmentor::update_tf_map_camera(geometry_msgs::TransformStamped &input)
{
  Eigen::Vector3f input_trans(input.transform.translation.x, input.transform.translation.y, input.transform.translation.z);
  Eigen::Quaternionf input_quat(input.transform.rotation.w, input.transform.rotation.x, input.transform.rotation.y, input.transform.rotation.z);
  Eigen::Matrix3f input_mat = input_quat.matrix();

  tf_map_camera_ << input_mat(0, 0), input_mat(0, 1), input_mat(0, 2), input_trans(0),
      input_mat(1, 0), input_mat(1, 1), input_mat(1, 2), input_trans(1),
      input_mat(2, 0), input_mat(2, 1), input_mat(2, 2), input_trans(2),
      0, 0, 0, 1;

  std::cout << "tf_map_camera_ : " << tf_map_camera_ << std::endl;
}

void ex_segmentor::transform_result_in_map_flame(Eigen::Matrix4f &input)
{
  input = tf_map_camera_ * input;
}

} // namespace segmentor