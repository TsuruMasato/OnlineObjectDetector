#pragma once

/*
A class for colored point cloud recognition.
Input : RGB-D point cloud + target .pcd (point cloud data)
Output : 6DoF (x,y,z,r,p,y) pose and tf flame 
*/

#include <vector>
#include <thread>
#include <random>
#include <atomic>
#include <mutex>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/kdtree.h>

// PCL filters
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// PCL sampling-base
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

// PCL segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>

// PCL 3D feature descriptor
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

// PCL registration
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/registration/gicp.h>

// ROS
#include <mc_rtc/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS_PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <mc_rtc/logging.h>
#include <mc_slam_project_msgs/objpos_viewpos.h>
#include "result_information_set.h"


#define COLOR_DEBUG
#define FPFH_MATCHING

namespace segmentor
{

class ex_segmentor
{

  // Types Definitions
  typedef pcl::PointXYZRGB PointXYZRGB;
  typedef pcl::PointNormal PointNormal;

  typedef pcl::FPFHSignature33 FPFH;
  typedef pcl::PointCloud<FPFH> FPFHCloud;

public:
  ex_segmentor(ros::NodeHandle &nh);
  template <typename PointCloudPtr>
  ex_segmentor(PointCloudPtr target_objet_ptr);
  ~ex_segmentor(){};

  void config();
  void config(ros::NodeHandle &nh_private);
  bool input_clouds_check();
  bool init();
  void set_result_flame_id(std::string &flame) { result_flame_id_ = flame; };
  void set_scene(pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud)
  {
    scene_->clear();
    debug_cloud_->clear();
    pcl::copyPointCloud(*input_cloud, *scene_);
  };
  bool set_object_from_PCD_file(std::string &target_object_pcd_path);
  template <typename PointCloudPtr>
  void set_object(PointCloudPtr &target_cloud) { pcl::copyPointCloud(*target_cloud, *object_); };

  //main process sequence. execute all prcesses at once.
  bool run();

  int get_results_vector(std::vector<ResultInformationSet> &output);

  void reset_best_result();

  void update_tf_map_camera(geometry_msgs::TransformStamped &input);

  void update_camera_pos(const nav_msgs::OdometryConstPtr &input);

  bool is_best_result_updated() { return best_result_updated_; };

  geometry_msgs::TransformStamped convert_result_TF(const Eigen::Matrix4f &result, const std::string &flame_from, const std::string &flame_to);
  geometry_msgs::TransformStamped convert_result_TF(const Eigen::Matrix4f &result);
  geometry_msgs::TransformStamped convert_result_TF(const ResultInformationSet &result);
  geometry_msgs::TransformStamped convert_result_TF(const ResultInformationSet &result, std::string flame_name);
  geometry_msgs::PoseStamped convert_result_Pose(const Eigen::Matrix4f &result);
  geometry_msgs::PoseStamped convert_result_Pose(const ResultInformationSet &result);
  sensor_msgs::PointCloud2 convert_result_PC2msg(const Eigen::Matrix4f &result, const std::string &flame, uint8_t color_r = 255, uint8_t color_g = 0, uint8_t color_b = 0);
  sensor_msgs::PointCloud2 convert_result_PC2msg(const Eigen::Matrix4f &result, uint8_t color_r = 255, uint8_t color_g = 0, uint8_t color_b = 0);
  mc_slam_project_msgs::objpos_viewpos convert_result_original_msg(const ResultInformationSet &result);
  void add_candidate_to_debug_cloud(const Eigen::Matrix4f &result, uint8_t color_r = 255, uint8_t color_g = 0, uint8_t color_b = 0);
  void add_pointcloud_to_debug_cloud(pcl::PointCloud<PointXYZRGB> target, uint8_t color_r = 255, uint8_t color_g = 0, uint8_t color_b = 0);

  std::string get_result_flame_id() { return result_flame_id_; };

  void switch_to_local_mode()
  {
    region_limit_enable_ = true; //warning!!
    grobal_mode_ = false;
  };

  void switch_to_grobal_mode()
  {
    region_limit_enable_ = true;
    grobal_mode_ = true;
  };

  void switch_mode()
  {
    if (grobal_mode_)
    {
      switch_to_local_mode();
    }
    else
    {
      switch_to_grobal_mode();
    }
    reset_best_result();
  }

  bool is_grobal_mode()
  {
    return grobal_mode_;
  };

  ResultInformationSet best_result_;
  pcl::PointCloud<PointXYZRGB>::Ptr debug_cloud_{new pcl::PointCloud<PointXYZRGB>};

protected:
  /* inner PCL process functions */
  void pass_through_filter(const pcl::PointCloud<PointXYZRGB>::Ptr &cloud);

  void statical_outlier_filter(const pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, int nr_k = 12, double stddev_mult = 0.1);

  void voxel_filter(const pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, float resolution = 0.007f);

  bool remove_plane(const pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, const Eigen::Vector3f &axis = Eigen::Vector3f(0.0, 0.0, 1.0), double plane_thickness = 0.008);

  void clustoring_with_color(pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> &clusters, int min_cluster_size = 100, float distance_th = 0.03f, float color_th = 12.0f, float region_color_th = 75, unsigned int num_nbr = 40);

  bool object_size_check(pcl::PointCloud<PointXYZRGB>::Ptr &input_obj, pcl::PointCloud<PointXYZRGB>::Ptr &input_scene, const float size_th = 0.4);

  template <typename PointCloudPtr>
  bool icp_registration(PointCloudPtr &input_obj, PointCloudPtr &input_scene, PointCloudPtr &output_obj, Eigen::Matrix4f &result_transform, float &result_error, uint max_iteration = 20, float max_distance = 0.05f, float ransac_th = 0.05f);

  void FPFH_generation(pcl::PointCloud<PointXYZRGB>::Ptr &input, FPFHCloud::Ptr &output);

  template <typename PointType, typename PointCloudPtr>
  bool FPFH_matching(PointCloudPtr &object, FPFHCloud::Ptr &object_feature, PointCloudPtr &scene, FPFHCloud::Ptr &scene_feature, PointCloudPtr &result_cloud, Eigen::Matrix4f &result_transformation);

  bool object_registration(pcl::PointCloud<PointXYZRGB>::Ptr &cluster);

  void PCA_registration(pcl::PointCloud<PointXYZRGB>::Ptr &input_obj, pcl::PointCloud<PointXYZRGB>::Ptr &input_scene, pcl::PointCloud<PointXYZRGB>::Ptr &projected_obj, Eigen::Matrix4f &result_transform);

  void calc_sight_center();

  void save_result(const Eigen::Matrix4f &input, const double &icp_score);
  void update_best_result(const Eigen::Matrix4f &input, const double &icp_score);
  void update_best_result(const Eigen::Matrix4f &input);
  bool is_best_result_inside();

  void transform_result_in_map_flame(Eigen::Matrix4f &input);

  void random_rotation();

  void three_steps_ICP_registration(Eigen::Matrix4f input_matrix);


  /* additional ros joint functions */

  void camera_odom_callback(const nav_msgs::Odometry &input);
  void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input_msg);
  void publish_all_reults();

  /* communication flags */
  bool initialized_;
  bool best_result_updated_;
  bool grobal_mode_ = true;

  /* ros */
  std::shared_ptr<ros::NodeHandle> nh_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  tf2_ros::TransformListener tfListener{tfBuffer_};
  //ros::Publisher pub_pose_best_;
  ros::Publisher pub_original_msg_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_pc2_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_best_cloud_;
  ros::Publisher pub_camera_cloud_;
  ros::Subscriber sub_camera_odom_;
  ros::Subscriber sub_point_cloud_;

  /* config params */

  bool region_limit_enable_;
  bool voxel_filter_enable_;
  bool remove_plane_enable_;
  
  bool publish_best_result_;
  bool publish_result_as_tf_;
  bool publish_result_as_pose_;
  bool publish_result_as_PC2_;
  bool publish_result_as_original_msg_;
  bool debug_mode_;
  bool goICP_mode_;
  double icp_bottom_th_;
  float FPFH_priority_score_;
  std::atomic<float> minimum_error_;
  std::atomic<float> extrime_icp_error_;
  //std::atomic<float> first_minimum_error_;
  std::string result_flame_id_ = std::string("map");
  std::string target_object_pcd_path_;
  std::string camera_odom_topic_;
  std::string point_cloud_subscribe_topic_;

  /*for region limit*/
  double centerX_ = 1.0;
  double centerY_ = 0.0;
  double centerZ_ = 0.0;
  double gaze_length_ = 1.3;

  bool point_cloud_ready_ = false;
  bool camera_pos_ready_ = false;
  bool has_been_outside_;

  /*result information*/
  std::vector<ResultInformationSet> results_vector_;
  pcl::PointCloud<PointXYZRGB>::Ptr best_result_cloud_{new pcl::PointCloud<PointXYZRGB>};

  /*input scene and recognition target*/
  pcl::PointCloud<PointXYZRGB>::Ptr object_{new pcl::PointCloud<PointXYZRGB>};
  FPFHCloud::Ptr object_feature_{new FPFHCloud};
  pcl::PointCloud<PointXYZRGB>::Ptr scene_{new pcl::PointCloud<PointXYZRGB>};

  /*others*/
  std::vector<pcl::PointIndices> clusters_indices_;
  Eigen::Matrix4f tf_map_camera_;
  geometry_msgs::TransformStamped latest_tf_map_camera_;
  nav_msgs::Odometry latest_camera_pos_;
  std::mutex best_result_mutex_;
  std::mutex cloud_update_mutex_;
  std::mutex mutex_best_cloud_;
  std::mutex mutex_scene_cloud_;
  std::mutex mutex_debug_cloud_;
  //std::mutex mutex_thread_icp_score;
};

} // namespace segmentor