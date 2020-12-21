#include "ex_segmentor.h"

namespace segmentor
{

ex_segmentor::ex_segmentor(ros::NodeHandle &nh)
    : nh_(std::make_shared<ros::NodeHandle>(nh))
{
  // private NodeHandle for parameters and private messages (debug / info)

  config(*nh_);
  sub_camera_odom_ = nh_->subscribe<nav_msgs::Odometry>(camera_odom_topic_, 1, &ex_segmentor::update_camera_pos, this);
  sub_point_cloud_ = nh_->subscribe<sensor_msgs::PointCloud2>(point_cloud_subscribe_topic_, 1, &ex_segmentor::point_cloud_callback, this);
  pub_pose_ = nh_->advertise<geometry_msgs::PoseStamped>("detection_result", 1);
  //pub_pose_best_ = nh_->advertise<geometry_msgs::PoseStamped>("best_result", 1);
  pub_pc2_ = nh_->advertise<sensor_msgs::PointCloud2>("icp_result", 1);
  pub_original_msg_ = nh_->advertise<mc_slam_project_msgs::objpos_viewpos>("result_msg", 1);
  pub_debug_ = nh_->advertise<sensor_msgs::PointCloud2>("inner_debug", 1);

  set_object_from_PCD_file(target_object_pcd_path_);
  init();
};

template <typename PointCloudPtr>
ex_segmentor::ex_segmentor(PointCloudPtr target_objet_ptr)
{
  pcl::copyPointCloud(*target_objet_ptr, *object_);
}

bool ex_segmentor::init()
{
  if (object_->size() == 0)
  {
    return false;
  }
  if (voxel_filter_enable_)
  {
    voxel_filter(object_);
  }
  FPFH_generation(object_, object_feature_);
  minimum_error_ = 100.0f;
  //first_minimum_error_ = 100.0f;
  reset_best_result();

  has_been_outside_ = false;
  point_cloud_ready_ = false;
  camera_pos_ready_ = false;
  initialized_ = true;

  FPFH_priority_score_ = 0.0f;

  return true;
}

void ex_segmentor::config()
{
  //nh.getParam("target_object_pcd", target_object_pcd);
  //nh.getParam("subscribe_cloud_topic", subscribe_cloud_topic);
  remove_plane_enable_ = true;
  region_limit_enable_ = true;
  voxel_filter_enable_ = true;
  icp_bottom_th_ = 0.001050; //0.000550

  // if (subscribe_cloud_topic == rtab_topic)
  // {
  //   grobal_mode = true;
  //   ROS_ERROR("grobal_mode");
  // }
  // else
  // {
  //   grobal_mode = false;
  //   ROS_ERROR("local_mode");
  // }
  return;
}

bool ex_segmentor::run()
{
  //LOG_INFO("start run()")
  if (!initialized_)
  {
    LOG_ERROR("Please initialize this instance by init()")
    return false;
  }
  
  if(!point_cloud_ready_ || !camera_pos_ready_)
  {
    if (!point_cloud_ready_)
      LOG_ERROR("No input cloud")
    if (!camera_pos_ready_)
      LOG_ERROR("No input camera")
    return false;
  }

  //config(*nh_);

  if (!input_clouds_check())
  {
    return false;
  }

  if (region_limit_enable_)
  {
    pass_through_filter(scene_);
    if (scene_->size() <= 10)
    {
      ROS_WARN("Because of region limit, all points are removed");
      return false;
    }
  }

  if (voxel_filter_enable_)
  {
    voxel_filter(scene_);
  }

  if(grobal_mode_)
    statical_outlier_filter(scene_);

  if (remove_plane_enable_)
  {
    remove_plane(scene_);
  }

  if (grobal_mode_)
    statical_outlier_filter(scene_);

  std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> clusters;
  clustoring_with_color(scene_, clusters);

  std::vector<std::thread> threads;
  results_vector_.clear();
  std::cout << "start multithreads registration" << std::endl;
  for (size_t i = 0; i < clusters.size(); i++)
  {
    threads.emplace_back(std::thread(&ex_segmentor::object_registration, this, std::ref(clusters[i])));
    //statical_outlier_filter(clusters[i], 12, 3.0);
    //add_pointcloud_to_debug_cloud(*clusters[i], uint(rand() % 256), uint(rand() % 256), uint(rand() % 256));
    //bool icp_success = object_registration(clusters[i]);
    // if (icp_success)
    // {
    //   ROS_INFO("cluster[%d] is estimated as an object", i);
    //   LOG_INFO("cluster" << i << "is estimated as an object")
    // }
  }
  for (auto &thread : threads)
  {
    thread.join();
  }
  //for (size_t i = 0; i < clusters.size(); i++)
  //  add_pointcloud_to_debug_cloud(*clusters[i], uint(rand() % 256), uint(rand() % 256), uint(rand() % 256));
  std::cout << "finish multithreads registration" << std::endl;

  //ROS_INFO("%d results are stored in results_vector. Please use get_results_vector", results_vector_.size());
  if (best_result_cloud_ != nullptr && best_result_cloud_->size() > 0 && best_result_updated_)
  {
    best_fpfh_updated_ = false;
    if (is_best_result_inside())
    {
      random_rotation();
      /*
      float first_icp_error, second_icp_error, extrime_icp_error;
      Eigen::Matrix4f first_icp_transform, second_icp_transform, extrime_icp_transform;
      pcl::PointCloud<PointXYZRGB>::Ptr temp_icp_cloud(new pcl::PointCloud<PointXYZRGB>);
      icp_registration(best_result_cloud_, scene_, temp_icp_cloud, first_icp_transform, first_icp_error, 20, 0.03f, 0.01f);
      icp_registration(temp_icp_cloud, scene_, temp_icp_cloud, second_icp_transform, second_icp_error, 20, 0.02f, 0.01f);
      best_result_updated_ = icp_registration(temp_icp_cloud, scene_, temp_icp_cloud, extrime_icp_transform, extrime_icp_error, 30, 0.017f, 0.01f); //50 0.02 0.03 //50, 0.01, 0.01 //50, 0.015, 0.03
      */

      /*
      if (extrime_icp_error < minimum_error_)
      {
        pcl::copyPointCloud(*temp_icp_cloud, *best_result_cloud_);
        update_best_result((extrime_icp_transform * second_icp_transform * first_icp_transform * best_result_.transform), extrime_icp_error);
      }
      */

      // if (extrime_icp_error_ > icp_bottom_th_ * 1.5)
      // {
      //   //detect that the object is not in sight.
      //   reset_best_result();
      // }
    }
  }
  else if (best_result_cloud_ != nullptr && best_result_cloud_->size() > 0)
  {
    add_pointcloud_to_debug_cloud(*best_result_cloud_);
  }
  publish_all_reults();
  return true;
}

bool ex_segmentor::input_clouds_check()
{
  if (object_->size() == 0)
  {
    ROS_ERROR("No target object. Please set target object by constructor or set_object()");
    return false;
  }
  if (scene_->size() <= 5) // || scene_->is_dense == false)
  {
    ROS_ERROR("No scene. Please set scene by set_scene()");
    return false;
  }
  return true;
}

void ex_segmentor::pass_through_filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
{
  std::cout << "start pass_through_filter" << std::endl;
  calc_sight_center();
  // build the condition
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_limit(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, centerX_ - 1.5)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, centerX_ + 1.5)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, centerY_ - 1.5)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, centerY_ + 1.5)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, centerZ_ - 1.5)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, centerZ_ + 1.5)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setCondition(range_limit);
  condrem.setInputCloud(input_cloud);

  // apply filter
  condrem.filter(*input_cloud);
}

bool ex_segmentor::is_best_result_inside()
{
  std::cout << "start is_best_result_inside" << std::endl;
  bool result = true;
  pcl::PointCloud<PointXYZRGB> surrounding_cloud;

  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_limit(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, best_result_.pos.x() - 0.1)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, best_result_.pos.x() + 0.1)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, best_result_.pos.y() - 0.1)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, best_result_.pos.y() + 0.1)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, best_result_.pos.z() - 0.1)));
  range_limit->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, best_result_.pos.z() + 0.1)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setCondition(range_limit);
  condrem.setInputCloud(scene_);

  // apply filter
  condrem.filter(surrounding_cloud);

  if(surrounding_cloud.size() < 30)
  {
    //camera is gazing at another place
    std::cout << "There are no points around the best result" << std::endl;
    result = false;
    has_been_outside_ = true;
  }
  else
  {
    //camera is watching the highest area
    if(has_been_outside_) //if camera was watching outside just before...
    {
      std::cout << "minimum_error_ :" << minimum_error_ << std::endl;
      minimum_error_ = minimum_error_ + 0.00001f;
      FPFH_priority_score_ = 0.00003f;
      std::cout << "after minimum_error_ :" << minimum_error_ << std::endl;
      has_been_outside_ = false;
    }
    else
    {
      //usual case (continuing watching the highest area)
      FPFH_priority_score_ = 0.0f;
    }    
  }

  return result;
}

void ex_segmentor::statical_outlier_filter(const pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, int nr_k, double stddev_mult)
{
  pcl::StatisticalOutlierRemoval<PointXYZRGB> sorfilter(true); // Initializing with true will allow us to extract the removed indices
  sorfilter.setInputCloud(input_cloud);
  sorfilter.setMeanK(nr_k);
  sorfilter.setStddevMulThresh(stddev_mult);
  sorfilter.filter(*input_cloud);
}

void ex_segmentor::voxel_filter(const pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, float resolution)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(input_cloud);
  voxel.setLeafSize(resolution, resolution, resolution);
  voxel.filter(*input_cloud);
}

bool ex_segmentor::remove_plane(const pcl::PointCloud<PointXYZRGB>::Ptr &input_cloud, const Eigen::Vector3f &axis, double plane_thickness)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(500);
  seg.setAxis(axis);
  seg.setEpsAngle(0.25);
  seg.setDistanceThreshold(plane_thickness); //0.025 0.018
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);
  //ROS_INFO("plane size : %d", inliers->indices.size());

  if (inliers->indices.size() < 500)
  {
    //ROS_INFO("plane size is not enough large to remove.");
    return false;
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*input_cloud);

  return true;
}

void ex_segmentor::clustoring_with_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud, std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> &clusters, int min_cluster_size, float distance_th, float color_th, float region_color_th, unsigned int num_nbr)
{
  std::vector<pcl::PointIndices> clusters_indices;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdtree->setInputCloud(input_cloud);
  // Color-based region growing clustering object.  色ベースの領域成長クラスタリングのオブジェクト
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
  clustering.setInputCloud(input_cloud);
  clustering.setSearchMethod(kdtree);
  // Here, the minimum cluster size affects also the postprocessing step:
  // clusters_indices smaller than this will be merged with their neighbors.
  // ここで最小クラスタのサイズは後処理に影響する：これより小さなクラスタは近傍のクラスタにマージ
  clustering.setMinClusterSize(min_cluster_size); //170
  // Set the distance threshold, to know which points will be considered neighbors.
  // 距離の閾値の設定。近傍としてみなせるポイントを決めるためのもの
  clustering.setDistanceThreshold(distance_th); //1
  // Color threshold for comparing the RGB color of two points.　2点のRGB色を比較するための色閾値 大きい数値ほど様々な色の点をゴチャまぜにする。
  clustering.setPointColorThreshold(color_th); //9 6.5 25.0f 18.0f
  // Region color threshold for the postprocessing step: clusters_indices with colors
  // within the threshold will be merged in one.
  // 後処理のための領域色閾値: この閾値以下の色を持つクラスタは一つにマージ
  clustering.setRegionColorThreshold(region_color_th); //2
  //領域結合の際にチェックする近傍の数。デフォルトは100だが、処理が重くなる。結果に影響しない範囲で適度に小さく設定。
  clustering.setNumberOfRegionNeighbours(num_nbr);

  //clustering.setSmoothModeFlag(true);
  //clustering.setSmoothnessThreshold(0.95);

  clustering.extract(clusters_indices);

  for (std::vector<pcl::PointIndices>::const_iterator i = clusters_indices.begin(); i != clusters_indices.end(); ++i)
  {
    // ...add all points to a new cloud for each cluster...  そのポイントをすべて新たなクラウドに分割...
    pcl::PointCloud<PointXYZRGB>::Ptr cluster(new pcl::PointCloud<PointXYZRGB>);

    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
    {
      cluster->points.push_back(input_cloud->points[*point]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster);
  }
}

bool ex_segmentor::object_size_check(pcl::PointCloud<PointXYZRGB>::Ptr &input_obj, pcl::PointCloud<PointXYZRGB>::Ptr &input_scene, const float size_th)
{
  using PointType = PointXYZRGB;
  pcl::PCA<PointType> pca;

  pcl::PointCloud<PointType> objectProj;
  pca.setInputCloud(input_obj);
  pca.project(*input_obj, objectProj);

  pcl::PointCloud<PointType> sceneProj;
  pca.setInputCloud(input_scene);
  pca.project(*input_scene, sceneProj);

  PointType object_proj_min;
  PointType object_proj_max;
  pcl::getMinMax3D(objectProj, object_proj_min, object_proj_max);

  PointType scene_proj_min;
  PointType scene_proj_max;
  pcl::getMinMax3D(sceneProj, scene_proj_min, scene_proj_max);

  return std::abs((object_proj_max.x - object_proj_min.x) - (scene_proj_max.x - scene_proj_min.x)) < size_th && std::abs((object_proj_max.y - object_proj_min.y) - (scene_proj_max.y - scene_proj_min.y)) < size_th && std::abs((object_proj_max.z - object_proj_min.z) - (scene_proj_max.z - scene_proj_min.z)) < size_th;
}

template <typename PointCloudPtr>
bool ex_segmentor::icp_registration(PointCloudPtr &input_obj, PointCloudPtr &input_scene, PointCloudPtr &output_obj, Eigen::Matrix4f &result_transform, float &result_error, uint max_iteration, float max_distance, float ransac_th)
{
  pcl::IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
  icp.setInputSource(input_obj);
  icp.setInputTarget(input_scene);
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(max_distance); //0.05
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(max_iteration); //50
  // Set the transformation epsilon (criterion 2)
  //icp.setTransformationEpsilon (1e-5);  //1e-8
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon (0.000001);  //1
  icp.setRANSACOutlierRejectionThreshold(ransac_th);
  icp.align(*output_obj);
  if (icp.hasConverged())
  {
    result_transform = icp.getFinalTransformation();
    result_error = icp.getFitnessScore();
    return true;
  }
  else
  {
    result_transform = Eigen::Matrix4f::Identity(4, 4);
    result_error = 1.0;
    return false;
  }
}

void ex_segmentor::FPFH_generation(pcl::PointCloud<PointXYZRGB>::Ptr &input, FPFHCloud::Ptr &output)
{
  //first, generate normals
  pcl::NormalEstimationOMP<PointNormal, PointNormal> nest;
  pcl::PointCloud<PointNormal>::Ptr temp(new pcl::PointCloud<PointNormal>);
  pcl::copyPointCloud(*input, *temp);
  nest.setRadiusSearch(0.01); //0.03
  nest.setInputCloud(temp);
  nest.compute(*temp);

  //then, generate FPFH cloud
  pcl::FPFHEstimationOMP<PointNormal, PointNormal, FPFH> fest;
  fest.setRadiusSearch(0.01); //0.025
  fest.setInputCloud(temp);
  fest.setInputNormals(temp);
  fest.compute(*output);
}

template <typename PointType, typename PointCloudPtr>
bool ex_segmentor::FPFH_matching(PointCloudPtr &object, FPFHCloud::Ptr &object_feature, PointCloudPtr &scene, FPFHCloud::Ptr &scene_feature, PointCloudPtr &result_cloud, Eigen::Matrix4f &result_transformation)
{
  pcl::SampleConsensusPrerejective<PointType, PointType, FPFH> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_feature);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_feature);
  align.setMaximumIterations(5000);          // Number of RANSAC iterations 5000
  align.setNumberOfSamples(7);               // Number of points to sample for generating/prerejecting a pose 3
  align.setCorrespondenceRandomness(10);     // Number of nearest features to use 12
  align.setSimilarityThreshold(0.5f);        // Polygonal edge length similarity threshold 0.7
  align.setMaxCorrespondenceDistance(0.01f); // Inlier threshold
  align.setInlierFraction(0.05f);             // Required inlier fraction for accepting a pose hypothesis 0.7
  align.align(*result_cloud);
  if (align.hasConverged())
  {
    result_transformation = align.getFinalTransformation();
    //pcl::console::print_info("Inliers: %i/%i , %i\n", align.getInliers().size(), scene->size(), object->size());
    //return (float(align.getInliers().size()) / float(object->size()));
    return true;
  }
  //return 0.0f;
  return false;
}

void ex_segmentor::PCA_registration(pcl::PointCloud<PointXYZRGB>::Ptr &input_obj, pcl::PointCloud<PointXYZRGB>::Ptr &input_scene, pcl::PointCloud<PointXYZRGB>::Ptr &projected_obj, Eigen::Matrix4f &result_transform)
{
  using PointType = PointXYZRGB;
  pcl::PCA<PointType> pca;

  pcl::PointCloud<PointType> objProj;
  pca.setInputCloud(input_obj);
  pca.project(*input_obj, objProj);
  Eigen::Matrix3f EigenSpaceObj = pca.getEigenVectors();
  //std::cout << pca.getMean() << std::endl;
  Eigen::Vector3f PcaTransObj(pca.getMean()(0), pca.getMean()(1), pca.getMean()(2));
  Eigen::Matrix4f transform_obj;
  transform_obj << EigenSpaceObj(0, 0), EigenSpaceObj(0, 1), EigenSpaceObj(0, 2), PcaTransObj(0),
      EigenSpaceObj(1, 0), EigenSpaceObj(1, 1), EigenSpaceObj(1, 2), PcaTransObj(1),
      EigenSpaceObj(2, 0), EigenSpaceObj(2, 1), EigenSpaceObj(2, 2), PcaTransObj(2),
      0, 0, 0, 1;

  std::cout << transform_obj << std::endl;

  Eigen::Matrix3f EigenSpaceObjT = EigenSpaceObj.transpose();
  Eigen::Vector3f PcaTransObj_inv = -EigenSpaceObjT * PcaTransObj;
  Eigen::Matrix4f transform_obj_inv;
  transform_obj_inv << EigenSpaceObjT(0, 0), EigenSpaceObjT(0, 1), EigenSpaceObjT(0, 2), PcaTransObj_inv(0),
      EigenSpaceObjT(1, 0), EigenSpaceObjT(1, 1), EigenSpaceObjT(1, 2), PcaTransObj_inv(1),
      EigenSpaceObjT(2, 0), EigenSpaceObjT(2, 1), EigenSpaceObjT(2, 2), PcaTransObj_inv(2),
      0, 0, 0, 1;

  std::cout << transform_obj_inv << std::endl;

  pcl::PointCloud<PointType> sceneProj;
  pca.setInputCloud(input_scene);
  pca.project(*input_scene, sceneProj);
  Eigen::Matrix3f EigenSpaceScene = pca.getEigenVectors();
  Eigen::Vector4f PcaTransScene = pca.getMean();
  Eigen::Matrix4f transform_scene;
  //std::cout << pca.getMean() << std::endl;

  transform_scene << EigenSpaceScene(0, 0), EigenSpaceScene(0, 1), EigenSpaceScene(0, 2), PcaTransScene(0),
      EigenSpaceScene(1, 0), EigenSpaceScene(1, 1), EigenSpaceScene(1, 2), PcaTransScene(1),
      EigenSpaceScene(2, 0), EigenSpaceScene(2, 1), EigenSpaceScene(2, 2), PcaTransScene(2),
      0, 0, 0, 1;

  std::cout << transform_scene << std::endl;

  result_transform = transform_scene * transform_obj_inv;
  pcl::transformPointCloud(*input_obj, *projected_obj, result_transform);

  return;
};

bool ex_segmentor::object_registration(pcl::PointCloud<PointXYZRGB>::Ptr &cluster)
{
  std::cout << "start object_registration" << std::endl;
  statical_outlier_filter(cluster, 12, 3.0);
  //ROS_INFO("start object registration");
  float icp_error;
  Eigen::Matrix4f align_result_transform;
  align_result_transform.setIdentity();
  Eigen::Matrix4f icp_result_transform, total_rotation;
  pcl::PointCloud<PointXYZRGB>::Ptr moved(new pcl::PointCloud<PointXYZRGB>);

  //statical_outlier_filter(cluster, 12, 3.0);

  if (!object_size_check(object_, cluster))
  {
    add_pointcloud_to_debug_cloud(*cluster, 255, 255, 255);
    //ROS_INFO("cluseter size mismatch");
    return false;
  }
  //ROS_WARN("cluster size match");
  add_pointcloud_to_debug_cloud(*cluster, uint(random() % 256), uint(random() % 256), uint(random() % 256));

  pcl::PointCloud<PointXYZRGB>::Ptr object_aligned(new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr Final(new pcl::PointCloud<PointXYZRGB>);
#ifdef FPFH_MATCHING
  //using object feature, estimate object pos&pose
  //calc Normals and calc FPFH features
  FPFHCloud::Ptr cluster_feature(new FPFHCloud);
  FPFH_generation(cluster, cluster_feature);
  ROS_INFO("cluster_size : %d, feature size : %d", cluster->size(), cluster_feature->size());
  bool FPFH_match_success = FPFH_matching<PointXYZRGB>(object_, object_feature_, cluster, cluster_feature, object_aligned, align_result_transform);
  if (FPFH_match_success)
  {
    //add_pointcloud_to_debug_cloud(*object_aligned, 0, 0, 0);
    //ROS_WARN("Features matched. now try ICP...");

#endif
#ifndef FPFH_MATCHING
    PCA_registration(object_, cluster, object_aligned, align_result_transform);
#endif
    //pcl::console::print_info("Inliers: %i/%i , %i\n", align.getInliers().size(), clusterT->size(), object->size());
    icp_registration(object_aligned, cluster, Final, icp_result_transform, icp_error);
    if (icp_error < icp_bottom_th_)
    {
      //add_pointcloud_to_debug_cloud(*Final, 0, 0, 200);
      //ROS_ERROR("ICP success; score : %f", icp_score);
      Eigen::Matrix4f total_transform = icp_result_transform * align_result_transform;

      // if (camera_flame_id_ != "map") {
      //   transform_result_in_map_flame(total_transform);
      // }

      save_result(total_transform, icp_bottom_th_ - icp_error);
      //ROS_ERROR("save the result in results_vector.");

      //if (icp_error < first_minimum_error_)
      if (icp_error < minimum_error_ + FPFH_priority_score_)
      {
        //first_minimum_error_ = icp_error;
        minimum_error_ = icp_error;
        update_best_result(total_transform, icp_error);
        std::lock_guard<std::mutex> lock_guad(cloud_update_mutex_);
        pcl::copyPointCloud(*Final, *best_result_cloud_);
        best_fpfh_updated_ = true;
        best_result_updated_ = true;
      }

      return true;
    }
    else
    {
      add_pointcloud_to_debug_cloud(*Final, 100, 100, 100);
      //ROS_INFO("ICP fitting socre was lower than threshold : %f", icp_error);
    }
#ifdef FPFH_MATCHING
  }
  else
  {
    //ROS_INFO("FPFH features mismatch");
  }
#endif
  return false;
}

void ex_segmentor::random_rotation()
{
  //ROS_INFO("start random rotation");
  std::vector<std::thread> threads;
  extrime_icp_error_ = 100.0f;
  //Eigen::Matrix4f lowest_icp_result_transform;
  float best_result_x = best_result_.transform(0, 3);
  float best_result_y = best_result_.transform(1, 3);
  float best_result_z = best_result_.transform(2, 3);
  Eigen::Matrix4f prev_best_result_rot = best_result_.transform;
  prev_best_result_rot(0, 3) = 0.0f;
  prev_best_result_rot(1, 3) = 0.0f;
  prev_best_result_rot(2, 3) = 0.0f;
  // if(!prev_best_result_rot.isUnitary())
  // {
  //   std::cout << "isUnitary1 :" << prev_best_result_rot.isUnitary() << std::endl;
  //   return;
  // }

  for (size_t i = 0; i < 6; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      size_t k = 0;
      //for (size_t k = 0; k < 2; k++)
      //{
        //ROS_INFO("now:%d %d %d", i, j, k);
        Eigen::Matrix4f rotation_matrix_X;
        Eigen::Matrix4f rotation_matrix_Y;
        Eigen::Matrix4f rotation_matrix_Z;
        double theta_X = M_PI / 3.0 * 0.0;
        double theta_Y = M_PI / 3.0 * 0.0;
        double theta_Z = M_PI / 3.0 * i;
        //行列を作成する 4x4
        rotation_matrix_X << 1, 0, 0, 0, \ 
                0,
            cos(theta_X), -sin(theta_X), 0,
            0, sin(theta_X), cos(theta_X), 0,
            0, 0, 0, 1;

        rotation_matrix_Y << cos(theta_Y), 0, sin(theta_Y), 0, \ 
                0,
            1, 0, 0,
            -sin(theta_Y), 0, cos(theta_Y), 0,
            0, 0, 0, 1;

        rotation_matrix_Z << cos(theta_Z), -sin(theta_Z), 0, 0, \ 
                sin(theta_Z),
            cos(theta_Z), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        //Eigen::Matrix4f total_rotation = rotation_matrix_X * rotation_matrix_Y * rotation_matrix_Z * prev_best_result_rot;
        Eigen::Matrix4f total_rotation = rotation_matrix_Z;

        // if (!total_rotation.isUnitary())
        // {
        //   std::cout << "isUnitary2 :" << total_rotation.isUnitary() << std::endl;
        //   continue;
        // }

        total_rotation(0, 3) = best_result_x;
        total_rotation(1, 3) = best_result_y;
        total_rotation(2, 3) = best_result_z - 0.05f * j;

        threads.emplace_back(std::thread(&ex_segmentor::three_steps_ICP_registration, this, std::ref(total_rotation)));

      //}
    }
  }
  for (auto &thread : threads)
  {
    thread.join();
  }
}

void ex_segmentor::three_steps_ICP_registration(Eigen::Matrix4f input_matrix)
{
  std::cout << "start three steps ICP" << std::endl;
  //std::lock_guard<std::mutex> lock(mutex_best_cloud_);
  
  //回転
  pcl::PointCloud<PointXYZRGB>::Ptr rotated(new pcl::PointCloud<PointXYZRGB>);
  pcl::transformPointCloud(*object_, *rotated, input_matrix);
  {
    std::lock_guard<std::mutex> lock(mutex_best_cloud_);
    add_pointcloud_to_debug_cloud(*rotated);
  }

  float first_icp_error, second_icp_error, extrime_icp_error;
  Eigen::Matrix4f first_icp_transform, second_icp_transform, extrime_icp_transform;
  pcl::PointCloud<PointXYZRGB>::Ptr temp_icp_cloud(new pcl::PointCloud<PointXYZRGB>);
  icp_registration(rotated, scene_, temp_icp_cloud, first_icp_transform, first_icp_error, 20, 0.03f, 0.01f);
  //std::cout << first_icp_error << std::endl;
  if (first_icp_error < 0.1)
  {
    icp_registration(temp_icp_cloud, scene_, temp_icp_cloud, second_icp_transform, second_icp_error, 20, 0.02f, 0.01f);
    icp_registration(temp_icp_cloud, scene_, temp_icp_cloud, extrime_icp_transform, extrime_icp_error, 30, 0.017f, 0.01f);
    if (extrime_icp_error < extrime_icp_error_)
    {
      extrime_icp_error_ = extrime_icp_error;
    }
    if (extrime_icp_error < minimum_error_)
    {
      Eigen::Matrix4f total_icp_transform = extrime_icp_transform * second_icp_transform * first_icp_transform * input_matrix;
      if(total_icp_transform.isUnitary())
      {
        //std::lock_guard<std::mutex> lock(mutex_best_cloud_);
        pcl::copyPointCloud(*temp_icp_cloud, *best_result_cloud_);
        update_best_result(total_icp_transform, extrime_icp_error);
        best_result_updated_ = true;
      }
    }
  }
}

} // namespace segmentor