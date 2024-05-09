#include "voxel_mapping/map_server.h"

namespace voxel_mapping {
MapServer::MapServer(ros::NodeHandle &nh) {

  tsdf_.reset(new TSDF());
  esdf_.reset(new ESDF());
  occupancy_grid_.reset(new OccupancyGrid());
  feature_map_.reset(new FeatureMap());
  transformer_.reset(new Transformer(nh));

  /* ------------------------ Map general configuration ----------------------- */

  MapConfig &map_config = tsdf_->map_config_;
  nh.param("voxel_mapping/resolution", map_config.resolution_, 0.1);
  nh.param("voxel_mapping/obstacles_inflation", map_config.obstacles_inflation_, 0.0);
  nh.param("voxel_mapping/local_bound_inflate", map_config.local_bound_inflate_, 0.0);
  nh.param("voxel_mapping/local_map_margin", map_config.local_map_margin_, 0.0);
  nh.param("voxel_mapping/default_dist", map_config.default_dist_, 5.0);
  nh.param("voxel_mapping/optimistic", map_config.optimistic_, true);
  nh.param("voxel_mapping/signed_dist", map_config.signed_dist_, false);
  vector<string> axis = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    nh.param("voxel_mapping/map_min_" + axis[i], map_config.map_min_[i], 0.0);
    nh.param("voxel_mapping/map_max_" + axis[i], map_config.map_max_[i], 0.0);
    nh.param("voxel_mapping/box_min_" + axis[i], map_config.box_min_[i], map_config.map_min_[i]);
    nh.param("voxel_mapping/box_max_" + axis[i], map_config.box_max_[i], map_config.map_max_[i]);
    nh.param("voxel_mapping/vbox_min_" + axis[i], map_config.vbox_min_[i], map_config.map_min_[i]);
    nh.param("voxel_mapping/vbox_max_" + axis[i], map_config.vbox_max_[i], map_config.map_max_[i]);
  }

  // Check map size configuration
  for (int i = 0; i < 3; ++i) {
    CHECK_LT(map_config.map_min_[i], map_config.map_max_[i]);
    CHECK_LT(map_config.box_min_[i], map_config.box_max_[i]);
    CHECK_LT(map_config.vbox_min_[i], map_config.vbox_max_[i]);
    CHECK_LE(map_config.map_min_[i], map_config.box_min_[i]);
    CHECK_GE(map_config.map_max_[i], map_config.box_max_[i]);
    CHECK_LE(map_config.map_min_[i], map_config.vbox_min_[i]);
    CHECK_GE(map_config.map_max_[i], map_config.vbox_max_[i]);
  }

  map_config.map_size_ = map_config.map_max_ - map_config.map_min_;
  map_config.resolution_inv_ = 1 / map_config.resolution_;
  map_config.local_bound_inflate_ = max(map_config.resolution_, map_config.local_bound_inflate_);
  for (int i = 0; i < 3; ++i)
    map_config.map_size_idx_(i) = ceil(map_config.map_size_(i) / map_config.resolution_);
  tsdf_->positionToIndex(map_config.box_min_, map_config.box_min_idx_);
  tsdf_->positionToIndex(map_config.box_max_, map_config.box_max_idx_);
  tsdf_->positionToIndex(map_config.vbox_min_, map_config.vbox_min_idx_);
  tsdf_->positionToIndex(map_config.vbox_max_, map_config.vbox_max_idx_);

  // Use same general configuration
  esdf_->map_config_ = map_config;
  occupancy_grid_->map_config_ = map_config;

  /* ----------------------- Map specific configurations ---------------------- */

  // Map server configuration
  nh.param("voxel_mapping/fx", config_.fx_, 0.0);
  nh.param("voxel_mapping/fy", config_.fy_, 0.0);
  nh.param("voxel_mapping/cx", config_.cx_, 0.0);
  nh.param("voxel_mapping/cy", config_.cy_, 0.0);
  nh.param("voxel_mapping/depth_filter_margin", config_.depth_filter_margin_, 0);
  nh.param("voxel_mapping/depth_scaling_factor", config_.depth_scaling_factor_, 1000.0);
  config_.depth_scaling_factor_inv_ = 1.0 / config_.depth_scaling_factor_;
  nh.param("voxel_mapping/skip_pixel", config_.skip_pixel_, 0);
  nh.param("voxel_mapping/publish_tsdf", config_.publish_tsdf_, false);
  nh.param("voxel_mapping/publish_esdf", config_.publish_esdf_, false);
  nh.param("voxel_mapping/publish_occupancy_grid", config_.publish_occupancy_grid_, false);
  nh.param("voxel_mapping/publish_feature_map", config_.publish_feature_map_, false);

  nh.param("voxel_mapping/publish_tsdf_slice", config_.publish_tsdf_slice_, false);
  nh.param("voxel_mapping/publish_esdf_slice", config_.publish_esdf_slice_, false);
  nh.param("voxel_mapping/publish_occupancy_grid_slice", config_.publish_occupancy_grid_slice_,
           false);
  nh.param("voxel_mapping/publish_tsdf_period", config_.publish_tsdf_period_, 1.0);
  nh.param("voxel_mapping/publish_esdf_period", config_.publish_esdf_period_, 1.0);
  nh.param("voxel_mapping/publish_occupancy_grid_period", config_.publish_occupancy_grid_period_,
           1.0);
  nh.param("voxel_mapping/tsdf_slice_height", config_.tsdf_slice_height_, -1.0);
  nh.param("voxel_mapping/tsdf_slice_visualization_height",
           config_.tsdf_slice_visualization_height_, -1.0);
  nh.param("voxel_mapping/esdf_slice_height", config_.esdf_slice_height_, -1.0);
  nh.param("voxel_mapping/esdf_slice_visualization_height",
           config_.esdf_slice_visualization_height_, -1.0);
  nh.param("voxel_mapping/occupancy_grid_slice_height", config_.occupancy_grid_slice_height_, -1.0);
  nh.param("voxel_mapping/world_frame", config_.world_frame_, string("world"));
  nh.param("voxel_mapping/sensor_frame", config_.sensor_frame_, string("camera"));

  nh.param("voxel_mapping/verbose", config_.verbose_, false);
  nh.param("voxel_mapping/verbose_time", config_.verbose_time_, false);

  string mode_string;
  nh.param("voxel_mapping/mode", mode_string, string(""));
  if (mode_string == "airsim")
    config_.mode_ = MapServer::Config::Mode::AIRSIM;
  else if (mode_string == "uav_simulator")
    config_.mode_ = MapServer::Config::Mode::UAV_SIMULATOR;
  else if (mode_string == "real")
    config_.mode_ = MapServer::Config::Mode::REAL;
  else
    CHECK(false) << "Unknown mode: " << mode_string;

  // TSDF configuration
  TSDF::Config &tsdf_config = tsdf_->config_;
  nh.param("voxel_mapping/tsdf/truncated_dist", tsdf_config.truncated_dist_, 0.1);
  nh.param("voxel_mapping/tsdf/truncated_dist_behind", tsdf_config.truncated_dist_behind_, 0.2);
  nh.param("voxel_mapping/tsdf/raycast_min", tsdf_config.raycast_min_, 0.0);
  nh.param("voxel_mapping/tsdf/raycast_max", tsdf_config.raycast_max_, 5.0);
  nh.param("voxel_mapping/tsdf/result_truncated_dist", tsdf_config.result_truncated_dist_, 0.2);
  nh.param("voxel_mapping/tsdf/epsilon", tsdf_config.epsilon_, 1e-4);
  if (config_.mode_ == MapServer::Config::Mode::AIRSIM)
    tsdf_config.depth_axis_ = TSDF::Config::DepthAxis::Y;
  else
    tsdf_config.depth_axis_ = TSDF::Config::DepthAxis::Z;

  // ESDF configuration

  // Occupancy grid configuration
  OccupancyGrid::Config &occupancy_grid_config = occupancy_grid_->config_;
  string occupancy_grid_mode;
  nh.param("voxel_mapping/occupancy_grid/TSDF_cutoff_dist", occupancy_grid_config.TSDF_cutoff_dist_,
           0.1);
  nh.param("voxel_mapping/occupancy_grid/occupancy_grid_mode", occupancy_grid_mode,
           string("GEN_TSDF"));
  if (occupancy_grid_mode == "GEN_TSDF")
    occupancy_grid_config.mode_ = OccupancyGrid::Config::MODE::GEN_TSDF;
  else if (occupancy_grid_mode == "GEN_PCL")
    occupancy_grid_config.mode_ = OccupancyGrid::Config::MODE::GEN_PCL;
  else
    ROS_ERROR("Unknown occupancy grid mode: %s", occupancy_grid_mode.c_str());

  // FeatureMap configuration
  FeatureMap::Config &feature_map_config = feature_map_->config_;
  nh.param("voxel_mapping/feature_map/depth_min", feature_map_config.depth_min_, 0.0);
  nh.param("voxel_mapping/feature_map/depth_max", feature_map_config.depth_max_, 5.0);

  /* ----------------------------- Initialization ----------------------------- */

  // Initialize TSDF
  tsdf_->initMapData();
  tsdf_->initRaycaster();
  tsdf_->setESDF(esdf_);
  tsdf_->setOccupancyGrid(occupancy_grid_);

  // Initialize ESDF
  esdf_->initMapData();
  esdf_->initRaycaster();
  esdf_->setTSDF(tsdf_);
  esdf_->setOccupancyGrid(occupancy_grid_);

  // Initialize occupancy grid
  occupancy_grid_->initMapData();
  occupancy_grid_->initRaycaster();
  occupancy_grid_->setTSDF(tsdf_);

  // Initialize publishers
  tsdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/tsdf", 10);
  tsdf_slice_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/tsdf_slice", 10);
  esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/esdf", 10);
  esdf_slice_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/esdf_slice", 10);
  occupancy_grid_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/occupancy_grid", 10);
  feature_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/feature_map", 10);
  interpolated_pose_pub_ = nh.advertise<nav_msgs::Odometry>("/voxel_mapping/interpolated_pose", 10);
  feature_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/features", 10);
  depth_pointcloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/voxel_mapping/depth_pointcloud", 10);

  // Initialize callbacks
  // depth_sub_ =
  //     nh.subscribe("/voxel_mapping/depth_image_topic", 10, &MapServer::depthCallback, this);
  pointcloud_sub_ =
      nh.subscribe("/voxel_mapping/pointcloud", 100, &MapServer::globalMapCallback, this);
  // feature_cloud_sub_ =
  //     nh.subscribe("/voxel_mapping/feature_cloud", 100, &MapServer::featureCloudCallback, this);
  debug_visualization_pub_ =
      nh.advertise<visualization_msgs::Marker>("/voxel_mapping/debug_visualization", 10);

  // Initialize timer
  publish_map_timer_ =
      nh.createTimer(ros::Duration(0.05), &MapServer::publishMapTimerCallback, this);

  if (config_.verbose_) {
    ROS_INFO("[MapServer] Voxel mapping server initialized with parameters: ");
    ROS_INFO("  - Resolution: %.2f", map_config.resolution_);
    ROS_INFO("  - Map size: %.2f x %.2f x %.2f", map_config.map_size_(0), map_config.map_size_(1),
             map_config.map_size_(2));
    ROS_INFO("  - Map size index: %d x %d x %d", map_config.map_size_idx_(0),
             map_config.map_size_idx_(1), map_config.map_size_idx_(2));
    ROS_INFO("  - Box min: %.2f , %.2f , %.2f", map_config.box_min_(0), map_config.box_min_(1),
             map_config.box_min_(2));
    ROS_INFO("  - Box max: %.2f , %.2f , %.2f", map_config.box_max_(0), map_config.box_max_(1),
             map_config.box_max_(2));
    ROS_INFO("  - Box min index: %d , %d , %d", map_config.box_min_idx_(0),
             map_config.box_min_idx_(1), map_config.box_min_idx_(2));
    ROS_INFO("  - Box max index: %d , %d , %d", map_config.box_max_idx_(0),
             map_config.box_max_idx_(1), map_config.box_max_idx_(2));
    ROS_INFO("  - Visualizing Box min: %.2f , %.2f , %.2f", map_config.vbox_min_(0),
             map_config.vbox_min_(1), map_config.vbox_min_(2));
    ROS_INFO("  - Visualizing Box max: %.2f , %.2f , %.2f", map_config.vbox_max_(0),
             map_config.vbox_max_(1), map_config.vbox_max_(2));
    ROS_INFO("  - Visualizing Box min index: %d , %d , %d", map_config.vbox_min_idx_(0),
             map_config.vbox_min_idx_(1), map_config.vbox_min_idx_(2));
    ROS_INFO("  - Visualizing Box max index: %d , %d , %d", map_config.vbox_max_idx_(0),
             map_config.vbox_max_idx_(1), map_config.vbox_max_idx_(2));
    ROS_INFO("  - TSDF truncated distance: %.2f", tsdf_config.truncated_dist_);
  }

  pos_min_ = Eigen::Vector3d::Zero();
  pos_max_ = Eigen::Vector3d::Zero();
}

void MapServer::depthCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  if (config_.verbose_) {
    ROS_INFO("[MapServer] Received depth image with stamp: %f", image_msg->header.stamp.toSec());
  }

  TicToc tic1;
  cv::Mat depth_image;
  PointCloudType pointcloud;
  Transformation sensor_pose;
  ros::Time img_stamp;

  image_queue_.push(image_msg);

  while (getNextImageFromQueue(image_queue_, depth_image, sensor_pose, img_stamp)) {
    depthToPointcloud(depth_image, sensor_pose, pointcloud, img_stamp);

    for (int i = 0; i < 3; i++) {
      pos_min_(i) = min(pos_min_(i), sensor_pose.getPosition()(i));
      pos_max_(i) = max(pos_max_(i), sensor_pose.getPosition()(i));
    }

    if (config_.verbose_time_) {
      ROS_INFO("[MapServer] pos_min: %.2f, %.2f, %.2f", pos_min_(0), pos_min_(1), pos_min_(2));
      ROS_INFO("[MapServer] pos_max: %.2f, %.2f, %.2f", pos_max_(0), pos_max_(1), pos_max_(2));
    }
    publishDebugVisualizationBBox(pos_min_, pos_max_);
    publishInterpolatedPose(sensor_pose, img_stamp);

    TicToc tic2;
    tsdf_->inputPointCloud(pointcloud);

    if (config_.verbose_time_) {
      ROS_INFO("[MapServer] Depth pointcloud size: %d", (int)pointcloud.size());
      ROS_INFO("[MapServer] Depth pointcloud process time: %fs", tic2.toc());
    }
  }

  if (config_.verbose_time_) {
    ROS_INFO("[MapServer] Depth callback time: %fs", tic1.toc());
  }
}

void MapServer::globalMapCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  if (load_map_from_pcd_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    occupancy_grid_->loadMapFromPcd(cloud);
    load_map_from_pcd_ = false;
    publishOccupancyGrid();
  }
}

void MapServer::publishMapTimerCallback(const ros::TimerEvent &event) {
  if (config_.publish_tsdf_)
    publishTSDF();

  if (config_.publish_tsdf_slice_)
    publishTSDFSlice();

  if (config_.publish_esdf_slice_)
    publishESDFSlice();

  if (config_.publish_occupancy_grid_)
    publishOccupancyGrid();

  if (config_.publish_feature_map_)
    publishFeatureMap();
}

void MapServer::publishTSDF() {
  if (tsdf_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing TSDF...");

  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pcl::PointXYZI point;
  const FloatingPoint min_dist = -tsdf_->config_.result_truncated_dist_;
  const FloatingPoint max_dist = tsdf_->config_.result_truncated_dist_;
  // const FloatingPoint min_dist = -tsdf_->config_.truncated_dist_;
  // const FloatingPoint max_dist = tsdf_->config_.truncated_dist_;

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      for (int z = tsdf_->map_config_.vbox_min_idx_[2]; z < tsdf_->map_config_.vbox_max_idx_[2];
           ++z) {
        VoxelIndex idx(x, y, z);
        Position pos = tsdf_->indexToPosition(idx);

        // unknown or cleared voxel
        if (tsdf_->getVoxel(idx).weight < tsdf_->config_.epsilon_)
          continue;

        // only publish occupied voxel
        if (tsdf_->getVoxel(idx).value > max_dist || tsdf_->getVoxel(idx).value < min_dist)
          continue;

        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        point.intensity = (tsdf_->getVoxel(idx).value - min_dist) / (max_dist - min_dist);
        pointcloud.points.push_back(point);
      }
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  tsdf_pub_.publish(pointcloud_msg);
}

void MapServer::publishTSDFSlice() {
  if (tsdf_slice_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing TSDF slice...");

  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pcl::PointXYZI point;
  const FloatingPoint min_dist = -tsdf_->config_.truncated_dist_;
  const FloatingPoint max_dist = tsdf_->config_.truncated_dist_;

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      VoxelIndex idx(x, y, 0);
      Position pos = tsdf_->indexToPosition(idx);
      pos.z() = config_.tsdf_slice_height_;
      tsdf_->positionToIndex(pos, idx);

      // unknown or cleared voxel
      if (tsdf_->getVoxel(idx).weight < tsdf_->config_.epsilon_)
        continue;

      point.x = pos(0);
      point.y = pos(1);
      point.z = config_.tsdf_slice_visualization_height_;
      point.intensity = (tsdf_->getVoxel(idx).value - min_dist) / (max_dist - min_dist);
      pointcloud.points.push_back(point);
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  tsdf_slice_pub_.publish(pointcloud_msg);
}

void MapServer::publishESDFSlice() {
  if (esdf_slice_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing ESDF slice...");

  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pcl::PointXYZI point;

  for (int x = esdf_->map_config_.vbox_min_idx_[0]; x < esdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = esdf_->map_config_.vbox_min_idx_[1]; y < esdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      VoxelIndex idx(x, y, 0);
      Position pos = esdf_->indexToPosition(idx);
      pos.z() = config_.esdf_slice_height_;
      esdf_->positionToIndex(pos, idx);

      if (occupancy_grid_->getVoxel(idx).value == OccupancyType::UNKNOWN)
        continue;

      point.x = pos(0);
      point.y = pos(1);
      point.z = config_.esdf_slice_visualization_height_;
      point.intensity = esdf_->getVoxel(idx).value;
      pointcloud.points.push_back(point);
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  esdf_slice_pub_.publish(pointcloud_msg);
}

void MapServer::publishOccupancyGrid() {
  if (occupancy_grid_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO_THROTTLE(1.0, "[MapServer] Publishing occupancy grid...");

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::PointXYZ point;

  for (int x = tsdf_->map_config_.vbox_min_idx_[0]; x < tsdf_->map_config_.vbox_max_idx_[0]; ++x) {
    for (int y = tsdf_->map_config_.vbox_min_idx_[1]; y < tsdf_->map_config_.vbox_max_idx_[1];
         ++y) {
      for (int z = tsdf_->map_config_.vbox_min_idx_[2]; z < tsdf_->map_config_.vbox_max_idx_[2];
           ++z) {
        VoxelIndex idx(x, y, z);
        Position pos = occupancy_grid_->indexToPosition(idx);

        // unknown or cleared voxel
        if (occupancy_grid_->getVoxel(idx).value == OccupancyType::OCCUPIED) {
          point.x = pos(0);
          point.y = pos(1);
          point.z = pos(2);
          pointcloud.points.push_back(point);
        }
      }
    }
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  occupancy_grid_pub_.publish(pointcloud_msg);
}

void MapServer::publishFeaturePointcloud(const AlignedVector<Position> features) {
  if (feature_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing feature pointcloud...");

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::PointXYZ point;
  for (Position feature : features) {
    point.x = feature[0];
    point.y = feature[1];
    point.z = feature[2];
    pointcloud.points.push_back(point);
  }

  pointcloud.width = pointcloud.points.size();
  pointcloud.height = 1;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  feature_pub_.publish(pointcloud_msg);
}

void MapServer::publishFeatureMap() {
  if (feature_map_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing feature pointcloud...");

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
  feature_map_->getFeatureCloud(pointcloud);

  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true;
  pointcloud->header.frame_id = config_.world_frame_;

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  feature_map_pub_.publish(pointcloud_msg);
}

void MapServer::publishDepthPointcloud(const PointCloudType &pointcloud,
                                       const ros::Time &img_stamp) {
  if (depth_pointcloud_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing depth pointcloud...");

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header.stamp = img_stamp;
  pointcloud_msg.header.frame_id = config_.sensor_frame_;
  depth_pointcloud_pub_.publish(pointcloud_msg);
}
void MapServer::publishInterpolatedPose(const Transformation &sensor_pose,
                                        const ros::Time &img_stamp) {
  nav_msgs::Odometry odo_msg;
  odo_msg.header.stamp = img_stamp;
  odo_msg.header.frame_id = config_.world_frame_;
  odo_msg.pose.pose.position.x = sensor_pose.getPosition().x();
  odo_msg.pose.pose.position.y = sensor_pose.getPosition().y();
  odo_msg.pose.pose.position.z = sensor_pose.getPosition().z();
  odo_msg.pose.pose.orientation.x = sensor_pose.getRotation().x();
  odo_msg.pose.pose.orientation.y = sensor_pose.getRotation().y();
  odo_msg.pose.pose.orientation.z = sensor_pose.getRotation().z();
  odo_msg.pose.pose.orientation.w = sensor_pose.getRotation().w();
  interpolated_pose_pub_.publish(odo_msg);
}

void MapServer::publishDebugVisualizationBBox(const Position &bbox_min, const Position &bbox_max) {
  if (debug_visualization_pub_.getNumSubscribers() == 0)
    return;

  if (config_.verbose_)
    ROS_INFO("[MapServer] Publishing debug bbox...");

  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.world_frame_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = (bbox_min(0) + bbox_max(0)) / 2;
  marker.pose.position.y = (bbox_min(1) + bbox_max(1)) / 2;
  marker.pose.position.z = (bbox_min(2) + bbox_max(2)) / 2;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = bbox_max(0) - bbox_min(0);
  marker.scale.y = bbox_max(1) - bbox_min(1);
  marker.scale.z = bbox_max(2) - bbox_min(2);
  marker.color.a = 0.2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  debug_visualization_pub_.publish(marker);
}

void MapServer::depthToPointcloud(const cv::Mat &depth_image, const Transformation &T_w_c,
                                  PointCloudType &pointcloud, const ros::Time &img_stamp) {
  Position pt_c, pt_w;
  const uint16_t *row_ptr;
  FloatingPoint depth;
  int cols = depth_image.cols;
  int rows = depth_image.rows;
  int points_num = 0;

  pointcloud.points.resize(cols * rows / (config_.skip_pixel_ * config_.skip_pixel_));

  for (int v = config_.depth_filter_margin_; v < rows - config_.depth_filter_margin_;
       v += config_.skip_pixel_) {
    //  Pointer to the first element (exclude the margin) in n-th row
    row_ptr = depth_image.ptr<uint16_t>(v) + config_.depth_filter_margin_;
    for (int u = config_.depth_filter_margin_; u < cols - config_.depth_filter_margin_;
         u += config_.skip_pixel_) {
      depth = (*row_ptr) * 0.001; // mm to m
      row_ptr = row_ptr + config_.skip_pixel_;
      if (depth == 0)
        continue;

      // double z = -(v - config_.cy_) * depth / config_.fy_;
      // if (z < 0.1)
      //   continue;

      // Point in camera frame
      if (config_.mode_ == MapServer::Config::Mode::AIRSIM) {
        // x right, y forward, z up
        pointcloud.points[points_num].x = (u - config_.cx_) * depth / config_.fx_;
        pointcloud.points[points_num].y = depth;
        pointcloud.points[points_num].z = -(v - config_.cy_) * depth / config_.fy_;
      } else {
        // x right, y down, z forward
        pointcloud.points[points_num].x = (u - config_.cx_) * depth / config_.fx_;
        pointcloud.points[points_num].y = (v - config_.cy_) * depth / config_.fy_;
        pointcloud.points[points_num].z = depth;
      }

      points_num++;
    }
  }

  pointcloud.points.resize(points_num);
  pointcloud.points.shrink_to_fit();
  pointcloud.sensor_origin_.head<3>() = T_w_c.getPosition().cast<float>();
  pointcloud.sensor_orientation_ = T_w_c.getRotationMatrix().cast<float>();

  publishDepthPointcloud(pointcloud, img_stamp);
}

bool MapServer::getNextImageFromQueue(std::queue<sensor_msgs::ImageConstPtr> &queue,
                                      cv::Mat &depth_image, Transformation &sensor_pose,
                                      ros::Time &img_stamp) {
  if (queue.empty()) {
    return false;
  }

  if (config_.verbose_)
    ROS_INFO("[MapServer] Getting next image from queue...");

  sensor_msgs::ImageConstPtr image_msg;
  const int kMaxQueueSize = 10;

  image_msg = queue.front();
  if (transformer_->lookupTransform(image_msg->header.stamp, sensor_pose)) {
    if (config_.verbose_)
      ROS_INFO("[MapServer] Found sensor pose for image timestamp: %f",
               image_msg->header.stamp.toSec());

    // TODO: maybe no need for cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    if (image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, config_.depth_scaling_factor_);
    cv_ptr->image.copyTo(depth_image);
    img_stamp = image_msg->header.stamp;

    queue.pop();
    return true;
  } else {
    if (config_.verbose_)
      ROS_ERROR("[MapServer] Failed to lookup transform for image");
    if (queue.size() >= kMaxQueueSize) {
      if (config_.verbose_)
        ROS_ERROR("[MapServer] Queue size is too large, dropping oldest image");
      while (queue.size() >= kMaxQueueSize) {
        queue.pop();
      }
    }
  }

  return false;
}

bool MapServer::getNextPointcloudFromQueue(std::queue<sensor_msgs::PointCloud2ConstPtr> &queue,
                                           PointCloudType &pointcloud_msg,
                                           Transformation &sensor_pose) {
  return false;
}

bool MapServer::getNextFeatureCloudFromQueue(std::queue<sensor_msgs::PointCloud2ConstPtr> &queue,
                                             PointCloudType &pointcloud_msg,
                                             Transformation &sensor_pose) {
  if (queue.empty()) {
    return false;
  }

  if (config_.verbose_)
    ROS_INFO("[MapServer] Getting next feature pointcloud from queue...");

  sensor_msgs::PointCloud2ConstPtr msg;
  const int kMaxQueueSize = 10;

  msg = queue.front();
  if (transformer_->lookupTransform(msg->header.stamp, sensor_pose)) {
    if (config_.verbose_)
      ROS_INFO("[MapServer] Found sensor pose for feature pointcloud timestamp: %f",
               msg->header.stamp.toSec());

    // Convert sensor_msgs::PointCloud2 to PointCloudType(pcl::PointCloud<pcl::PointXYZ>)
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pointcloud_msg);

    queue.pop();
    return true;
  } else {
    if (config_.verbose_)
      ROS_ERROR("[MapServer] Failed to lookup transform for feature pointcloud");
    if (queue.size() >= kMaxQueueSize) {
      if (config_.verbose_)
        ROS_ERROR("[MapServer] Queue size is too large, dropping oldest feature pointcloud");
      while (queue.size() >= kMaxQueueSize) {
        queue.pop();
      }
    }
  }

  return false;
}

// Old API
void MapServer::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size) {
  ori = tsdf_->map_config_.map_min_;
  size = tsdf_->map_config_.map_size_;
}

Position MapServer::getOrigin() { return tsdf_->map_config_.map_min_; }

double MapServer::getResolution() { return tsdf_->map_config_.resolution_; };

int MapServer::getVoxelNum() { return tsdf_->map_config_.map_size_idx_.prod(); };

void MapServer::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax) {
  bmin = tsdf_->map_config_.box_min_;
  bmax = tsdf_->map_config_.box_max_;
}

bool MapServer::isInBox(const Position &pos) { return tsdf_->isInBox(pos); };
bool MapServer::isInBox(const VoxelIndex &idx) { return tsdf_->isInBox(idx); };

OccupancyType MapServer::getOccupancy(const Eigen::Vector3d &pos) {
  return occupancy_grid_->getVoxel(pos).value;
}

OccupancyType MapServer::getOccupancy(const Eigen::Vector3i &idx) {
  return occupancy_grid_->getVoxel(idx).value;
}

void MapServer::getExploredRegion(Position &bbox_min, Position &bbox_max) {
  bbox_min = pos_min_;
  bbox_max = pos_max_;
}

void MapServer::saveMap(const string &filename) {
  std::cout << "[MapServer] Saving map to " << filename << std::endl;
  occupancy_grid_->saveMap(filename);
  std::cout << "[MapServer] Done" << std::endl;
}

void MapServer::loadMap(const string &filename) {
  occupancy_grid_->loadMap(filename);
  std::cout << "[MapServer] Finished loading occupancy map." << std::endl;
}

void MapServer::loadMap(const string &filename_occu, const string &filename_esdf,
                        const string &filename_feature) {
  occupancy_grid_->loadMap(filename_occu);
  esdf_->loadMap(filename_esdf);
  feature_map_->loadMap(filename_feature);
  std::cout << "[MapServer] Finished loading occupancy and esdf map." << std::endl;
}

void MapServer::scaleColor(const double value, const double max_value, const double min_value,
                           double &color_value) {
  // Scale the value to the range [0, 1]
  double scaled_value = (value - min_value) / (max_value - min_value);
  // Apply the color map to convert the value to a grayscale color
  double color_map_scale = 1.0;
  color_value = color_map_scale * scaled_value;
}
} // namespace voxel_mapping