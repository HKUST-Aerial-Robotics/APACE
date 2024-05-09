#ifndef TSDF_SERVER_H
#define TSDF_SERVER_H

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "tic_toc.h"
#include "transformer/transformer.h"
#include "voxel_mapping/esdf.h"
#include "voxel_mapping/feature_map.h"
#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

using std::shared_ptr;
using std::string;

namespace voxel_mapping {
class MapServer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<MapServer> Ptr;
  typedef shared_ptr<const MapServer> ConstPtr;

  struct Config {
    enum Mode { AIRSIM, UAV_SIMULATOR, REAL };

    Mode mode_;
    double fx_, fy_, cx_, cy_;
    int image_width_, image_height_;
    int depth_filter_margin_;
    double depth_scaling_factor_, depth_scaling_factor_inv_;
    int skip_pixel_;

    bool publish_tsdf_, publish_esdf_, publish_occupancy_grid_, publish_feature_map_;
    bool publish_tsdf_slice_, publish_esdf_slice_, publish_occupancy_grid_slice_;
    double publish_tsdf_period_, publish_esdf_period_, publish_occupancy_grid_period_;
    double tsdf_slice_height_, esdf_slice_height_, occupancy_grid_slice_height_;
    double tsdf_slice_visualization_height_, esdf_slice_visualization_height_,
        occupancy_grid_slice_visualization_height_;

    string world_frame_, sensor_frame_;

    bool verbose_, verbose_time_;
  };

  MapServer(ros::NodeHandle &nh);

  TSDF::Ptr getTSDF() { return tsdf_; }
  ESDF::Ptr getESDF() { return esdf_; }
  OccupancyGrid::Ptr getOccupancyGrid() { return occupancy_grid_; }
  Transformer::Ptr getTransformer() { return transformer_; }
  FeatureMap::Ptr getFeatureMap() { return feature_map_; }

  void depthCallback(const sensor_msgs::ImageConstPtr &img);
  void globalMapCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void featureCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void covarianceCallback(const sensor_msgs::PointCloud2ConstPtr &pcd_msg,
                          const nav_msgs::OdometryConstPtr &odo_msg);
  void publishMapTimerCallback(const ros::TimerEvent &event);

  void publishTSDF();
  void publishTSDFSlice();
  void publishESDF();
  void publishESDFSlice();
  void publishOccupancyGrid();
  void publishOccupancyGridSlice();
  void publishFeatureMap();
  void publishFeaturePointcloud(const AlignedVector<Position> features);
  void publishDepthPointcloud(const PointCloudType &pointcloud, const ros::Time &img_stamp);
  void publishInterpolatedPose(const Transformation &sensor_pose, const ros::Time &img_stamp);
  void publishDebugVisualizationBBox(const Position &bbox_min, const Position &bbox_max);

  // Transform a depth image to a point cloud under sensor frame
  // sensor pose is stored in pointcloud sensor_origin_ and sensor_orientation_
  void depthToPointcloud(const cv::Mat &depth_image, const Transformation &T_w_c,
                         PointCloudType &pointcloud, const ros::Time &img_stamp);
  bool getNextImageFromQueue(std::queue<sensor_msgs::ImageConstPtr> &queue, cv::Mat &depth_image,
                             Transformation &sensor_pose, ros::Time &img_stamp);
  bool getNextPointcloudFromQueue(std::queue<sensor_msgs::PointCloud2ConstPtr> &queue,
                                  PointCloudType &pointcloud_msg, Transformation &sensor_pose);
  bool getNextFeatureCloudFromQueue(std::queue<sensor_msgs::PointCloud2ConstPtr> &queue,
                                    PointCloudType &pointcloud_msg, Transformation &sensor_pose);

  // Old API
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  Position getOrigin();
  double getResolution();
  int getVoxelNum();
  void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
  bool isInBox(const Position &pos);
  bool isInBox(const VoxelIndex &idx);
  OccupancyType getOccupancy(const Eigen::Vector3d &pos);
  OccupancyType getOccupancy(const Eigen::Vector3i &idx);

  void getExploredRegion(Position &bbox_min, Position &bbox_max);

  void saveMap(const string &filename);
  void loadMap(const string &filename);
  void loadMap(const string &filename_occu, const string &filename_esdf, const string &filename_feature);
  void scaleColor(const double value, const double max_value, const double min_value,
                  double &color_value);

private:
  Config config_;

  ros::Publisher tsdf_pub_, tsdf_slice_pub_;
  ros::Publisher esdf_pub_, esdf_slice_pub_;
  ros::Publisher occupancy_grid_pub_, occupancy_grid_slice_pub_;
  ros::Publisher feature_map_pub_;
  ros::Publisher interpolated_pose_pub_;
  ros::Publisher feature_pub_;
  ros::Publisher depth_pointcloud_pub_;
  ros::Subscriber depth_sub_, pointcloud_sub_, feature_cloud_sub_;
  ros::Timer publish_map_timer_;

  // Debug visualization
  ros::Publisher debug_visualization_pub_;

  TSDF::Ptr tsdf_;
  ESDF::Ptr esdf_;
  OccupancyGrid::Ptr occupancy_grid_;
  FeatureMap::Ptr feature_map_;
  Transformer::Ptr transformer_;

  std::queue<sensor_msgs::ImageConstPtr> image_queue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> pointcloud_queue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> feature_cloud_queue_;

  Position pos_min_, pos_max_;
  bool load_map_from_pcd_;

};
} // namespace voxel_mapping

#endif // TSDF_SERVER_H