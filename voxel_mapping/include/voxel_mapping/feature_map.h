#ifndef FEATURE_MAP_H
#define FEATURE_MAP_H

#include <Eigen/Eigen>
#include <memory>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::string;

namespace voxel_mapping {
class FeatureMap {
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<FeatureMap> Ptr;
  typedef shared_ptr<const FeatureMap> ConstPtr;

  struct Config {
    double depth_min_;
    double depth_max_;
  };

  FeatureMap(){};
  ~FeatureMap(){};

  void loadMap(const string &filename);
  void getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res);
  void getFeaturesIndex(const Eigen::Vector3d &pos, vector<int> &res);
  void indexToFeatures(const vector<int> &index, vector<Eigen::Vector3d> &res);

  Config config_;

private:
  pcl::PointCloud<pcl::PointXYZ> features_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> features_kdtree_;
};
} // namespace voxel_mapping

#endif // FEATURE_MAP_H