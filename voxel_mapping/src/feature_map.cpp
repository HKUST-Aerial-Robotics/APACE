#include "voxel_mapping/feature_map.h"

namespace voxel_mapping {
void FeatureMap::loadMap(const string &filename) {
  features_cloud_.clear();
  bool use_simple_features = (filename == "") ? true : false;

  if (use_simple_features) {
    // Simple line features at fixed height
    // for (int i = 0; i < 50; i++) {
    //   double step = 20.0 / 50;
    //   pcl::PointXYZ p;
    //   p.x = 5.0;
    //   p.y = i * step;
    //   p.z = 5.0;
    //   features_cloud_.push_back(p);
    // }

    // Features at the central of the wall
    for (double y = 3.0; y < 17.0; y += 0.2) {
      for (double z = 0.0; z < 5.0; z += 0.2) {
        pcl::PointXYZ p;
        p.x = 5.0;
        p.y = y;
        p.z = z;
        features_cloud_.push_back(p);
      }
    }
  } else {
    pcl::io::loadPCDFile<pcl::PointXYZ>(filename, features_cloud_);
  }
  features_kdtree_.setInputCloud(features_cloud_.makeShared());
}

void FeatureMap::getFeatureCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  cloud = features_cloud_.makeShared();
}

void FeatureMap::getFeatures(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &res) {
  if (features_cloud_.points.size() == 0)
    return;

  res.clear();

  pcl::PointXYZ searchPoint;
  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  features_kdtree_.radiusSearch(searchPoint, config_.depth_max_, pointIdxRadiusSearch,
                                pointRadiusSquaredDistance);

  for (int index : pointIdxRadiusSearch) {
    Eigen::Vector3d f(features_cloud_[index].x, features_cloud_[index].y, features_cloud_[index].z);
    if ((f - pos).norm() > config_.depth_min_)
      res.push_back(f);
  }
}

void FeatureMap::getFeaturesIndex(const Eigen::Vector3d &pos, vector<int> &res) {
  if (features_cloud_.points.size() == 0)
    return;

  res.clear();

  pcl::PointXYZ searchPoint;
  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  features_kdtree_.radiusSearch(searchPoint, config_.depth_max_, pointIdxRadiusSearch,
                                pointRadiusSquaredDistance);

  for (int index : pointIdxRadiusSearch) {
    Eigen::Vector3d f(features_cloud_[index].x, features_cloud_[index].y, features_cloud_[index].z);
    if ((f - pos).norm() > config_.depth_min_)
      res.push_back(index);
  }
}

void FeatureMap::indexToFeatures(const vector<int> &index, vector<Eigen::Vector3d> &res) {
  res.clear();
  for (int ind : index) {
    Eigen::Vector3d f(features_cloud_[ind].x, features_cloud_[ind].y, features_cloud_[ind].z);
    res.push_back(f);
  }
}
} // namespace voxel_mapping