#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <string>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_downsampler");

  ros::NodeHandle nh;

  string origin_filename = "/home/cindy/Downloads/icra2024/features_wall.pcd";
  string downsampled_filename = "/home/cindy/Downloads/icra2024/features_wall_downsampled.pcd";

  // Load the point cloud from a pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(origin_filename, *cloud) == -1) {
    std::cout << "Error reading input file." << std::endl;
    return -1;
  }

  // Filter outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
  for (pcl::PointXYZ point : cloud->points) {
    if (point.y > 4.0 && point.y < 22.0 && point.z > 0.0 && point.z < 7.0) {
      cloud_inliers->points.push_back(point);
    }
  }

  // Perform downsampling using VoxelGrid filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(cloud_inliers);
  voxelGrid.setLeafSize(0.5f, 0.5f, 0.5f); // Set the voxel grid leaf size
  voxelGrid.filter(*downsampledCloud);

  // Save the downsampled point cloud
  pcl::io::savePCDFile<pcl::PointXYZ>(downsampled_filename, *downsampledCloud);
  ROS_INFO("Saved to %s.", downsampled_filename.c_str());

  ROS_INFO("Downsampling complete from %ld points to %ld points.", cloud->points.size(),
           downsampledCloud->points.size());

  ros::spin();
  return 0;
}