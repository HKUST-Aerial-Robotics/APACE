#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_data_(new pcl::PointCloud<pcl::PointXYZ>);
std::string pcd_file_ = "/home/cindy/Downloads/icra2024/features_wall.pcd";

void callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcd(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *new_pcd);
  if (new_pcd->points.empty())
    return;
  *pcd_data_ += *new_pcd;
  pcl::io::savePCDFile(pcd_file_, *pcd_data_);
  ROS_INFO("Saved pcd file containing %ld points.", pcd_data_->points.size());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_saver");

  ros::NodeHandle nh;
  // nh.param("pcd_file", pcd_file_, std::string(""));

  ros::Subscriber sub = nh.subscribe("/pointcloud_topic", 1, callback);

  ros::spin();
  return 0;
}