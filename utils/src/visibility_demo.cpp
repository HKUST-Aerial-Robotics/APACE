#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>

using namespace std;

// Global variables
enum VisibilityModel { Proposed, Conical };
double fov_h = 90 * M_PI / 180;
double fov_v = 60 * M_PI / 180;
int number_of_samples = 1000000;
ros::Publisher visib_pub, fov_pub;

vector<Eigen::Vector3d> samples;
vector<double> visibility;
vector<double> visibility_conical;

// v1 visibility function
double v1(double sin_theta1) {
  double sin_thr = sin(M_PI / 2.0 - fov_v / 2.0);
  double v1 = 1.0 / (1.0 + exp(-40 * (sin_theta1 - sin_thr)));
  return v1;
}

// v2 visibility function
double v2(double cos_theta2) {
  double cos_thr = 0.0;
  double v2 = 1.0 / (1.0 + exp(-10 * (cos_theta2 - cos_thr)));
  return v2;
}

// v3 visibility function
double v3(double sin_theta3) {
  double sin_thr = sin(M_PI / 2.0 - fov_h / 2.0);
  double v3 = 1.0 / (1.0 + exp(-20 * (sin_theta3 - sin_thr)));
  return v3;
}

// conical visibility function
double v_conical(double cos_theta) {
  double cos_thr = cos(fov_v / 2.0);
  double v = 1.0 / (1.0 + exp(-40 * (cos_theta - cos_thr)));
  return v;
}

void generateSamples() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-1.0, 1.0);

  for (int i = 0; i < number_of_samples; i++) {
    double x, y, z, magnitude;
    do {
      // Generate random coordinates within the unit cube
      x = dis(gen);
      y = dis(gen);
      z = dis(gen);

      // Compute the magnitude of the generated point
      magnitude = std::sqrt(x * x + y * y + z * z);
    } while (magnitude > 1.0); // Reject points outside the unit sphere

    // Create and return the generated point
    Eigen::Vector3d p(x, y, z);
    samples.push_back(p);
  }
}

void visualize(VisibilityModel model) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (size_t i = 0; i < samples.size(); i++) {
    Eigen::Vector3d sample = samples[i];
    double visib = model == VisibilityModel::Proposed ? visibility[i] : visibility_conical[i];

    pcl::PointXYZRGBA p;
    p.x = sample.x();
    p.y = sample.y();
    p.z = sample.z();
    // p.r = visib * 255;
    // p.g = 175;
    // p.b = (1 - visib) * 255;
    p.r = 50 + visib * 205;
    p.g = 185;
    p.b = 250;
    p.a = visib * 255;
    pointcloud->points.push_back(p);
  }

  pointcloud->header.frame_id = "world";
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  visib_pub.publish(pointcloud_msg);
}

void visualize_fov() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01; // Line width
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  double x = cos(fov_h / 2.0);
  double y = sin(fov_h / 2.0);
  double z = sin(fov_v / 2.0);
  double norm = sqrt(x * x + y * y + z * z);

  geometry_msgs::Point o, p1, p2, p3, p4;
  vector<geometry_msgs::Point> p;
  vector<double> y_sign = {1, 1, -1, -1};
  vector<double> z_sign = {1, -1, -1, 1};

  o.x = 0.0;
  o.y = 0.0;
  o.z = 0.0;

  for (int i = 0; i < 4; i++) {
    geometry_msgs::Point pi;
    pi.x = x / norm;
    pi.y = y_sign[i] * y / norm;
    pi.z = z_sign[i] * z / norm;
    p.push_back(pi);
  }

  for (int i = 0; i < 4; i++) {
    marker.points.push_back(p[i]);
    marker.points.push_back(p[(i + 1) % 4]);
    marker.points.push_back(p[i]);
    marker.points.push_back(o);
  }

  fov_pub.publish(marker);
}

void evaluateVisibility() {
  // Proposed visibility model
  Eigen::Vector3d n1(0, 0, 1);
  Eigen::Vector3d n2(1, 0, 0);
  Eigen::Vector3d n3(0, 1, 0);

  for (Eigen::Vector3d p : samples) {
    double sin_theta1 = p.cross(n1).norm() / p.norm();
    double cos_theta2 = p.dot(n2) / p.norm();
    double sin_theta3 = p.cross(n3).norm() / p.norm();
    double visib = v1(sin_theta1) * v2(cos_theta2) * v3(sin_theta3);
    visibility.push_back(visib);
  }

  // Conical visibility model
  Eigen::Vector3d n(1, 0, 0);
  for (Eigen::Vector3d p : samples) {
    double cos_theta = p.dot(n) / p.norm();
    double visib = v_conical(cos_theta);
    visibility_conical.push_back(visib);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visibility_demo");
  ros::NodeHandle nh;
  visib_pub = nh.advertise<sensor_msgs::PointCloud2>("visibility", 1);
  fov_pub = nh.advertise<visualization_msgs::Marker>("fov", 1);

  generateSamples();
  evaluateVisibility();

  // Specify which visibility model to visualize
  VisibilityModel visulize_model = Proposed;

  ros::Rate loop_rate(10.0);
  while (ros::ok()) {
    visualize(visulize_model);
    visualize_fov();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}