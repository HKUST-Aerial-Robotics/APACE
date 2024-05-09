#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <string>

using namespace std;

// Global variables
double fov_h = 90 * M_PI / 180;
vector<Eigen::Vector3d> bs;
vector<double> gammas, betas;
vector<Eigen::Vector3d> changed_bs;

// v3 visibility function
double v3(double sin_theta3) {
  double sin_thr = sin(M_PI / 2.0 - fov_h / 2.0);
  double v3 = 1.0 / (1.0 + exp(-20 * (sin_theta3 - sin_thr)));
  return v3;
}

// Generate samples for evaluation
void generateSamples() {
  // Sample b using Fibonicci sphere
  double phi = M_PI * (3 - sqrt(5));
  double x, y, z, radius, theta;
  int sample_density = 50;
  for (int i = 0; i < sample_density; i++) {
    y = 1 - 2 * ((double)i / (double)(sample_density - 1));
    radius = sqrt(1 - y * y);
    theta = phi * i;
    x = cos(theta) * radius;
    z = sin(theta) * radius;

    Eigen::Vector3d sample;
    sample << x, y, z;
    bs.push_back(sample);
  }

  // Sample gamma and beta [-angle_max, angle_max]
  double angle_max = 14.3 * M_PI / 180;
  int sample_density2 = 20;
  for (int i = 0; i < sample_density2; i++) {
    double angle = -angle_max + 2 * angle_max * ((double)i / (double)(sample_density2 - 1));
    gammas.push_back(angle);
    betas.push_back(angle);
  }
}

// Main function
void visibility_dv3_droll() {
  generateSamples();

  int count_total = 0;
  int count_change = 0;
  for (Eigen::Vector3d b : bs) {
    for (double gamma : gammas) {
      // Before rotation
      double sin_theta3 = sqrt(1 - b(1) * b(1));
      // After rotation beta around axis n2
      for (double beta : betas) {
        double m = 1.0 / sqrt(pow(cos(gamma), 2) * pow(cos(beta), 2) + pow(sin(beta), 2));
        double cos_theta3_prime = b(1) * m * cos(gamma) * cos(beta) + b(2) * m * sin(beta);
        double sin_theta3_prime = sqrt(1 - pow(cos_theta3_prime, 2));
        if (fabs(v3(sin_theta3) - v3(sin_theta3_prime)) > 0.5) {
          count_change++;
        }
        count_total++;
      }
    }
  }

  double change_ratio = (double)count_change / (double)count_total;
  cout << "count_total: " << count_total << ", count_change: " << count_change << endl;
  cout << "change_ratio: " << change_ratio << endl;

  // Print all b in changed_bs
  for (Eigen::Vector3d b : changed_bs) {
    cout << b.transpose() << endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visibility_dv3_droll");
  ros::NodeHandle nh;

  visibility_dv3_droll();

  ros::spin();
  return 0;
}