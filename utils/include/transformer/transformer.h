#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <kindr/minimal/quat-transformation.h>
#include <kindr/minkindr_conversions/kindr_msg.h>
#include <kindr/minkindr_conversions/kindr_tf.h>
#include <kindr/minkindr_conversions/kindr_xml.h>
#include "exploration_types.h"

using std::shared_ptr;

class Transformer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<Transformer> Ptr;
  typedef shared_ptr<const Transformer> ConstPtr;

  struct Config {
    enum class POSE_TYPE { POSE, ODOM, TRANSFORM, UNKNOWN };
    POSE_TYPE pose_topic_type_;
  };

  Transformer(){};
  ~Transformer(){};

  Transformer(ros::NodeHandle &nh);

  // loop up transformation by timestamp
  bool lookupTransform(const ros::Time &timestamp, Transformation &transform);

  void transformCallback(const TransformStampedMsgType &msg);
  void poseCallback(const PoseStampedMsgType &msg);
  void odometryCallback(const OdometryMsgType &msg);

private:
  Config config_;
  
  std::string world_frame_;
  std::string sensor_frame_;
  ros::Duration timestamp_tolerance_;

  ros::Subscriber transform_sub_;
  AlignedDeque<TransformStampedMsgType> transform_queue_;

  bool verbose_;
};

#endif // TRANSFORMER_H