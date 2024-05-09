#include "transformer/transformer.h"

Transformer::Transformer(ros::NodeHandle &nh) : world_frame_("world"), sensor_frame_("camera") {
  double timestamp_tolerance_duration;
  nh.param("transformer/world_frame", world_frame_, world_frame_);
  nh.param("transformer/sensor_frame", sensor_frame_, sensor_frame_);
  nh.param("transformer/timestamp_tolerance", timestamp_tolerance_duration, 1e-3);
  nh.param("transformer/verbose", verbose_, false);

  std::string pose_topic_type;
  nh.param("transformer/pose_topic_type", pose_topic_type, std::string(""));
  if (pose_topic_type == "pose") {
    config_.pose_topic_type_ = Config::POSE_TYPE::POSE;
  } else if (pose_topic_type == "odometry") {
    config_.pose_topic_type_ = Config::POSE_TYPE::ODOM;
  } else if (pose_topic_type == "transform") {
    config_.pose_topic_type_ = Config::POSE_TYPE::TRANSFORM;
  } else {
    config_.pose_topic_type_ = Config::POSE_TYPE::UNKNOWN;
  }

  timestamp_tolerance_ = ros::Duration(timestamp_tolerance_duration);
  switch (config_.pose_topic_type_) {
  case Config::POSE_TYPE::POSE:
    transform_sub_ =
        nh.subscribe("/transformer/sensor_pose_topic", 100, &Transformer::poseCallback, this);
    break;
  case Config::POSE_TYPE::ODOM:
    transform_sub_ =
        nh.subscribe("/transformer/sensor_pose_topic", 100, &Transformer::odometryCallback, this);
    break;
  case Config::POSE_TYPE::TRANSFORM:
    transform_sub_ =
        nh.subscribe("/transformer/sensor_pose_topic", 100, &Transformer::transformCallback, this);
    break;
  case Config::POSE_TYPE::UNKNOWN:
    CHECK(false) << "Unknown pose topic type: " << pose_topic_type;
    break;
  default:
    break;
  }

  if (verbose_) {
    ROS_INFO("[Transformer] Initialized transformer with world frame: %s, sensor frame: %s",
             world_frame_.c_str(), sensor_frame_.c_str());
    ROS_INFO("[Transformer] Subscribe from transformer topic: %s",
             transform_sub_.getTopic().c_str());
  }
}

bool Transformer::lookupTransform(const ros::Time &timestamp, Transformation &transform) {
  if (transform_queue_.empty()) {
    if (verbose_)
      ROS_WARN_STREAM_THROTTLE(1, "[Transformer] No match found for transform timestamp: "
                                      << timestamp << " as transform queue is empty.");
    return false;
  }

  bool match_found = false;
  std::deque<TransformStampedMsgType>::reverse_iterator it = transform_queue_.rbegin();
  for (; it != transform_queue_.rend(); ++it) {
    if (it->header.stamp < timestamp) {
      if (timestamp - it->header.stamp < timestamp_tolerance_) {
        match_found = true;
      }
      break;
    }

    if (it->header.stamp - timestamp < timestamp_tolerance_) {
      match_found = true;
      break;
    }
  }

  if (match_found) {
    tf::transformMsgToKindr(it->transform, &transform);
  } else {
    if (it == transform_queue_.rbegin() || it == transform_queue_.rend()) {
      if (verbose_)
        ROS_WARN_STREAM("[Transformer] No match found for transform timestamp: "
                        << timestamp << " Queue front: " << transform_queue_.front().header.stamp
                        << " back: " << transform_queue_.back().header.stamp);
      return false;
    }
    // Interpolate bwtween the two closest transforms
    // Newest should be one past the requested timestamp
    // Oldest should be one before the requested timestamp.
    it--;
    Transformation T_newest;
    tf::transformMsgToKindr(it->transform, &T_newest);
    int64_t offset_newest_ns = (it->header.stamp - timestamp).toNSec();
    if (verbose_)
      ROS_INFO("[Transformer] Found T_newest with timestamp: %f", it->header.stamp.toSec());
    // We already checked that this is not the beginning.
    it++;
    Transformation T_oldest;
    tf::transformMsgToKindr(it->transform, &T_oldest);
    int64_t offset_oldest_ns = (timestamp - it->header.stamp).toNSec();
    if (verbose_)
      ROS_INFO("[Transformer] Found T_oldest with timestamp: %f", it->header.stamp.toSec());

    // Interpolate between the two transformations using the exponential map.
    FloatingPoint t_diff_ratio = static_cast<FloatingPoint>(offset_oldest_ns) /
                                 static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector = (T_oldest.inverse() * T_newest).log();
    transform = T_oldest * Transformation::exp(t_diff_ratio * diff_vector);

    if (verbose_)
      ROS_INFO("[Transformer] Interpolated T with timestamp: %f", timestamp.toSec());
  }

  return true;
}

void Transformer::transformCallback(const TransformStampedMsgType &msg) {
  if (verbose_)
    ROS_INFO_THROTTLE(1.0, "[Transformer] Received transform, transformation queue size: %d",
                      transform_queue_.size());

  transform_queue_.push_back(msg);
}

void Transformer::poseCallback(const PoseStampedMsgType &msg) {
  if (verbose_)
    ROS_INFO_THROTTLE(1.0, "[Transformer] Received transform, transformation queue size: %d",
                      transform_queue_.size());

  TransformStampedMsgType transform_msg;
  transform_msg.header = msg.header;
  transform_msg.transform.translation.x = msg.pose.position.x;
  transform_msg.transform.translation.y = msg.pose.position.y;
  transform_msg.transform.translation.z = msg.pose.position.z;
  transform_msg.transform.rotation.x = msg.pose.orientation.x;
  transform_msg.transform.rotation.y = msg.pose.orientation.y;
  transform_msg.transform.rotation.z = msg.pose.orientation.z;
  transform_msg.transform.rotation.w = msg.pose.orientation.w;
  transform_queue_.push_back(transform_msg);
}

void Transformer::odometryCallback(const OdometryMsgType &msg) {
  if (verbose_)
    ROS_INFO_THROTTLE(1.0, "[Transformer] Received transform, transformation queue size: %d",
                      transform_queue_.size());

  TransformStampedMsgType transform_msg;
  transform_msg.header = msg.header;
  transform_msg.transform.translation.x = msg.pose.pose.position.x;
  transform_msg.transform.translation.y = msg.pose.pose.position.y;
  transform_msg.transform.translation.z = msg.pose.pose.position.z;
  transform_msg.transform.rotation.x = msg.pose.pose.orientation.x;
  transform_msg.transform.rotation.y = msg.pose.pose.orientation.y;
  transform_msg.transform.rotation.z = msg.pose.pose.orientation.z;
  transform_msg.transform.rotation.w = msg.pose.pose.orientation.w;
  transform_queue_.push_back(transform_msg);
}
