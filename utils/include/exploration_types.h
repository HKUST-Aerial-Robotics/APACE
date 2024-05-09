#ifndef TYPES_H
#define TYPES_H

#include <kindr/minimal/position.h>
#include <kindr/minimal/quat-transformation.h>

#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <list>
#include <queue>
#include <stack>
#include <vector>

typedef double FloatingPoint;

typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint> Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<FloatingPoint> Rotation;
typedef kindr::minimal::Position Position;
typedef kindr::minimal::RotationQuaternionTemplate<FloatingPoint>::Implementation Quaternion;

typedef geometry_msgs::Transform TransformMsgType;
typedef geometry_msgs::TransformStamped TransformStampedMsgType;
typedef geometry_msgs::Pose PoseMsgType;
typedef geometry_msgs::PoseStamped PoseStampedMsgType;
typedef nav_msgs::Odometry OdometryMsgType;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudType;
typedef Eigen::Vector3i VoxelIndex;
typedef int VoxelAddress;
typedef int FrameIndex;
typedef int ClusterIndex;

template <typename Type> using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
template <typename Type> using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;
template <typename Type> using AlignedQueue = std::queue<Type, AlignedDeque<Type>>;
template <typename Type> using AlignedStack = std::stack<Type, AlignedDeque<Type>>;
template <typename Type> using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

typedef Eigen::Matrix<double, 6, 60> Matrix6x60;
typedef Eigen::Matrix<double, 60, 6> Matrix60x6;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

#define ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(SIZE, SUFFIX)                            \
  using Matrix##SUFFIX = Eigen::Matrix<double, SIZE, SIZE>;                    \
  using Matrix1##SUFFIX = Eigen::Matrix<double, 1, SIZE>;                      \
  using Matrix2##SUFFIX = Eigen::Matrix<double, 2, SIZE>;                      \
  using Matrix3##SUFFIX = Eigen::Matrix<double, 3, SIZE>;                      \
  using Matrix4##SUFFIX = Eigen::Matrix<double, 4, SIZE>;                      \
  using Matrix5##SUFFIX = Eigen::Matrix<double, 5, SIZE>;                      \
  using Matrix6##SUFFIX = Eigen::Matrix<double, 6, SIZE>;                      \
  using Matrix7##SUFFIX = Eigen::Matrix<double, 7, SIZE>;                      \
  using Matrix8##SUFFIX = Eigen::Matrix<double, 8, SIZE>;                      \
  using Matrix9##SUFFIX = Eigen::Matrix<double, 9, SIZE>;                      \
  using Matrix##SUFFIX##X = Eigen::Matrix<double, SIZE, Eigen::Dynamic>;       \
  using MatrixX##SUFFIX = Eigen::Matrix<double, Eigen::Dynamic, SIZE>;         \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::IdentityReturnType           \
      I_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Identity();                      \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::ConstantReturnType           \
      Z_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Zero()

ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(1, 1);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(2, 2);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(3, 3);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(4, 4);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(5, 5);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(6, 6);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(7, 7);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(8, 8);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(9, 9);

#undef ZE_MAKE_EIGEN_MATRIX_TYPEDEFS

// Typedef arbitary length vector and arbitrary sized matrix.
using VectorX = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using MatrixX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXi = Eigen::VectorXi;

// Commonly used fixed size vectors.
using Vector1d = Eigen::Matrix<double, 1, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector10d = Eigen::Matrix<double, 10, 1>;

// Commonly used fixed size matrices.
// using Matrix1d = Eigen::Matrix<double, 1, 1>;
// using Matrix2d = Eigen::Matrix<double, 2, 2>;
// using Matrix3d = Eigen::Matrix<double, 3, 3>;
// using Matrix4d = Eigen::Matrix<double, 4, 4>;
// using Matrix5d = Eigen::Matrix<double, 5, 5>;
// using Matrix6d = Eigen::Matrix<double, 6, 6>;
// using Matrix7d = Eigen::Matrix<double, 7, 7>;
// using Matrix8d = Eigen::Matrix<double, 8, 8>;
// using Matrix9d = Eigen::Matrix<double, 9, 9>;

#endif