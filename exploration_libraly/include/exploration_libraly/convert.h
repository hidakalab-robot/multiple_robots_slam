#ifndef CONVERT_H
#define CONVERT_H

#include <geometry_msgs/Quaternion.h>
#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

// 型変換用
namespace ExpLib
{
namespace Convert
{
double qToYaw(const tf::Quaternion &q);

double qToYaw(const geometry_msgs::Quaternion &q);
double qToYaw(const Eigen::Quaterniond &q);

geometry_msgs::Quaternion yawToQ(double yaw);

Eigen::Vector2d qToVector2d(const geometry_msgs::Quaternion &q);

geometry_msgs::Quaternion vector2dToQ(const Eigen::Vector2d &v);

geometry_msgs::Pose pointToPose(const geometry_msgs::Point &point);

Eigen::Vector3d pointToVector3d(const geometry_msgs::Point &point);

Eigen::Vector2d pointToVector2d(const geometry_msgs::Point &point);

geometry_msgs::Point vector2dToPoint(const Eigen::Vector2d &vec);

Eigen::Vector2d msgVectorToVector2d(const geometry_msgs::Vector3 &vector);

geometry_msgs::Point pclPointXYZToPoint(const pcl::PointXYZ &pcl);

pcl::PointXYZ pointToPclPointXYZ(const geometry_msgs::Point &point);

geometry_msgs::PoseStamped pointToPoseStamped(const geometry_msgs::Point &p, const std::string &f);

geometry_msgs::PointStamped pointToPointStamped(const geometry_msgs::Point &p, const std::string &f);

geometry_msgs::PointStamped poseStampedToPointStamped(const geometry_msgs::PoseStamped &p);

geometry_msgs::PoseStamped poseToPoseStamped(const geometry_msgs::Pose &p, const std::string &f);

geometry_msgs::PoseStamped pointStampedToPoseStamped(const geometry_msgs::PointStamped &p);

geometry_msgs::Quaternion tfQuaToGeoQua(const tf::Quaternion &tq);

tf::Quaternion geoQuaToTfQua(const geometry_msgs::Quaternion &gq);
geometry_msgs::Quaternion eigenQuaToGeoQua(const Eigen::Quaterniond &eq);

Eigen::Quaterniond geoQuaToEigenQua(const geometry_msgs::Quaternion &gq);

} // namespace Convert
} // namespace ExpLib

#endif // CONVERT_H