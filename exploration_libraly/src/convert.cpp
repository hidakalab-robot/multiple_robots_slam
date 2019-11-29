#ifndef CONVERT_HPP
#define CONVERT_HPP

#include <exploration_libraly/convert.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

// 型変換用
namespace ExpLib
{
namespace Convert
{
double qToYaw(const tf::Quaternion &q){
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double qToYaw(const geometry_msgs::Quaternion &q){
    return qToYaw(tf::Quaternion(q.x, q.y, q.z, q.w));
}

double qToYaw(const Eigen::Quaterniond &q){
    return qToYaw(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
}

geometry_msgs::Quaternion yawToQ(double yaw){
    geometry_msgs::Quaternion msg;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, yaw), msg);
    return msg;
}

Eigen::Vector2d qToVector2d(const geometry_msgs::Quaternion &q){
    double yaw = qToYaw(q);
    return Eigen::Vector2d(cos(yaw), sin(yaw));
}

geometry_msgs::Quaternion vector2dToQ(const Eigen::Vector2d &v){
    double yaw = atan2(v.y(), v.x());
    return yawToQ(yaw);
}

geometry_msgs::Pose pointToPose(const geometry_msgs::Point &point){
    geometry_msgs::Pose msg;
    msg.position.x = point.x;
    msg.position.y = point.y;
    msg.position.z = point.z;
    msg.orientation.w = 1.0;
    return msg;
}

Eigen::Vector3d pointToVector3d(const geometry_msgs::Point &point){
    Eigen::Vector3d vec;
    vec.x() = point.x;
    vec.y() = point.y;
    vec.z() = point.z;
    return vec;
}

Eigen::Vector2d pointToVector2d(const geometry_msgs::Point &point){
    Eigen::Vector2d vec;
    vec.x() = point.x;
    vec.y() = point.y;
    return vec;
}

geometry_msgs::Point vector2dToPoint(const Eigen::Vector2d &vec){
    geometry_msgs::Point point;
    point.x = vec.x();
    point.y = vec.y();
    return point;
}

Eigen::Vector2d msgVectorToVector2d(const geometry_msgs::Vector3 &vector){
    Eigen::Vector2d vec;
    vec.x() = vector.x;
    vec.y() = vector.y;
    return vec;
}

geometry_msgs::Point pclPointXYZToPoint(const pcl::PointXYZ &pcl){
    geometry_msgs::Point point;
    point.x = pcl.x;
    point.y = pcl.y;
    point.z = pcl.z;
    return point;
}

pcl::PointXYZ pointToPclPointXYZ(const geometry_msgs::Point &point){
    pcl::PointXYZ pcl;
    pcl.x = point.x;
    pcl.y = point.y;
    pcl.z = point.z;
    return pcl;
}

geometry_msgs::PoseStamped pointToPoseStamped(const geometry_msgs::Point &p, const std::string &f){
    geometry_msgs::PoseStamped msg;
    msg.pose = pointToPose(p);
    msg.header.frame_id = f;
    return msg;
}

geometry_msgs::PointStamped pointToPointStamped(const geometry_msgs::Point &p, const std::string &f){
    geometry_msgs::PointStamped msg;
    msg.point = p;
    msg.header.frame_id = f;
    return msg;
}

geometry_msgs::PointStamped poseStampedToPointStamped(const geometry_msgs::PoseStamped &p){
    geometry_msgs::PointStamped msg;
    msg.point = p.pose.position;
    msg.header.frame_id = p.header.frame_id;
    return msg;
}

geometry_msgs::PoseStamped poseToPoseStamped(const geometry_msgs::Pose &p, const std::string &f){
    geometry_msgs::PoseStamped msg;
    msg.pose = p;
    msg.header.frame_id = f;
    return msg;
}

geometry_msgs::PoseStamped pointStampedToPoseStamped(const geometry_msgs::PointStamped &p){
    geometry_msgs::PoseStamped msg;
    msg.pose = pointToPose(p.point);
    msg.header.frame_id = p.header.frame_id;
    return msg;
}

geometry_msgs::Quaternion tfQuaToGeoQua(const tf::Quaternion &tq){
    geometry_msgs::Quaternion q;
    q.x = tq.getX();
    q.y = tq.getY();
    q.z = tq.getZ();
    q.w = tq.getW();
    return q;
}

tf::Quaternion geoQuaToTfQua(const geometry_msgs::Quaternion &gq){
    return tf::Quaternion(gq.x,gq.y,gq.z,gq.w);
}

geometry_msgs::Quaternion eigenQuaToGeoQua(const Eigen::Quaterniond &eq){
    geometry_msgs::Quaternion q;
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
    q.w = eq.w();
    return q;
}

Eigen::Quaterniond geoQuaToEigenQua(const geometry_msgs::Quaternion &gq){
    Eigen::Quaterniond q;
    q.x() = gq.x;
    q.y() = gq.y;
    q.z() = gq.z;
    q.w() = gq.w;
    return q;
}

} // namespace Convert
} // namespace ExpLib

#endif // CONVERT_HPP