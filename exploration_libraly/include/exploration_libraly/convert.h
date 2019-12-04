#ifndef CONVERT_H
#define CONVERT_H

#include <memory>
#include <Eigen/Geometry>

// 前方宣言
/// ros
namespace tf{
    class Quaternion;
}
/// rosmsgs
namespace geometry_msgs{
    template <class ContainerAllocator>
    struct Point_;
    typedef ::geometry_msgs::Point_<std::allocator<void>> Point;
    template <class ContainerAllocator>
    struct PointStamped_;
    typedef ::geometry_msgs::PointStamped_<std::allocator<void>> PointStamped;
    template <class ContainerAllocator>
    struct Pose_;
    typedef ::geometry_msgs::Pose_<std::allocator<void>> Pose;
    template <class ContainerAllocator>
    struct PoseStamped_;
    typedef ::geometry_msgs::PoseStamped_<std::allocator<void>> PoseStamped;
    template <class ContainerAllocator>
    struct Quaternion_;
    typedef ::geometry_msgs::Quaternion_<std::allocator<void>> Quaternion;
    template <class ContainerAllocator>
    struct Twist_;
    typedef ::geometry_msgs::Twist_<std::allocator<void>> Twist; 
    template <class ContainerAllocator>
    struct Vector3_;
    typedef ::geometry_msgs::Vector3_<std::allocator<void>> Vector3;
}
/// others
namespace pcl{
    struct PointXYZ;
}
// 前方宣言ここまで

// 型変換用
namespace ExpLib{
    namespace Convert{
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