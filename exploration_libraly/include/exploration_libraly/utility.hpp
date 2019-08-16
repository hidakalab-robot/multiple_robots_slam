#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <exploration_libraly/construct.hpp>
#include <exploration_libraly/convert.hpp>
#include <tf/transform_listener.h>

namespace ExpLib
{
namespace Utility
{
// destFrame から見た origFrame の座標を取得する // origFrame の座標を destFrame での座標に変換する
template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p);

template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p){
    tf::StampedTransform transform;
    l.lookupTransform(destFrame, origFrame, ros::Time(0), transform);

    tf::Quaternion transQ = transform.getRotation();
    double transYaw = Convert::qToYaw(transQ);
    double transX = transform.getOrigin().getX();
    double transY = transform.getOrigin().getY();

    Eigen::Matrix2d rotation;
    rotation << cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw);

    Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.position.x, p.position.y) + Eigen::Vector2d(transX, transY));
    p.position = Construct::msgPoint(tempPoint.x(),tempPoint.y());
    p.orientation = Convert::tfQuaToGeoQua(tf::Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)*=transQ);
}

template<> geometry_msgs::Pose coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p){
    coordinateConverter2d<void>(l,destFrame,origFrame,p);
    return p;
}

template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p);
template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p);

template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p){
    geometry_msgs::Pose ps = Convert::pointToPose(p);
    coordinateConverter2d<void>(l,destFrame,origFrame,ps);
    p = ps.position;
}

template<> geometry_msgs::Point coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p){
    geometry_msgs::Point pl = p;
    coordinateConverter2d<void>(l,destFrame,origFrame,pl);
    return pl;
}

template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, pcl::PointXYZ& p);

template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, pcl::PointXYZ& p){
    geometry_msgs::Pose ps = Convert::pointToPose(Convert::pclPointXYZToPoint(p));
    coordinateConverter2d<void>(l,destFrame,origFrame,ps);
    p = Convert::pointToPclPointXYZ(ps.position);
}

}
}
#endif // UTILITY_HPP