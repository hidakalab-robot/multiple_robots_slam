#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <tf/transform_listener.h>
#include <exploration_libraly/convert.hpp>
#include <exploration_libraly/constructor.hpp>

namespace ExpLib
{
// destFrame から見た origFrame の座標を取得する // origFrame の座標を destFrame での座標に変換する
template <typename T> T coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p);

template<> void coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p){
    tf::StampedTransform transform;
    l.lookupTransform(destFrame, origFrame, ros::Time(0), transform);

    tf::Quaternion transQ = transform.getRotation();
    double transYaw = qToYaw(transQ);
    double transX = transform.getOrigin().getX();
    double transY = transform.getOrigin().getY();

    // ROS_INFO_STREAM("trans : " << transX << ", " << transY << ", " << transYaw);

    Eigen::Matrix2d rotation;
    rotation << cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw);

    Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.position.x, p.position.y) + Eigen::Vector2d(transX, transY));
    p.position = msgPoint(tempPoint.x(),tempPoint.y());
    p.orientation = tfQuaToGeoQua(tf::Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)*=transQ);
}

template<> geometry_msgs::Pose coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p){
    coordinateConverter<void>(l,destFrame,origFrame,p);
    return p;
}

template <typename T> T coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p);
template <typename T> T coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p);

template<> void coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p){
    geometry_msgs::Pose ps = pointToPose(p);
    coordinateConverter<void>(l,destFrame,origFrame,ps);
    p = ps.position;
}

template<> geometry_msgs::Point coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p){
    // ROS_INFO_STREAM("input : " << p.x << ", " << p.y);
    geometry_msgs::Point pl = p;
    coordinateConverter<void>(l,destFrame,origFrame,pl);
    // ROS_INFO_STREAM("output : " << pl.x << ", " << pl.y);
    return pl;
}

}

#endif // UTILITY_HPP