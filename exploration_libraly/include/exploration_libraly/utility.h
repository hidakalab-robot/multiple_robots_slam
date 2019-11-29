#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <exploration_libraly/construct.h>
#include <exploration_libraly/convert.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ExpLib
{
namespace Utility
{
// destFrame から見た origFrame の座標を取得する // origFrame の座標を destFrame での座標に変換する
template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p);

template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p);


template<> geometry_msgs::Pose coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p);

template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p);
template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p);

template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p);

template<> geometry_msgs::Point coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p);

template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, pcl::PointXYZ& p);

template<> void coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, pcl::PointXYZ& p);

geometry_msgs::Point mapIndexToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info);

Eigen::Vector2i coordinateToMapIndex(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info);

Eigen::Vector2i coordinateToMapIndex(const geometry_msgs::Point& coordinate,const nav_msgs::MapMetaData& info);

std::vector<std::vector<int8_t>> mapArray1dTo2d(const std::vector<int8_t>& data, const nav_msgs::MapMetaData& info);

double shorterRotationAngle(const double orig, const double dest);

double shorterRotationAngle(const geometry_msgs::Quaternion& orig, const geometry_msgs::Quaternion& dest);

}
}
#endif // UTILITY_HPP