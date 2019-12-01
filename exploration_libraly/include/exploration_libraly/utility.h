#ifndef UTILITY_H
#define UTILITY_H

#include <memory>
#include <Eigen/Core>

// 前方宣言
namespace nav_msgs{
    template <class ContainerAllocator>
    struct MapMetaData_;
    typedef ::nav_msgs::MapMetaData_<std::allocator<void>> MapMetaData;
}
namespace geometry_msgs{
    template <class ContainerAllocator>
    struct Point_;
    typedef ::geometry_msgs::Point_<std::allocator<void>> Point;
    template <class ContainerAllocator>
    struct Quaternion_;
    typedef ::geometry_msgs::Quaternion_<std::allocator<void>> Quaternion;
    template <class ContainerAllocator>
    struct Pose_;
    typedef ::geometry_msgs::Pose_<std::allocator<void>> Pose;     
}
namespace tf{
    class TransformListener;
}
namespace pcl{
    struct PointXYZ;
}
// 前方宣言 ここまで

namespace ExpLib{
    namespace Utility{
        // destFrame から見た origFrame の座標を取得する // origFrame の座標を destFrame での座標に変換する
        template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Pose& p);
        template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p);
        template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, const geometry_msgs::Point& p);
        template <typename T> T coordinateConverter2d(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, pcl::PointXYZ& p);
        geometry_msgs::Point mapIndexToCoordinate(int indexX,int indexY,const nav_msgs::MapMetaData& info);
        Eigen::Vector2i coordinateToMapIndex(const Eigen::Vector2d& coordinate,const nav_msgs::MapMetaData& info);
        Eigen::Vector2i coordinateToMapIndex(const geometry_msgs::Point& coordinate,const nav_msgs::MapMetaData& info);
        std::vector<std::vector<int8_t>> mapArray1dTo2d(const std::vector<int8_t>& data, const nav_msgs::MapMetaData& info);
        double shorterRotationAngle(const double orig, const double dest);
        double shorterRotationAngle(const geometry_msgs::Quaternion& orig, const geometry_msgs::Quaternion& dest);
    }
}
#endif // UTILITY_H