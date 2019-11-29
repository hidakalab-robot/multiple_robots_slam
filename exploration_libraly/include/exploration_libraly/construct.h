#ifndef CONSTRUCT_HPP
#define CONSTRUCT_HPP

#include <exploration_msgs/Frontier.h>
#include <exploration_msgs/RobotInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>

namespace ExpLib
{
namespace Construct
{
geometry_msgs::Point msgPoint(double x=0,double y=0,double z=0);

geometry_msgs::Pose msgPose(const geometry_msgs::Point& p, const geometry_msgs::Quaternion& q);

geometry_msgs::Vector3 msgVector(double x=0,double y=0,double z=0);

geometry_msgs::Twist msgTwist(double x=0,double z=0);

std_msgs::Bool msgBool(bool b=true);

std_msgs::Float64 msgDouble(double d);

std_msgs::Int32 msgInt(int i);

std_msgs::Int8 msgInt8(int i);

exploration_msgs::Frontier msgFrontier(const geometry_msgs::Point& p, double a, const geometry_msgs::Vector3& v, double c);

pcl::PointXYZRGB pclXYZRGB(float x,float y,float z,float r,float g,float b);

exploration_msgs::RobotInfo msgRobotInfo(const std::string& n, const geometry_msgs::Pose& p);

geometry_msgs::Quaternion msgGeoQuaternion(double x, double y, double z, double w);

template <typename T>
std::vector<T> oneFactorVector(const T& factor);

visualization_msgs::Marker msgCubeListMarker(const std::string& frame_id, const double scale=0.5, const float r=1.0, const float g=0.0, const float b=0.0, const float a=1.0);

}
}
#endif // CONSTRUCT_HPP
