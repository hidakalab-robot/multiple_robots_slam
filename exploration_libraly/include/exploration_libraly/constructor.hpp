#ifndef CONSTRUCTOR_HPP
#define CONSTRUCTOR_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <exploration_msgs/Frontier.h>
#include <pcl_ros/point_cloud.h>
#include <exploration_msgs/RobotInfo.h>

namespace ExpLib
{
geometry_msgs::Point msgPoint(double x=0,double y=0,double z=0){
    geometry_msgs::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    return msg;
}

geometry_msgs::Pose msgPose(const geometry_msgs::Point& p, const geometry_msgs::Quaternion& q){
    geometry_msgs::Pose msg;
    msg.position = p;
    msg.orientation = q;
    return msg;
}

geometry_msgs::Vector3 msgVector(double x=0,double y=0,double z=0){
    geometry_msgs::Vector3 msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    return msg;
}

geometry_msgs::Twist msgTwist(double x=0,double z=0){
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.angular.z = z;
    return msg;
}

std_msgs::Bool msgBool(bool b=true){
    std_msgs::Bool msg;
    msg.data = b;
    return msg;
}

std_msgs::Float64 msgDouble(double d){
    std_msgs::Float64 msg;
    msg.data = d;
    return msg;
}

std_msgs::Int32 msgInt(int i){
    std_msgs::Int32 msg;
    msg.data = i;
    return msg;
}

exploration_msgs::Frontier msgFrontier(const geometry_msgs::Point& c, double a, const geometry_msgs::Vector3& v, double cv){
    exploration_msgs::Frontier msg;
    msg.coordinate = c;
    msg.area = a;
    msg.variance = v;
    msg.covariance = cv;
    return msg;
}

pcl::PointXYZRGB pclXYZRGB(float x,float y,float z,float r,float g,float b){
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = r;
    p.g = g;
    p.b = b;
    return p;
}

exploration_msgs::RobotInfo msgRobotInfo(const std::string& n, const geometry_msgs::Pose& p){
    exploration_msgs::RobotInfo msg;
    msg.name = n;
    msg.pose = p;
    return msg;
}

// exploration_msgs::RobotInfo msgRobotInfo(const std::string& n, const geometry_msgs::PoseStamped& pe, const geometry_msgs::Point& pt, const geometry_msgs::Vector3& v){
//     exploration_msgs::RobotInfo msg;
//     msg.name = n;
//     msg.pose = pe;
//     msg.coordinate = pt;
//     msg.vector = v;
//     return msg;
// }

// exploration_msgs::RobotInfo msgRobotInfo(const std::string& n, const geometry_msgs::Point& pt, const geometry_msgs::Vector3& v){
//     exploration_msgs::RobotInfo msg;
//     msg.name = n;
//     geometry_msgs::PoseStamped ps;
//     msg.pose = ps;
//     msg.coordinate = pt;
//     msg.vector = v;
//     return msg;
// }

geometry_msgs::Quaternion msgGeoQuaternion(double x, double y, double z, double w){
    geometry_msgs::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

}

#endif // CONSTRUCTOR_HPP