#include <exploration_libraly/construct.h>
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

namespace ExpLib{
    namespace Construct{
        geometry_msgs::Point msgPoint(double x,double y,double z){
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

        geometry_msgs::Vector3 msgVector(double x,double y,double z){
            geometry_msgs::Vector3 msg;
            msg.x = x;
            msg.y = y;
            msg.z = z;
            return msg;
        }

        geometry_msgs::Twist msgTwist(double x,double z){
            geometry_msgs::Twist msg;
            msg.linear.x = x;
            msg.angular.z = z;
            return msg;
        }

        std_msgs::Bool msgBool(bool b){
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

        std_msgs::Int8 msgInt8(int i){
            std_msgs::Int8 msg;
            msg.data = i;
            return msg;
        }

        exploration_msgs::Frontier msgFrontier(const geometry_msgs::Point& p, double a, const geometry_msgs::Vector3& v, double c){
            exploration_msgs::Frontier msg;
            msg.point = p;
            msg.area = a;
            msg.variance = v;
            msg.covariance = c;
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

        geometry_msgs::Quaternion msgGeoQuaternion(double x, double y, double z, double w){
            geometry_msgs::Quaternion q;
            q.x = x;
            q.y = y;
            q.z = z;
            q.w = w;
            return q;
        }

        visualization_msgs::Marker msgCubeListMarker(const std::string& frame_id, const double scale, const float r, const float g, const float b, const float a){
            visualization_msgs::Marker m;
            m.header.frame_id = frame_id;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = scale;
            m.type = visualization_msgs::Marker::CUBE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            m.lifetime = ros::Duration(0);
            m.color.r = r;
            m.color.g = g;
            m.color.b = b;
            m.color.a = a;
            return m;
        }
    }
}