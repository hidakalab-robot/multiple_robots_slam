#ifndef COMMON_LIB_HPP
#define COMMON_LIB_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_msgs/Frontier.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <Eigen/Core>
#include <exploration_msgs/RobotInfo.h>
#include <tf/transform_listener.h>

namespace CommonLib
{
enum class DuplicationStatus{
	NOT_DUPLECATION,
	OLDER,
	NEWER
};

template <typename T>
struct subStruct{
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::CallbackQueue q;
    T data;
    subStruct(const std::string& topic,uint32_t queue_size){
        n.setCallbackQueue(&q);
        sub = n.subscribe<T>(topic, queue_size, [this](const boost::shared_ptr<const T>& msg) {data = *msg;});//データをコピーするコールバック関数を自動生成
    };

    template <class U,typename V>
    subStruct(const std::string& topic,uint32_t queue_size, void(U::*fp)(V), U* obj){
        n.setCallbackQueue(&q);
        sub = n.subscribe(topic,queue_size,fp,obj);
    };
};

struct subStructSimple{
    ros::NodeHandle n;
    ros::Subscriber sub;
    template <class U,typename V>
    subStructSimple(const std::string& topic,uint32_t queue_size, void(U::*fp)(V), U *obj){ sub = n.subscribe(topic,queue_size,fp,obj);};

    template<typename V>
    subStructSimple(const std::string& topic,uint32_t queue_size, void(*fp)(V)){ sub = n.subscribe(topic,queue_size,fp);};
};

template<typename T>
struct pubStruct{
    ros::NodeHandle n;
    ros::Publisher pub;
    pubStruct(const std::string& topic,uint32_t queue_size,bool latch=false){ pub = n.advertise<T>(topic, queue_size, latch);};
};

struct scanStruct{
    std::vector<float> ranges;
    std::vector<float> angles;
    float angleMax;
    scanStruct(int size,float angle):angleMax(angle){
        ranges.reserve(size);
        angles.reserve(size);
    };
};

struct listStruct{
    geometry_msgs::Point point;
    CommonLib::DuplicationStatus duplication;
    listStruct(){};
    listStruct(const geometry_msgs::Point& p):point(p){};
};

double qToYaw(const tf::Quaternion& q){
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    return yaw;
}

double qToYaw(const geometry_msgs::Quaternion& q){
    // tf::Quaternion tq(q.x, q.y, q.z, q.w);
    // return qToYaw(tq);
    return qToYaw(tf::Quaternion(q.x, q.y, q.z, q.w));
}

double qToYaw(const Eigen::Quaterniond& q){
    // tf::Quaternion tq(q.x, q.y, q.z, q.w);
    // return qToYaw(tq);
    return qToYaw(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
}

geometry_msgs::Quaternion yawToQ(double yaw){
    geometry_msgs::Quaternion msg;
    // tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,yaw);  
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,0,yaw), msg);
    return msg;
}

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

geometry_msgs::Pose pointToPose(const geometry_msgs::Point& point){
    geometry_msgs::Pose msg;
    msg.position.x = point.x;
    msg.position.y = point.y;
    msg.position.z = point.z;
    msg.orientation.w = 1.0;
    return msg;
}

Eigen::Vector3d pointToVector3d(const geometry_msgs::Point& point){
    Eigen::Vector3d vec;
    vec.x() = point.x;
    vec.y() = point.y;
    vec.z() = point.z;
    return vec;
}

Eigen::Vector2d pointToVector2d(const geometry_msgs::Point& point){
    Eigen::Vector2d vec;
    vec.x() = point.x;
    vec.y() = point.y;
    return vec;
}

Eigen::Vector2d msgVectorToVector2d(const geometry_msgs::Vector3& vector){
    Eigen::Vector2d vec;
    vec.x() = vector.x;
    vec.y() = vector.y;
    return vec;
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

exploration_msgs::Frontier msgFrontier(const geometry_msgs::Point& c, double a, const geometry_msgs::Vector3& v, double cv){
    exploration_msgs::Frontier msg;
    msg.coordinate = c;
    msg.area = a;
    msg.variance = v;
    msg.covariance = cv;
    return msg;
}

geometry_msgs::PoseStamped pointToPoseStamped(const geometry_msgs::Point& p, const std::string& f){
    geometry_msgs::PoseStamped msg;
    msg.pose = pointToPose(p);
    msg.header.frame_id = f;
    return msg;
}

geometry_msgs::PoseStamped poseToPoseStamped(const geometry_msgs::Pose& p, const std::string& f){
    geometry_msgs::PoseStamped msg;
    msg.pose = p;
    msg.header.frame_id = f;
    return msg;
}

exploration_msgs::RobotInfo msgRobotInfo(const std::string& n, const geometry_msgs::PoseStamped& pe, const geometry_msgs::Point& pt, const geometry_msgs::Vector3& v){
    exploration_msgs::RobotInfo msg;
    msg.name = n;
    msg.pose = pe;
    msg.coordinate = pt;
    msg.vector = v;
    return msg;
}

exploration_msgs::RobotInfo msgRobotInfo(const std::string& n, const geometry_msgs::Point& pt, const geometry_msgs::Vector3& v){
    exploration_msgs::RobotInfo msg;
    msg.name = n;
    geometry_msgs::PoseStamped ps;
    msg.pose = ps;
    msg.coordinate = pt;
    msg.vector = v;
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

geometry_msgs::Quaternion tfQuaToGeoQua(const tf::Quaternion& tq){
    geometry_msgs::Quaternion q;
    q.x = tq.getX();
    q.y = tq.getY();
    q.z = tq.getZ();
    q.w = tq.getW();
    return q;
}

// destFrame から見た origFrame の座標を取得する // origFrame の座標を destFrame での座標に変換する
template <typename T> T coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p, geometry_msgs::Quaternion& q);

template<> void coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p, geometry_msgs::Quaternion& q){
    tf::StampedTransform transform;
    l.lookupTransform(destFrame, origFrame, ros::Time(0), transform);

    tf::Quaternion transQ = transform.getRotation();
    double transYaw = CommonLib::qToYaw(transQ);
    double transX = transform.getOrigin().getX();
    double transY = transform.getOrigin().getY();

    Eigen::Matrix2d rotation;
    rotation << cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw);

    Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.x - transX, p.y - transY));
    p = msgPoint(tempPoint.x(),tempPoint.y());
    q = tfQuaToGeoQua(tf::Quaternion(q.x,q.y,q.z,q.w)*=transQ);
}

template<> geometry_msgs::Pose coordinateConverter(const tf::TransformListener& l, const std::string& destFrame, const std::string& origFrame, geometry_msgs::Point& p, geometry_msgs::Quaternion& q){
    tf::StampedTransform transform;
    l.lookupTransform(destFrame, origFrame, ros::Time(0), transform);

    tf::Quaternion transQ = transform.getRotation();
    double transYaw = CommonLib::qToYaw(transQ);
    double transX = transform.getOrigin().getX();
    double transY = transform.getOrigin().getY();

    Eigen::Matrix2d rotation;
    rotation << cos(transYaw),-sin(transYaw),sin(transYaw),cos(transYaw);

    Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.x - transX, p.y - transY));
    p = msgPoint(tempPoint.x(),tempPoint.y());
    q = tfQuaToGeoQua(tf::Quaternion(q.x,q.y,q.z,q.w)*=transQ);

    return msgPose(p,q);
}

template <typename T> T coordinateConverter(const tf::TransformListener& l, const std::string& target, const std::string& source, geometry_msgs::Point& p);

template<> void coordinateConverter(const tf::TransformListener& l, const std::string& target, const std::string& source, geometry_msgs::Point& p){
    geometry_msgs::Quaternion q;
    coordinateConverter<void>(l,target,source,p,q);
}

template<> geometry_msgs::Point coordinateConverter(const tf::TransformListener& l, const std::string& target, const std::string& source, geometry_msgs::Point& p){
    geometry_msgs::Quaternion q;
    coordinateConverter<void>(l,target,source,p,q);
    return p;
}

}

#endif //COMMON_LIB_HPP