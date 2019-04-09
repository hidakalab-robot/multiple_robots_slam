#ifndef COMMON_LIB_HPP
#define COMMON_LIB_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <exploration_msgs/Frontier.h>
#include <std_msgs/Bool.h>

namespace CommonLib
{
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

geometry_msgs::Point msgPoint(double x=0,double y=0,double z=0){
    geometry_msgs::Point msg;
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

std_msgs::Bool msgBool(bool d=true){
    std_msgs::Bool msg;
    msg.data = d;
    return msg;
}

geometry_msgs::Pose pointToPose(const geometry_msgs::Point& point){
    geometry_msgs::Pose msg;
    msg.position.x = point.x;
    msg.position.y = point.y;
    msg.position.z = point.z;
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

exploration_msgs::Frontier msgFrontier(const geometry_msgs::Point& c, double a, const geometry_msgs::Point& v){
    exploration_msgs::Frontier msg;
    msg.coordinate = c;
    msg.area = a;
    msg.variance = v;
    return msg;
}

}

#endif //COMMON_LIB_HPP