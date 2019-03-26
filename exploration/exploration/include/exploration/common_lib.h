#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

namespace CommonLib
{
template <typename T>
struct subStruct{
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::CallbackQueue q;
    T data;
    //template <class U,class V>
    subStruct(const std::string& topic,uint32_t queue_size){
        n.setCallbackQueue(&q);
        //sub = n.subscribe(topic,queue_size,fp,obj);
        //sub = n.subscribe<T>(topic, queue_size, [obj, this](const boost::shared_ptr<const T>& msg) {data = *msg;});
        sub = n.subscribe<T>(topic, queue_size, [this](const boost::shared_ptr<const T>& msg) {data = *msg;});//データをコピーするコールバック関数を自動生成//違う処理をさせたい場合はクラスのコンストラクタで上書き
    }
};

template<typename T>
struct pubStruct{
    ros::NodeHandle n;
    ros::Publisher pub;
    pubStruct(const std::string& topic,uint32_t queue_size,bool latch=false){
        pub = n.advertise<T>(topic, queue_size, latch);
    }
};

double qToYaw(const geometry_msgs::Quaternion& q){
    tf::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(tq).getRPY(roll,pitch,yaw);
    return yaw;
}


geometry_msgs::Point msgPoint(double x=0,double y=0,double z=0){
    geometry_msgs::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    return msg;
}

std_msgs::Empty msgEmpty(){
    std_msgs::Empty msg;
    return msg;
}

geometry_msgs::Twist msgTwist(double x=0,double z=0){
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.angular.z = z;
    return msg;
}
}



#endif //COMMON_LIB_H