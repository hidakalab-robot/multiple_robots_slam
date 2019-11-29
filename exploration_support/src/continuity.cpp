#include <exploration_support/continuity.h>
#include <exploration_libraly/struct.h>
#include <ros/ros.h>

namespace ExStc = ExpLib::Struct;

template<typename T>
Continuity<T>::Continuity(const std::string& sub_topic, const std::string& pub_topic):sub_(sub_topic,1),pub_(pub_topic,1){};
    
template<typename T>
void Continuity<T>::publish(void){
    sub_.q.callOne(ros::WallDuration(0.1));
    pub_.pub.publish(sub_.data);
}

template<typename T>
void Continuity<T>::publishNow(void){
    sub_.q.callOne(ros::WallDuration(0.1));
    sub_.data.header.stamp = ros::Time::now();
    pub_.pub.publish(sub_.data);
};
