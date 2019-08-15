#ifndef STRUCT_HPP
#define STRUCT_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Point.h>
#include <exploration_libraly/enum.hpp>

namespace ExpLib
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

struct subStructStd{
    ros::NodeHandle n;
    ros::Subscriber sub;
};

template<typename T>
struct pubStruct{
    ros::NodeHandle n;
    ros::Publisher pub;
    pubStruct(const std::string& topic,uint32_t queue_size,bool latch=false){ pub = n.advertise<T>(topic, queue_size, latch);};
};

struct pubStructStd{
    ros::NodeHandle n;
    ros::Publisher pub;
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
    DuplicationStatus duplication;
    listStruct():duplication(ExpLib::DuplicationStatus::NOT_DUPLECATION){};
    listStruct(const geometry_msgs::Point& p):point(p),duplication(ExpLib::DuplicationStatus::NOT_DUPLECATION){};
};

}

#endif // STRUCT_HPP