//topicのpublishレートを上げる(中間を埋めるだけ)
#ifndef CONTINUITY_HPP
#define CONTINUITY_HPP

#include <ros/ros.h>
#include <exploration_libraly/struct.hpp>

template<typename T>
class Continuity
{
private:
    ExpLib::subStruct<T> sub_;
    ExpLib::pubStruct<T> pub_;

public:
    Continuity(const std::string& sub_topic, const std::string& pub_topic):sub_(sub_topic,1),pub_(pub_topic,1){};
    void publish(void){
        sub_.q.callOne(ros::WallDuration(0.1));
        pub_.pub.publish(sub_.data);
    };
};

#endif //CONTINUITY_HPP