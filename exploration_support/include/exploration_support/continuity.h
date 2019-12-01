//topicのpublishレートを上げる(中間を埋めるだけ)
#ifndef CONTINUITY_H
#define CONTINUITY_H

#include <ros/ros.h>
#include <exploration_libraly/struct.h>

namespace ExStc = ExpLib::Struct;
template<typename T>
class Continuity{
    private:
        ExStc::subStruct<T> sub_;
        ExStc::pubStruct<T> pub_;

    public:
        Continuity(const std::string& sub_topic, const std::string& pub_topic)
            :sub_(sub_topic,1),pub_(pub_topic,1){};

        void publish(void){
            sub_.q.callOne(ros::WallDuration(0.1));
            pub_.pub.publish(sub_.data);
        };
        
        void publishNow(void){
            sub_.q.callOne(ros::WallDuration(0.1));
            sub_.data.header.stamp = ros::Time::now();
            pub_.pub.publish(sub_.data);
        };
};

#endif //CONTINUITY_H