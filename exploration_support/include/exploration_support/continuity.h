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
        std::string tfName_;
        std::string ctfName_;

    public:
        Continuity(const std::string& sub_topic, const std::string& pub_topic, const std::string& tfn="none", const std::string& ctfn="none")
            :sub_(sub_topic,1),pub_(pub_topic,1),tfName_(tfn),ctfName_(ctfn){};

        void publish(void){
            sub_.q.callOne(ros::WallDuration(0.1));
            pub_.pub.publish(sub_.data);
        };

        void publishTf(void){
            sub_.q.callOne(ros::WallDuration(0.1));
            sub_.data.header.frame_id = tfName_ != "none" ? tfName_ : sub_.data.header.frame_id;
            sub_.data.child_frame_id = ctfName_ != "none" ? ctfName_ : sub_.data.child_frame_id;
            pub_.pub.publish(sub_.data);
        };
        
        void publishNow(void){
            sub_.q.callOne(ros::WallDuration(0.1));
            sub_.data.header.stamp = ros::Time::now();
            pub_.pub.publish(sub_.data);
        };

        void publishTfNow(void){
            sub_.q.callOne(ros::WallDuration(0.1));
            sub_.data.header.frame_id = tfName_ != "none" ? tfName_ : sub_.data.header.frame_id;
            sub_.data.child_frame_id = ctfName_ != "none" ? ctfName_ : sub_.data.child_frame_id;
            sub_.data.header.stamp = ros::Time::now();
            pub_.pub.publish(sub_.data);
        };
};

#endif //CONTINUITY_H