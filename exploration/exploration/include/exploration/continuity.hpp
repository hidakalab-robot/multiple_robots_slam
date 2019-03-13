//topicのpublishレートを上げる(中間を埋めるだけ)
#ifndef CONTINUITY_H
#define CONTINUITY_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

template<typename T>
class Continuity
{
private:
    ros::NodeHandle s;
    ros::NodeHandle p;
    ros::CallbackQueue q;
    ros::Subscriber sub;
    ros::Publisher pub;
    T input;

    void subCB(const typename T::ConstPtr& msg);

public:
    Continuity(std::string subTopic, std::string pubTopic);
    ~Continuity(){};
    void publish(void);
};

template<typename T>
Continuity<T>::Continuity(std::string subTopic, std::string pubTopic){
    s.setCallbackQueue(&q);
    sub = s.subscribe(subTopic,1,&Continuity::subCB,this);
    pub = p.advertise<T>(pubTopic, 1);
}

template<typename T>
void Continuity<T>::subCB(const typename T::ConstPtr& msg){
    input = *msg;
}

template<typename T>
void Continuity<T>::publish(void){
    q.callOne(ros::WallDuration(0.1));
    pub.publish(input);
}

#endif //CONTINUITY_H