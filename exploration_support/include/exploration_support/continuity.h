//topicのpublishレートを上げる(中間を埋めるだけ)
#ifndef CONTINUITY_H
#define CONTINUITY_H

#include <exploration_libraly/struct.h>
#include <ros/ros.h>

namespace ExStc = ExpLib::Struct;
template<typename T>
class Continuity
{
private:
    ExStc::subStruct<T> sub_;
    ExStc::pubStruct<T> pub_;

public:
    Continuity(const std::string& sub_topic, const std::string& pub_topic);
    void publish(void);
    void publishNow(void);
};

#endif //CONTINUITY_H