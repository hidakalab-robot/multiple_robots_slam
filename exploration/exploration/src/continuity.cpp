#include <exploration/continuity.hpp>
#include <kobuki_msgs/BumperEvent.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "continuity");

    ros::NodeHandle p("~");
    std::string bumperRawTopic;
    std::string bumperContinuityTopic;
    p.param<std::string>("bumper_raw_topic", bumperRawTopic, "bumper_raw");
    p.param<std::string>("bumper_continuity_topic", bumperContinuityTopic, "bumper_continuity");

    //remapにすればパラメータいらない

    Continuity<kobuki_msgs::BumperEvent> cbe(bumperRawTopic,bumperContinuityTopic);

    while(ros::ok()){
        cbe.publish();
    }
    
    return 0;
}