#include <exploration_support/continuity.h>
#include <kobuki_msgs/BumperEvent.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "bumper_continuity");
    Continuity<kobuki_msgs::BumperEvent> cbe("mobile_base/events/bumper", "bumper");
    while(ros::ok()) cbe.publish();    
    return 0;
}