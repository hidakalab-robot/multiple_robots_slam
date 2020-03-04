#include <exploration_support/continuity.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "continuity");
    std::string tfName;
    std::string ctfName;
    ros::NodeHandle("~").param<std::string>("tf_name",tfName,"none");
    ros::NodeHandle("~").param<std::string>("child_tf_name",ctfName,"none");

    Continuity<nav_msgs::Odometry> cod("sub_topic", "pub_topic",tfName,ctfName);
    while(ros::ok()) cod.publishTfNow();    
    return 0;
}