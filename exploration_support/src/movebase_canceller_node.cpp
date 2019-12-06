#include<actionlib_msgs/GoalStatusArray.h>
#include<ros/ros.h>

/*
how to use

$ rosrun exploration movebase_canceller _ns:=robot1
ns = namespace of move_base topic
*/

int main(int argc, char *argv[]){
    ros::init(argc, argv, "movebase_breaker");

    std::string ns;
    ros::NodeHandle("~").getParam("ns",ns);

    ros::Publisher pub = ros::NodeHandle().advertise<actionlib_msgs::GoalID>(ros::names::append(ns,"move_base/cancel"),1,true);
    ros::Subscriber sub = ros::NodeHandle().subscribe<actionlib_msgs::GoalStatusArray>(ros::names::append(ns,"move_base/status"),1,[&pub](const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
        static actionlib_msgs::GoalID lastId;
        static bool repeat = false;
        if(msg->status_list.empty()) return;
        actionlib_msgs::GoalID newId = msg->status_list.back().goal_id;
        if(lastId.id != newId.id){
            repeat = false;
            //その目標を許すかどうかの判断をする
            ROS_INFO_STREAM("id : " << newId.id);
            ROS_INFO_STREAM("Move to this target ? [y/n] : ");
            std::string input;
            std::getline(std::cin,input);
            if(input == "n"){
                pub.publish(newId);
                lastId = newId;
                repeat = true;
                ROS_INFO_STREAM("Rejected");
                return;
            }
            lastId = newId;
            ROS_INFO_STREAM("Accepted");
        }
        else if(repeat && msg->status_list.back().status != actionlib_msgs::GoalStatus::PREEMPTED) pub.publish(newId);
    });

    ros::spin();
    
    return 0;
}