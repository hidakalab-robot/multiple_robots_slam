#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

// clicked point の座標でrosparam を更新する
int main(int argc, char** argv){
    ros::init(argc, argv, "simulator_input_support");
    ros::NodeHandle nh("~");

    std::string PARAMETER_NAMESPACE;
    nh.param<std::string>("parameter_namespace",PARAMETER_NAMESPACE,"multi_exploration_simulator");

    ROS_INFO_STREAM("in");

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, [&](const geometry_msgs::PointStamped::ConstPtr& msg){
        ROS_INFO_STREAM("input clicked_point (" << msg->point.x << ", " << msg->point.y << ")");
        std::string name;
        ROS_INFO_STREAM("enter an element name, e.g. robot1, branch1 and frontier1.");
        std::cin >> name;
        nh.setParam("/"+PARAMETER_NAMESPACE+"/"+name+"_x",msg->point.x);
        nh.setParam("/"+PARAMETER_NAMESPACE+"/"+name+"_y",msg->point.y);
        ROS_INFO_STREAM("set << " << name << "_x : " << msg->point.x << ", " << name << "_y : " << msg->point.y);
    });

    ros::spin();

    return 0;
}