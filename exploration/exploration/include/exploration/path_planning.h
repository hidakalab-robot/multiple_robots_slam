#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

template<typename T>
class pathPlanning
{
private:
    tf::TransformListener tf;
    costmap_2d::Costmap2DROS gcr;
    std::string name;
    T planner;

    void rosSpinOnce(void);
public:
    pathPlanning();
    pathPlanning(std::string name);
    ~pathPlanning(){};

    bool createPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan);
};

template<typename T>
pathPlanning<T>::pathPlanning():tf(ros::Duration(10)),gcr("path_planner", tf){
    name = "path_planner";
    planner.initialize(name,&gcr);
}

template<typename T>
pathPlanning<T>::pathPlanning(std::string ns):tf(ros::Duration(10)),gcr(ns, tf){
    name = ns;
    planner.initialize(name,&gcr);
}

template<typename T>
void pathPlanning<T>::rosSpinOnce(void){
    ros::spinOnce();
}

template<typename T>
bool pathPlanning<T>::createPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan){
    rosSpinOnce();
    //T planner;
    ROS_DEBUG_STREAM("Path planner name : " << name << "\n");
    //planner.initialize(name,&gcr);

    return planner.makePlan(start,goal,plan);
}

#endif //PATH_PLANNING_H