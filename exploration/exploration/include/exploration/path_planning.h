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
    //std::string plannerName;
    T planner;
    std::string plannerName_;

    void rosSpinOnce(void);
public:
    //pathPlanning();
    //pathPlanning(std::string name);
    pathPlanning(std::string costmapName, std::string plannerName);
    ~pathPlanning(){};

    bool createPath(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, bool initializer=false);
    bool createPath(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGrid& map, bool initializer=false);
};

// template<typename T>
// pathPlanning<T>::pathPlanning():tf(ros::Duration(10)),gcr("global_costmap", tf){
//     plannerName = "path_planner";
//     planner.initialize(plannerName,&gcr);
// }

// template<typename T>
// pathPlanning<T>::pathPlanning(std::string name):tf(ros::Duration(10)),gcr(name, tf){
//     //name = ns;
//     planner.initialize(name,&gcr);
// }

template<typename T>
pathPlanning<T>::pathPlanning(std::string costmapName, std::string plannerName):tf(ros::Duration(10)),gcr(costmapName, tf){
    planner.initialize(plannerName,&gcr);
    plannerName_ = plannerName;
}

template<typename T>
void pathPlanning<T>::rosSpinOnce(void){
    ros::spinOnce();
}

template<typename T>
bool pathPlanning<T>::createPath(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, bool initializer){
    rosSpinOnce();
    //T planner;
    //ROS_DEBUG_STREAM("Path planner name : " << name << "\n");
    //planner.initialize(name,&gcr);
    if(initializer){
        planner.initialize(plannerName_,&gcr);
    }
    return planner.makePlan(start,goal,plan);
}

template<typename T>
bool pathPlanning<T>::createPath(geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGrid& map, bool initializer){
    rosSpinOnce();
    //T planner;
    //ROS_DEBUG_STREAM("Path planner name : " << name << "\n");
    //planner.initialize(name,&gcr);
    if(initializer){
        planner.initialize(plannerName_,&gcr);
    }
    return planner.makePlan(start,goal,plan,map);
}

#endif //PATH_PLANNING_H