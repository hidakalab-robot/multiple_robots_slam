#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

template<typename T>
class pathPlanning
{
private:
    tf::TransformListener tfl;
    costmap_2d::Costmap2DROS gcr;
    T planner;

public:
    pathPlanning():tfl(ros::Duration(10)),gcr("costmap", tfl){
        planner.initialize("path_planner",&gcr);
    };
    
    pathPlanning(const std::string& costmapName, const std::string& plannerName):tfl(ros::Duration(10)),gcr(costmapName, tfl){
        planner.initialize(plannerName,&gcr);
    };

    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        ros::spinOnce();
        return planner.makePlan(start,goal,plan);
    };

    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGrid& map){//only voronoi
        ros::spinOnce();
        return planner.makePlan(start,goal,plan,map);
    };
};

#endif //PATH_PLANNING_HPP