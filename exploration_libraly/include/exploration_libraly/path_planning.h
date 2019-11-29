#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <costmap_2d/costmap_2d_ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

/*
path_planning tutorial

In launch file 
    <rosparam file="$(find exploration)/param/path_planner_params.yaml" command="load"/>
    <param name="global_costmap/global_frame" value="map"/> map is robot map frame. e.g. /robot1/map 
    <param name="global_costmap/robot_base_frame" value="base_footprint"/>  base_footprint is robot base_footprint frame. e.g. /robot1/base_footprint

In source file

    if you want to use NavfnROS
        #include <exploration/path_planning.hpp>
        #include <navfn/navfn_ros.h>
            
            static PathPlanning<navfn::NavfnROS> pp("global_cosmap","NavfnROS");
            pp.createPath(start,goal,path); // createPath return bool and insert Navfn-path from start to goal into path 

    if you want to use VoronoiPlanner
        #include <exploration/path_planning.hpp>
        #include <voronoi_planner/planner_core.h>

            PathPlanning<voronoi_planner::VoronoiPlanner> pp("global_cosmap","voronoi_planner");
            pp.createPath(start,goal,path); // createPath return bool and insert voronoi-path from start to goal into path

            or

            pp.createPath(start,goal,path,map); // In addition to the above, insert voronoi-grid into map
*/

namespace ExpLib
{
template<typename T>
class PathPlanning
{
private:
    double PATH_TO_VECTOR_RATIO;
    tf::TransformListener tfl;
    costmap_2d::Costmap2DROS gcr;
    T planner;

public:
    PathPlanning();
    PathPlanning(const std::string& costmapName, const std::string& plannerName);

    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGrid& map); //only voronoi
    void getDistance(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance, std::vector<geometry_msgs::PoseStamped>& plan);
    bool getDistance(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance);
    void getVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, Eigen::Vector2d& vec, std::vector<geometry_msgs::PoseStamped>& plan);
    bool getVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, Eigen::Vector2d& vec);
    bool getDistanceAndVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance, Eigen::Vector2d& vec);
};
}

#endif //PATH_PLANNING_H