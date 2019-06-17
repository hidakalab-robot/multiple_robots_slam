#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

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


template<typename T>
class PathPlanning
{
private:
    tf::TransformListener tfl;
    costmap_2d::Costmap2DROS gcr;
    T planner;

public:
    PathPlanning():tfl(ros::Duration(10)),gcr("costmap", tfl){
        planner.initialize("path_planner",&gcr);
    };
    
    PathPlanning(const std::string& costmapName, const std::string& plannerName):tfl(ros::Duration(10)),gcr(costmapName, tfl){
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

    double getPathLength(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal){
        std::vector<geometry_msgs::PoseStamped> plan;
        if(createPath(start,goal,plan)){
            // plan に path が入ってるので長さを計算する
            double pathLength = 0;

            for(int i=1,ie=plan.size();i!=ie;++i){
                // double l = Eigen::Vector2d(plan[i].pose.position.x - plan[i-1].pose.position.x, plan[i].pose.position.y - plan[i-1].pose.position.y).norm();
                // ROS_INFO_STREAM("Path diff " << i << " : " << l);
                // pathLength += l;
                pathLength += Eigen::Vector2d(plan[i].pose.position.x - plan[i-1].pose.position.x, plan[i].pose.position.y - plan[i-1].pose.position.y).norm();
            }
            return pathLength;
        }
        return -DBL_MAX;
    }
};

#endif //PATH_PLANNING_HPP