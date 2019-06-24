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
        ros::spinOnce();
    };
    
    PathPlanning(const std::string& costmapName, const std::string& plannerName):tfl(ros::Duration(10)),gcr(costmapName, tfl){
        planner.initialize(plannerName,&gcr);
        ros::spinOnce();
    };

    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        ros::spinOnce();

        return planner.makePlan(start,goal,plan);
    };

    bool createPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGrid& map){//only voronoi
        ros::spinOnce();
        return planner.makePlan(start,goal,plan,map);
    };

    bool getDistance(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance){
        ros::spinOnce();
        std::vector<geometry_msgs::PoseStamped> plan;
        ROS_INFO_STREAM("start : (" << start.pose.position.x << ", " << start.pose.position.y << ", " << start.header.frame_id << ") , goal : (" << goal.pose.position.x << ", " << goal.pose.position.y <<  ", " << start.header.frame_id << ")");        // if(createPath(start,goal,plan)){
        if(planner.makePlan(start,goal,plan)){
            // plan に path が入ってるので長さを計算する
            distance = 0;
            for(int i=1,ie=plan.size();i!=ie;++i) distance += Eigen::Vector2d(plan[i].pose.position.x - plan[i-1].pose.position.x, plan[i].pose.position.y - plan[i-1].pose.position.y).norm();
            return true;
        }
        return false;
    }

    bool getDistanceAndVec(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double& distance, Eigen::Vector2d& vec){
        ros::spinOnce();
        std::vector<geometry_msgs::PoseStamped> plan;
        ROS_INFO_STREAM("start : (" << start.pose.position.x << ", " << start.pose.position.y << ", " << start.header.frame_id << ") , goal : (" << goal.pose.position.x << ", " << goal.pose.position.y <<  ", " << start.header.frame_id << ")");
        if(planner.makePlan(start,goal,plan)){
        // if(createPath(start,goal,plan)){
            // plan に path が入ってるので長さを計算する
            distance = 0;
            for(int i=1,ie=plan.size();i!=ie;++i) distance += Eigen::Vector2d(plan[i].pose.position.x - plan[i-1].pose.position.x, plan[i].pose.position.y - plan[i-1].pose.position.y).norm();
            //最後の80%ぐらいを使うか?
            int b = plan.size() - 1;
            int a = b * 0.8;
            //a-b間のベクトル
            vec = Eigen::Vector2d(plan[b].pose.position.x - plan[a].pose.position.x, plan[b].pose.position.y - plan[a].pose.position.y).normalized();
            ROS_INFO_STREAM("path function return: true");
            return true;
        }
        ROS_INFO_STREAM("path function return: false");
        return false;
    }
};

#endif //PATH_PLANNING_HPP