#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <voronoi_planner/planner_core.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_src/array_parser.cpp>
#include <costmap_src/costmap_2d.cpp>
#include <costmap_src/observation_buffer.cpp>
#include <costmap_src/layer.cpp>
#include <costmap_src/layered_costmap.cpp>
#include <costmap_src/costmap_2d_ros.cpp>
#include <costmap_src/costmap_2d_publisher.cpp>
#include <costmap_src/costmap_math.cpp>
#include <costmap_src/footprint.cpp>
#include <costmap_src/costmap_layer.cpp>
#include <voronoi_src/planner_core.cpp>


class CostmapToVoronoi
{
private:
  tf::TransformListener tf;
  costmap_2d::Costmap2DROS gcr;
  std::string name;
  //voronoi_planner::VoronoiPlanner vp;

public:
  CostmapToVoronoi();
  CostmapToVoronoi(std::string name);
  ~CostmapToVoronoi(){};

  //void voronoiInitialize();
  void rosSpinOnce();
  //void makeVoronoi(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
  void voronoiProcess(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
};

CostmapToVoronoi::CostmapToVoronoi()
:tf(ros::Duration(10)),
gcr("costmap_to_voronoi", tf)
{
  name = "costmap_to_voronoi";
}

CostmapToVoronoi::CostmapToVoronoi(std::string ns)
:tf(ros::Duration(10)),
gcr(ns, tf)
{
  name = ns;
}

// void CostmapToVoronoi::voronoiInitialize(){
//   vp.initialize(name, &gcr);
// }

void CostmapToVoronoi::rosSpinOnce(){
  ros::spinOnce();
}

// void CostmapToVoronoi::makeVoronoi(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
//   vp.makePlan(start,goal,plan);
// }

void CostmapToVoronoi::voronoiProcess(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
  rosSpinOnce();
  voronoi_planner::VoronoiPlanner vp;
  vp.initialize(name, &gcr);
  vp.makePlan(start,goal,plan);
}