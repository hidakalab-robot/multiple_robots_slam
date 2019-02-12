#include <costmap_to_voronoi/costmap_to_voronoi.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_to_voronoi");

  //CostmapToVoronoi ctv;

  std::string name;
  ros::NodeHandle p("~");
  p.getParam("topic_ns", name);

  CostmapToVoronoi ctv(name);

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  std::vector<geometry_msgs::PoseStamped> plan;

  start.header.frame_id = "robot1/map";
  goal.header.frame_id = "robot1/map";

  goal.pose.position.x = 0.5;

  //ctv.rosSpinOnce();

  while(ros::ok()){
    //ctv.voronoiInitialize();
    //ctv.makeVoronoi(start,goal,plan);
    ctv.voronoiProcess(start,goal,plan);
    //ctv.rosSpinOnce();
  }

  return 0;
}
