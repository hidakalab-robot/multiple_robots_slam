#include <costmap_to_voronoi/costmap_to_voronoi.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_to_voronoi");

  std::string name;
  ros::NodeHandle p("~");
  p.getParam("topic_ns", name);

  CostmapToVoronoi ctv(name);

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  std::vector<geometry_msgs::PoseStamped> plan;

  start.header.frame_id = "robot1/map";
  goal.header.frame_id = "robot1/map";

  goal.pose.position.x = 40.5;
  goal.pose.position.y = 9.5;

  while(ros::ok()){
    ctv.voronoiProcess(start,goal,plan);
  }

  return 0;
}
