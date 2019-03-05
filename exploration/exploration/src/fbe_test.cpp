#include <exploration/frontier_search.h>
#include <exploration/movement.h>
#include <exploration/path_planning.h>
//#include <navfn/navfn_ros.h>
//#include <voronoi_planner/planner_core.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "frontier_based_exploration_test");

    FrontierSearch fs;
    Movement mv;
    //pathPlanning<navfn::NavfnROS> ppn;
    //pathPlanning<voronoi_planner::VoronoiPlanner> ppv;

    geometry_msgs::Point goal;

    //std::vector<geometry_msgs::PoseStamped> plan;
    while(ros::ok()){
        //if(fs.getGoal(goal)){
        if(fs.getGoal()){
            //mv.moveToGoal(goal,true);
            //plan = mv.createPath(goal);
        }
        else{
            //mv.moveToForward();
        }
    }
    
    return 0;
}