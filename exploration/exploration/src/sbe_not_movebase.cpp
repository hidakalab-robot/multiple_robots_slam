#include <exploration/movement.hpp>
#include <exploration/branch_search.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor_based_exploration_not_movebase");

    BranchSearch bs;
    Movement mv;

    geometry_msgs::Point goal;
    ros::NodeHandle p("~");
    bool DEBUG_MODE;
    p.param<bool>("debug_mode",DEBUG_MODE,false);

    while(ros::ok()){
        if(bs.getGoal(goal) && !DEBUG_MODE){
            mv.moveToGoal(goal);
        }
        else{
            mv.moveToForward();
        }
    }
    
    return 0;
}
