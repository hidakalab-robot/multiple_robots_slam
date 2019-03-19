#include <exploration/branch_search.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor_based_exploration_movebase");

    BranchSearch bs;
    Movement mv;

    geometry_msgs::Point goal;
    
    while(ros::ok()){
        if(bs.getGoal(goal)){
            mv.moveToGoal(goal,true);
        }
        else{
            mv.moveToForward();
        }
    }
    
    return 0;
}