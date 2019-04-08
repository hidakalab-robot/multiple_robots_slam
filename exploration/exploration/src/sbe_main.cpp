#include <exploration/branch_search.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sensor_based_exploration_movebase");

    BranchSearch bs;
    Movement mv;

    geometry_msgs::Point goal;
    ros::NodeHandle p("~");
    bool DEBUG;
    p.param<bool>("debug",DEBUG,false);
    
    if(!DEBUG){
        mv.oneRotation();
    }

    while(ros::ok()){
        if(bs.getGoal(goal) && !DEBUG){
            mv.moveToGoal(goal);
        }
        else{
            mv.moveToForward();
        }
    }
    
    return 0;
}