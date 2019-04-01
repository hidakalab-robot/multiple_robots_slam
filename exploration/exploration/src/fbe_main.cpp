#include <exploration/frontier_search.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "frontier_based_exploration_movebase");

    FrontierSearch fs;
    Movement mv;

    geometry_msgs::Point goal;

    ros::NodeHandle p("~");
    bool DEBUG_MODE;
    p.param<bool>("debug_mode",DEBUG_MODE,false);

    if(!DEBUG_MODE){
        mv.oneRotation();
    }

    while(ros::ok()){
        if(fs.getGoal(goal) && !DEBUG_MODE){
            mv.moveToGoal(goal);
        }
    }
    
    return 0;
}