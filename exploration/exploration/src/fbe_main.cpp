#include <exploration/frontier_search.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "frontier_based_exploration_movebase");

    FrontierSearch fs;
    Movement mv;

    geometry_msgs::Point goal;

    ros::NodeHandle p("~");
    bool DEBUG;
    p.param<bool>("debug",DEBUG,false);

    if(!DEBUG){
        mv.oneRotation();
    }

    while(ros::ok()){
        if(fs.getGoal(goal) && !DEBUG){
            mv.moveToGoal(goal);
        }
    }
    
    return 0;
}