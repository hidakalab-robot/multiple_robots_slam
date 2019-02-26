#include <exploration/frontier_search.h>
#include <exploration/movement.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "frontier_based_exploration");

    FrontierSearch fs;
    Movement mv;

    geometry_msgs::Point goal;

    mv.oneRotation();
    
    while(ros::ok()){
        if(fs.getGoal(goal)){
            mv.moveToGoal(goal,true);
        }
    }
    
    return 0;
}