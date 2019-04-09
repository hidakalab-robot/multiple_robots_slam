#include <exploration/frontier_search.hpp>
#include <exploration/movement.hpp>

//mainのところにexploration_endを評価する部分
int main(int argc, char *argv[]){
    ros::init(argc, argv, "frontier_based_exploration_movebase");

    FrontierSearch fs;
    Movement mv;

    CommonLib::subStruct<std_msgs::Bool> isEnd("end",1);

    geometry_msgs::Point goal;

    ros::NodeHandle p("~");
    bool DEBUG;
    p.param<bool>("debug",DEBUG,false);

    if(!DEBUG) mv.oneRotation();

    while(ros::ok()){
        if(fs.getGoal(goal) && !DEBUG) mv.moveToGoal(goal);
        if(!isEnd.q.callOne(ros::WallDuration(1))&&isEnd.data.data) break;
    }
    
    return 0;
}