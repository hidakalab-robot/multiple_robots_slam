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

    ros::Time start = ros::Time::now();

    if(!DEBUG) mv.oneRotation();

    while(ros::ok()){
        if(fs.getGoal(goal) && !DEBUG) mv.moveToGoal(goal);
        if(!isEnd.q.callOne(ros::WallDuration(0.5))&&isEnd.data.data) break;
    }

    ROS_INFO_STREAM("exploration finish !!");
    ROS_INFO_STREAM("finish time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");

    ros::shutdown();
    
    return 0;
}