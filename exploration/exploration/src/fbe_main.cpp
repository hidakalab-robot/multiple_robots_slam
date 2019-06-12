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
    bool DEBUG,ROTATION,AUTO_FINISH;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);

    // auto getGoal = [&]{
    //     switch (fs.getGoal(goal)){
    //         case FrontierSearch::goalStatus::FOUND://ゴールがあった時
    //             if(!DEBUG) mv.moveToGoal(goal);
    //             return true;
    //         case FrontierSearch::goalStatus::NOTHING://ゴールがなかったとき終了するかどうか
    //             return END_CONDITION == 1 ? false : true;
    //         default:
    //             return true;
    //     }
    // };

    usleep(2e5);//timeがsim_timeに合うのを待つ
    ros::Time start = ros::Time::now();

    if(!DEBUG && ROTATION) mv.oneRotation();

    while(ros::ok()){
        if(fs.getGoal(goal) && !DEBUG) mv.moveToGoal(goal);
        if(AUTO_FINISH && !isEnd.q.callOne(ros::WallDuration(0.5)) && isEnd.data.data) break;
    }

    ROS_INFO_STREAM("exploration finish !! time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");

    ros::shutdown();
    
    return 0;
}