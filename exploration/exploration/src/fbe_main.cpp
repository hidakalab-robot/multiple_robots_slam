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
    int END_CONDITION;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);
    p.param<int>("end_condition",END_CONDITION,0);// 0:面積, 1:未知領域

    auto getGoal = [&]{
        switch (fs.getGoal(goal)){
            case FrontierSearch::goalStatus::FOUND://ゴールがあった時
                if(!DEBUG) mv.moveToGoal(goal);
                return true;
            case FrontierSearch::goalStatus::NOTHING://ゴールがなかったとき終了するかどうか
                return END_CONDITION == 1 ? false : true;
            default:
                return true;
        }
    };

    usleep(2e5);//timeがsim_timeに合うのを待つ
    ros::Time start = ros::Time::now();

    if(!DEBUG && ROTATION) mv.oneRotation();

    while(ros::ok()){
        if(!getGoal() && AUTO_FINISH) break;
        if(AUTO_FINISH && END_CONDITION == 0 && !isEnd.q.callOne(ros::WallDuration(0.5)) && isEnd.data.data) break;
    }

    ROS_INFO_STREAM("exploration finish !! time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");

    ros::shutdown();
    
    return 0;
}