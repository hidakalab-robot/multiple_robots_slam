#include <exploration/branch_search.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sensor_based_exploration_movebase");

    BranchSearch bs;
    Movement mv;

    CommonLib::subStruct<std_msgs::Bool> isEnd("end",1);

    geometry_msgs::Point goal;
    ros::NodeHandle p("~");
    bool DEBUG,ROTATION,AUTO_FINISH;
    double BRANCH_WAIT_TIME;
    int END_CONDITION;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);
    p.param<double>("branch_wait_time",BRANCH_WAIT_TIME,1.0);
    p.param<int>("end_condition",END_CONDITION,0);// 0:面積, 1:未知領域

    usleep(2e5);//timeがsim_timeに合うのを待つ

    ros::Time start = ros::Time::now();
    ros::Time getGoalTime= ros::Time::now();

    auto branchTimer = [&]{
        if(ros::Duration(ros::Time::now()-getGoalTime).toSec()>BRANCH_WAIT_TIME){
            getGoalTime = ros::Time::now();
            return true;
        }
        return false;
    };
    // ROS_DEBUG_STREAM("start time : " << start);
    if(!DEBUG && ROTATION) mv.oneRotation();

    while(ros::ok()){
        branchTimer() && bs.getGoal(goal) && !DEBUG ? mv.moveToGoal(goal) : mv.moveToForward();
        if(AUTO_FINISH){
            if(END_CONDITION == 1 && bs.fs.frontierDetection<int>() == 0) break;
            if(END_CONDITION == 0 && !isEnd.q.callOne(ros::WallDuration(0.5))&&isEnd.data.data) break;
        }
    }

    ROS_INFO_STREAM("exploration finish !! time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");
    // ROS_DEBUG_STREAM("end time : " << ros::Time::now());

    ros::shutdown();

    return 0;
}
