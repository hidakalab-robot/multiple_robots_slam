#include <ros/ros.h>
#include <exploration/seamless_hybrid_exploration.h>
// #include <exploration/frontier_based_exploration.h>
#include <exploration/movement.h>
#include <std_msgs/Bool.h>
#include <exploration_libraly/struct.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "seamless_hybrid_exploration_actively_frontier");

    SeamlessHybridExploration she;
    // FrontierBasedExploration fbe;
    Movement mv;

    ExpLib::Struct::subStruct<std_msgs::Bool> end("end",1);
    ExpLib::Struct::subStruct<std_msgs::Bool> areaDiff("area_diff",1);

    geometry_msgs::PointStamped goal;
    ros::NodeHandle p("~");
    bool DEBUG,ROTATION,AUTO_FINISH;
    double BRANCH_WAIT_TIME;
    double AREA_DIFF_THREDHOLD;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);
    p.param<double>("branch_wait_time",BRANCH_WAIT_TIME,1.0);

    usleep(2e5);//timeがsim_timeに合うのを待つ

    ros::Time start = ros::Time::now();
    ros::Time getGoalTime= ros::Time::now();

    auto branchTimer = [&]{
        if(DEBUG && std::abs(ros::Duration(ros::Time::now()-getGoalTime).toSec())>BRANCH_WAIT_TIME){
            getGoalTime = ros::Time::now();
            return true;
        }
        else if(ros::Duration(ros::Time::now()-getGoalTime).toSec()>BRANCH_WAIT_TIME){
            getGoalTime = ros::Time::now();
            return true;
        }
        return false;
    };

    if(!DEBUG && ROTATION) mv.oneRotation();

    while(ros::ok()){
        if(!areaDiff.q.callOne(ros::WallDuration(0.5)) && areaDiff.data.data) break;// ここに切り替え条件入れる
        branchTimer() && she.getGoal(goal) && !DEBUG ? mv.moveToGoal(goal) : mv.moveToForward();
        if(AUTO_FINISH && !end.q.callOne(ros::WallDuration(0.5)) && end.data.data) break;
        ros::spinOnce();
    }

    ros::Time switchTime = ros::Time::now();
    ROS_INFO_STREAM("Switch exploration method SHE to FBE -> switch time :" << ros::Duration(switchTime-start).toSec() << " [s]");

    while(ros::ok()){
        if(she.getGoalAF(goal) && !DEBUG) mv.moveToGoal(goal);
        if(AUTO_FINISH && !end.q.callOne(ros::WallDuration(0.5)) && end.data.data) break;
        ros::spinOnce();
    }

    ROS_INFO_STREAM("exploration finish !! -> time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");
    ROS_INFO_STREAM("switch time :" << ros::Duration(switchTime-start).toSec() << " [s]");

    ros::shutdown();
    
    return 0;
}