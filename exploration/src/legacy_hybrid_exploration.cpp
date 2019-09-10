#include <exploration/sensor_based_exploration.hpp>
#include <exploration/frontier_based_exploration.hpp>
#include <exploration/movement.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "legacy_hybrid_exploration");

    Movement mv;

    SensorBasedExploration sbe;
    FrontierBasedExploration fbe;
    ExpLib::Struct::subStruct<std_msgs::Int8> loop("loop_closure_counter/count",1);

    ExpLib::Struct::subStruct<std_msgs::Bool> end("end",1);

    geometry_msgs::PointStamped goal;
    ros::NodeHandle p("~");
    bool DEBUG,ROTATION,AUTO_FINISH;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);
    
    usleep(2e5);//timeがsim_timeに合うのを待つ

    ros::Time start = ros::Time::now();

    if(!DEBUG && ROTATION) mv.oneRotation();
    
    // sensor-based exploration
    {   
        double BRANCH_WAIT_TIME;
        int LOOP_COUNT_THRESHOLD;
        p.param<double>("branch_wait_time",BRANCH_WAIT_TIME,1.0);
        p.param<int>("loop_count_threshold",LOOP_COUNT_THRESHOLD,20);

        ros::Time lastBranchTime= ros::Time::now();

        auto branchTimer = [&]{
            if(ros::Duration(ros::Time::now()-lastBranchTime).toSec()>BRANCH_WAIT_TIME){
                lastBranchTime = ros::Time::now();
                return true;
            }
            return false;
        };

        while(ros::ok()){
            if(!loop.q.callOne(ros::WallDuration(0.5)) && loop.data.data >= LOOP_COUNT_THRESHOLD) break;
            branchTimer() && sbe.getGoal(goal) && !DEBUG ? mv.moveToGoal(goal) : mv.moveToForward();
            if(AUTO_FINISH && !end.q.callOne(ros::WallDuration(0.5)) && end.data.data) break;
        }
    }

    ros::Time switchTime = ros::Time::now();
    ROS_INFO_STREAM("Switch exploration method SBE to FBE -> switch time :" << ros::Duration(switchTime-start).toSec() << " [s]");

    // frontier-based exploration
    {
        while(ros::ok()){
            if(fbe.getGoal(goal) && !DEBUG) mv.moveToGoal(goal);
            if(AUTO_FINISH && !end.q.callOne(ros::WallDuration(0.5)) && end.data.data) break;
        }
    }

    ROS_INFO_STREAM("exploration finish !! -> time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");
    ROS_INFO_STREAM("switch time :" << ros::Duration(switchTime-start).toSec() << " [s]");

    ros::shutdown();

    return 0;
}
