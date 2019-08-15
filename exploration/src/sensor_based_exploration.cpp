#include <exploration/sensor_based_exploration.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sensor_based_exploration");

    SensorBasedExploration sbe;
    Movement mv;

    ExpLib::subStruct<std_msgs::Bool> end("end",1);

    geometry_msgs::PointStamped goal;
    ros::NodeHandle p("~");
    bool DEBUG,ROTATION,AUTO_FINISH;
    double BRANCH_WAIT_TIME;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);
    p.param<double>("branch_wait_time",BRANCH_WAIT_TIME,1.0);

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

    if(!DEBUG && ROTATION) mv.oneRotation();

    while(ros::ok()){
        branchTimer() && sbe.getGoal(goal) && !DEBUG ? mv.moveToGoal(goal) : mv.moveToForward();
        if(AUTO_FINISH && !end.q.callOne(ros::WallDuration(0.5)) && end.data.data) break;
    }

    ROS_INFO_STREAM("exploration finish !! -> time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");

    ros::shutdown();

    return 0;
}