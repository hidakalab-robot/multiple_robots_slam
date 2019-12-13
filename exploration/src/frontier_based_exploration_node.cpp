#include <ros/ros.h>
#include <exploration/frontier_based_exploration.h>
#include <exploration/movement.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <exploration_libraly/struct.h>
#include <geometry_msgs/PointStamped.h>

//mainのところにexploration_endを評価する部分
int main(int argc, char *argv[]){
    ros::init(argc, argv, "frontier_based_exploration_movebase");

    FrontierBasedExploration fbe;
    Movement mv;

    ExpLib::Struct::subStruct<std_msgs::Bool> end("end",1);
    ExpLib::Struct::pubStruct<std_msgs::Int8> checkpub("/timing_check", 1);
    geometry_msgs::PointStamped goal;

    ros::NodeHandle p("~");
    bool DEBUG,ROTATION,AUTO_FINISH;
    p.param<bool>("debug",DEBUG,false);
    p.param<bool>("rotation",ROTATION,true);
    p.param<bool>("auto_finish",AUTO_FINISH,true);

    usleep(2e5);//timeがsim_timeに合うのを待つ
    ros::Time start = ros::Time::now();
    std_msgs::Int8 timingStatus;
    timingStatus.data = 1;
    checkpub.pub.publish(timingStatus);
    sleep(1);
    timingStatus.data = 2;
    checkpub.pub.publish(timingStatus);
    if(!DEBUG && ROTATION) mv.oneRotation();

    while(ros::ok()){
        if(fbe.getGoal(goal) && !DEBUG) mv.moveToGoal(goal);
        if(AUTO_FINISH && !end.q.callOne(ros::WallDuration(0.5)) && end.data.data) break;
        ros::spinOnce();
    }

    ROS_INFO_STREAM("exploration finish !! -> time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");
    timingStatus.data = 3;
    checkpub.pub.publish(timingStatus);
    ros::shutdown();
    
    return 0;
}