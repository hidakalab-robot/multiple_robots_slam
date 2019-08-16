//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/construct.hpp>
#include <exploration_msgs/FrontierArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <thread>

class ExplorationManager
{
private:
    double END_AREA;
    int END_FRONTIER;
    double END_TIME;

    ExpLib::Struct::subStructSimple map_;
    ExpLib::Struct::subStructSimple frontier_;

    ExpLib::Struct::pubStruct<std_msgs::Bool> areaEnd_;
    ExpLib::Struct::pubStruct<std_msgs::Bool> frontierEnd_;
    ExpLib::Struct::pubStruct<std_msgs::Bool> timerEnd_;

    ExpLib::Struct::pubStruct<std_msgs::Float64> areaVal_;
    ExpLib::Struct::pubStruct<std_msgs::Int32> frontierVal_;
    ExpLib::Struct::pubStruct<std_msgs::Float64> timerVal_;

    void frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void timer(void);

public:
    ExplorationManager();
    void multiThreadMain(void);
};

ExplorationManager::ExplorationManager()
    :map_("map", 1, &ExplorationManager::mapCB, this)
    ,frontier_("frontier", 1, &ExplorationManager::frontierCB, this)
    ,areaEnd_("end/area",1,true)
    ,frontierEnd_("end/frontier",1,true)
    ,timerEnd_("end/timer",1,true)
    ,areaVal_("end/area/value",1,true)
    ,frontierVal_("end/frontier/value",1,true)
    ,timerVal_("end/timer/value",1,true){

    ros::NodeHandle p("~");
    p.param<int>("end_frontier",END_FRONTIER,0);
    p.param<double>("end_time",END_TIME,1200);// second
    p.param<double>("end_area",END_AREA,46.7*14-9.5*10-((4.1+2.7+7.5)*10-2.7*5.8)-8.0*10-7.5*10-0.9*10);//267.46
    double AREA_RATE,AREA_TOLERANCE;
    p.param<double>("area_rate",AREA_RATE,1.0);
    p.param<double>("area_tolerance",AREA_TOLERANCE,0.9);
    END_AREA *= AREA_RATE * AREA_TOLERANCE;
};


void ExplorationManager::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    int free = 0;
    for(const auto& m : msg->data){
        if(m == 0) ++free;
    }
    int val = msg->info.resolution * msg->info.resolution * free;
    areaVal_.pub.publish(ExpLib::Construct::msgDouble(val));
    areaEnd_.pub.publish(ExpLib::Construct::msgBool(val >= END_AREA ? true : false));
}

void ExplorationManager::frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg){
    int val = msg->frontiers.size();
    frontierVal_.pub.publish(ExpLib::Construct::msgInt(val));
    frontierEnd_.pub.publish(ExpLib::Construct::msgBool(val <= END_FRONTIER ? true : false));
}


void ExplorationManager::timer(void){
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1);
    while(ros::ok()){
        double elapsedTime = ros::Duration(ros::Time::now()-startTime).toSec();
        timerVal_.pub.publish(ExpLib::Construct::msgDouble(elapsedTime));
        timerEnd_.pub.publish(ExpLib::Construct::msgBool(elapsedTime >= END_TIME ? true : false));
        rate.sleep();
    }
}

void ExplorationManager::multiThreadMain(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread timerThread([this]{timer();});
    ros::spin();
    timerThread.join();
    ROS_INFO_STREAM("end main loop\n");
}

#endif //EXPLORATION_MANAGER_HPP