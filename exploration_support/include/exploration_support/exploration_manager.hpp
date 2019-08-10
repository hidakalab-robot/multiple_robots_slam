//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_HPP
#define EXPLORATION_MANAGER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/constructor.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <exploration_msgs/FrontierArray.h>
#include <thread>

class ExplorationManager
{
private:
    double END_AREA;
    int END_FRONTIER;
    double END_TIME;

    ExpLib::subStructSimple map_;
    ExpLib::subStructSimple frontier_;

    ExpLib::pubStruct<std_msgs::Bool> areaEnd_;
    ExpLib::pubStruct<std_msgs::Bool> frontierEnd_;
    ExpLib::pubStruct<std_msgs::Bool> timerEnd_;

    ExpLib::pubStruct<std_msgs::Float64> areaVal_;
    ExpLib::pubStruct<std_msgs::Int32> frontierVal_;
    ExpLib::pubStruct<std_msgs::Float64> timerVal_;

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
    ,areaEnd_("end_condition/area/is_end",1,true)
    ,frontierEnd_("end_condition/frontier/is_end",1,true)
    ,timerEnd_("end_condition/timer/is_end",1,true)
    ,areaVal_("end_condition/area/value",1,true)
    ,frontierVal_("end_condition/frontier/value",1,true)
    ,timerVal_("end_condition/timer/value",1,true){

    ros::NodeHandle p("~");
    p.param<int>("end_frontier",this->END_FRONTIER,0);
    p.param<double>("end_time",this->END_TIME,1200);// second
    p.param<double>("end_area",this->END_AREA,46.7*14-9.5*10-((4.1+2.7+7.5)*10-2.7*5.8)-8.0*10-7.5*10-0.9*10);//267.46
    double AREA_RATE,AREA_TOLERANCE;
    p.param<double>("area_rate",AREA_RATE,1.0);
    p.param<double>("area_tolerance",AREA_TOLERANCE,0.9);
    this->END_AREA *= AREA_RATE * AREA_TOLERANCE;
};


void ExplorationManager::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    int free = 0;
    for(const auto& m : msg->data){
        if(m == 0) ++free;
    }
    int val = msg->info.resolution * msg->info.resolution * free;
    this->areaVal_.pub.publish(ExpLib::msgDouble(val));
    this->areaEnd_.pub.publish(ExpLib::msgBool(val >= this->END_AREA ? true : false));
}

void ExplorationManager::frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg){
    int val = msg->frontiers.size();
    this->frontierVal_.pub.publish(ExpLib::msgInt(val));
    this->frontierEnd_.pub.publish(ExpLib::msgBool(val <= this->END_FRONTIER ? true : false));
}


void ExplorationManager::timer(void){
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1);
    while(ros::ok()){
        double elapsedTime = ros::Duration(ros::Time::now()-startTime).toSec();
        this->timerVal_.pub.publish(ExpLib::msgDouble(elapsedTime));
        this->timerEnd_.pub.publish(ExpLib::msgBool(elapsedTime >= this->END_TIME ? true : false));
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