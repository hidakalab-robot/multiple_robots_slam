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
#include <dynamic_reconfigure/server.h>
#include <exploration_support/exploration_manager_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;

class ExplorationManager
{
private:
    // dynamic parameters
    double END_AREA;
    int END_FRONTIER;
    double END_TIME;

    // static parameters
    std::string EXMNG_PARAMETER_FILE_PATH;
    bool OUTPUT_EXMNG_PARAMETERS;

    // variables
    ExStc::subStructSimple map_;
    ExStc::subStructSimple frontier_;
    ExStc::pubStruct<std_msgs::Bool> areaEnd_;
    ExStc::pubStruct<std_msgs::Bool> frontierEnd_;
    ExStc::pubStruct<std_msgs::Bool> timerEnd_;
    ExStc::pubStruct<std_msgs::Float64> areaVal_;
    ExStc::pubStruct<std_msgs::Int32> frontierVal_;
    ExStc::pubStruct<std_msgs::Float64> timerVal_;
    dynamic_reconfigure::Server<exploration_support::exploration_manager_parameter_reconfigureConfig> drs_;

    // functions
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg);
    void timer(void);
    void loadParams(void);
    void dynamicParamsCB(exploration_support::exploration_manager_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    ExplorationManager();
    ~ExplorationManager(){if(OUTPUT_EXMNG_PARAMETERS) outputParams();};
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
    ,timerVal_("end/timer/value",1,true)
    ,drs_(ros::NodeHandle("~/exmng")){
    loadParams();
    drs_.setCallback(boost::bind(&ExplorationManager::dynamicParamsCB,this, _1, _2));
};

void ExplorationManager::multiThreadMain(void){
    ROS_INFO_STREAM("start threads");
    ros::spinOnce();
    std::thread timerThread([this]{timer();});
    ros::spin();
    timerThread.join();
    ROS_INFO_STREAM("end main loop");
}

void ExplorationManager::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    int free = 0;
    for(const auto& m : msg->data){
        if(m == 0) ++free;
    }
    int val = msg->info.resolution * msg->info.resolution * free;
    areaVal_.pub.publish(ExCos::msgDouble(val));
    areaEnd_.pub.publish(ExCos::msgBool(val >= END_AREA ? true : false));
}

void ExplorationManager::frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg){
    int val = msg->frontiers.size();
    frontierVal_.pub.publish(ExCos::msgInt(val));
    frontierEnd_.pub.publish(ExCos::msgBool(val <= END_FRONTIER ? true : false));
}


void ExplorationManager::timer(void){
    usleep(2e5); // sim time が追いつくのを待機
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1);
    while(ros::ok()){
        double elapsedTime = ros::Duration(ros::Time::now()-startTime).toSec();
        timerVal_.pub.publish(ExCos::msgDouble(elapsedTime));
        timerEnd_.pub.publish(ExCos::msgBool(elapsedTime >= END_TIME ? true : false));
        rate.sleep();
    }
}

void ExplorationManager::loadParams(void){
    ros::NodeHandle nh("~/exmng");
    // dynamic parameters
    nh.param<double>("end_area",END_AREA,267.46);// m^2
    nh.param<int>("end_frontier",END_FRONTIER,0);
    nh.param<double>("end_time",END_TIME,1200);// second
    // static parameters
    nh.param<std::string>("exmng_parameter_file_path",EXMNG_PARAMETER_FILE_PATH,"exmng_last_parameters.yaml");
    nh.param<bool>("output_exmng_parameters",OUTPUT_EXMNG_PARAMETERS,true);
}

void ExplorationManager::dynamicParamsCB(exploration_support::exploration_manager_parameter_reconfigureConfig &cfg, uint32_t level){
    END_AREA = cfg.end_area;
    END_FRONTIER = cfg.end_frontier;
    END_TIME = cfg.end_time;
}

void ExplorationManager::outputParams(void){
    std::cout << "writing exmng last parameters ... ..." << std::endl;
    std::ofstream ofs(EXMNG_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "exmng param file open succeeded" << std::endl;
    else {
        std::cout << "exmng param file open failed" << std::endl;
        return;
    }

    ofs << "end_area: " << END_AREA << std::endl;
    ofs << "end_frontier: " << END_FRONTIER << std::endl;
    ofs << "end_time: " << END_TIME << std::endl;
 }

#endif //EXPLORATION_MANAGER_HPP