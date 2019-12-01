#include <exploration_support/exploration_manager.h>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/construct.h>
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

ExplorationManager::ExplorationManager()
    :map_(new ExStc::subStructSimple("map", 1, &ExplorationManager::mapCB, this))
    ,frontier_(new ExStc::subStructSimple("frontier", 1, &ExplorationManager::frontierCB, this))
    ,areaEnd_(new ExStc::pubStruct<std_msgs::Bool>("end/area",1,true))
    ,frontierEnd_(new ExStc::pubStruct<std_msgs::Bool>("end/frontier",1,true))
    ,timerEnd_(new ExStc::pubStruct<std_msgs::Bool>("end/timer",1,true))
    ,areaVal_(new ExStc::pubStruct<std_msgs::Float64>("end/area/value",1,true))
    ,frontierVal_(new ExStc::pubStruct<std_msgs::Int32>("end/frontier/value",1,true))
    ,timerVal_(new ExStc::pubStruct<std_msgs::Float64>("end/timer/value",1,true))
    ,drs_(new dynamic_reconfigure::Server<exploration_support::exploration_manager_parameter_reconfigureConfig>(ros::NodeHandle("~/exmng"))){
    loadParams();
    drs_->setCallback(boost::bind(&ExplorationManager::dynamicParamsCB,this, _1, _2));
}

ExplorationManager::~ExplorationManager(){
    if(OUTPUT_EXMNG_PARAMETERS) outputParams();
}

void ExplorationManager::multiThreadMain(void){
    ROS_INFO_STREAM("start threads");
    ros::spinOnce();
    std::thread timerThread([this]{timer();});
    ros::spin();
    timerThread.join();
    ROS_INFO_STREAM("end main loop");
}

void ExplorationManager::mapCB(const nav_msgs::OccupancyGridConstPtr& msg){
    int free = 0;
    for(const auto& m : msg->data){
        if(m == 0) ++free;
    }
    int val = msg->info.resolution * msg->info.resolution * free;
    areaVal_->pub.publish(ExCos::msgDouble(val));
    areaEnd_->pub.publish(ExCos::msgBool(val >= END_AREA ? true : false));
}

void ExplorationManager::frontierCB(const exploration_msgs::FrontierArrayConstPtr& msg){
    int val = msg->frontiers.size();
    frontierVal_->pub.publish(ExCos::msgInt(val));
    frontierEnd_->pub.publish(ExCos::msgBool(val <= END_FRONTIER ? true : false));
}


void ExplorationManager::timer(void){
    usleep(2e5); // sim time が追いつくのを待機
    ros::Time startTime = ros::Time::now();
    ros::Rate rate(1);
    while(ros::ok()){
        double elapsedTime = ros::Duration(ros::Time::now()-startTime).toSec();
        timerVal_->pub.publish(ExCos::msgDouble(elapsedTime));
        timerEnd_->pub.publish(ExCos::msgBool(elapsedTime >= END_TIME ? true : false));
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