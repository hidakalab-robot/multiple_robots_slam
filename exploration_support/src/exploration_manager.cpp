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
    ,areaDiffEnd_(new ExStc::pubStruct<std_msgs::Bool>("end/area_diff",1,true))
    ,areaDiffRateEnd_(new ExStc::pubStruct<std_msgs::Bool>("end/area_diff_rate",1,true))
    ,areaDiffAveRateEnd_(new ExStc::pubStruct<std_msgs::Bool>("end/area_diff_averate",1,true))
    ,areaVal_(new ExStc::pubStruct<std_msgs::Float64>("end/area/value",1,true))
    ,frontierVal_(new ExStc::pubStruct<std_msgs::Int32>("end/frontier/value",1,true))
    ,timerVal_(new ExStc::pubStruct<std_msgs::Float64>("end/timer/value",1,true))
    ,areaDiffVal_(new ExStc::pubStruct<std_msgs::Float64>("end/area_diff/value",1,true))
    ,areaDiffRateVal_(new ExStc::pubStruct<std_msgs::Float64>("end/area_diff_rate/value",1,true))
    ,areaDiffAveRateVal_(new ExStc::pubStruct<std_msgs::Float64>("end/area_diff_averate/value",1,true))
    ,drs_(new dynamic_reconfigure::Server<exploration_support::exploration_manager_parameter_reconfigureConfig>(ros::NodeHandle("~/exmng"))){
    loadParams();
    area_ = 0;
    drs_->setCallback(boost::bind(&ExplorationManager::dynamicParamsCB,this, _1, _2));
}

ExplorationManager::~ExplorationManager(){
    if(OUTPUT_EXMNG_PARAMETERS) outputParams();
}

void ExplorationManager::multiThreadMain(void){
    ROS_INFO_STREAM("start threads");
    ros::spinOnce();
    std::thread timerThread([this]{timer();});
    std::thread areaDiffThread([this]{areaDiff();});
    ros::spin();
    timerThread.join();
    areaDiffThread.join();
    ROS_INFO_STREAM("end main loop");
}

void ExplorationManager::mapCB(const nav_msgs::OccupancyGridConstPtr& msg){
    int free = 0;
    for(const auto& m : msg->data){
        if(m == 0) ++free;
    }
    area_ = msg->info.resolution * msg->info.resolution * free;
    areaVal_->pub.publish(ExCos::msgDouble(area_));
    areaEnd_->pub.publish(ExCos::msgBool(area_ > END_AREA ? true : false));
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
        timerEnd_->pub.publish(ExCos::msgBool(elapsedTime > END_TIME ? true : false));
        rate.sleep();
    }
}

void ExplorationManager::areaDiff(void){
    usleep(2e5); // sim time が追いつくのを待機
    double beforeArea = area_;
    double diff = DBL_MAX;
    double diffRate = DBL_MAX;
    double diffAveRate = DBL_MAX;
    ros::Time setTime = ros::Time::now();
    std::vector<double> areaDiffLog;
    ros::Rate rate(1);

    while(ros::ok()){
        if(ros::Duration(ros::Time::now()-setTime).toSec() > END_AREA_DIFF_INTERVAL){
            diff = std::abs(area_ - beforeArea);
            beforeArea = area_;
            setTime = ros::Time::now();
            if(areaDiffLog.size()>0){
                diffRate = diff / areaDiffLog[areaDiffLog.size()-1];
                diffAveRate = diff/(std::accumulate(areaDiffLog.begin(),areaDiffLog.end(),0.0) / areaDiffLog.size());
            }
            areaDiffLog.emplace_back(diff);
        }
        if(diff != DBL_MAX) areaDiffVal_->pub.publish(ExCos::msgDouble(diff));
        if(diffRate != DBL_MAX) areaDiffRateVal_->pub.publish(ExCos::msgDouble(diffRate));
        if(diffAveRate != DBL_MAX) areaDiffAveRateVal_->pub.publish(ExCos::msgDouble(diffAveRate));
        areaDiffEnd_->pub.publish(ExCos::msgBool(diff < END_AREA_DIFF ? true : false));
        areaDiffRateEnd_->pub.publish(ExCos::msgBool(diffRate < END_AREA_DIFF_RATE ? true : false));
        areaDiffAveRateEnd_->pub.publish(ExCos::msgBool(diffAveRate < END_AREA_DIFF_AVERATE ? true : false));
        rate.sleep();
    }
}

void ExplorationManager::loadParams(void){
    ros::NodeHandle nh("~/exmng");
    // dynamic parameters
    nh.param<double>("end_area",END_AREA,267.46);// m^2
    nh.param<int>("end_frontier",END_FRONTIER,0);
    nh.param<double>("end_time",END_TIME,1200);// second
    nh.param<double>("end_area_diff",END_AREA_DIFF,5);// m^2
    nh.param<double>("end_area_diff_interval",END_AREA_DIFF_INTERVAL,120);// second
    nh.param<double>("end_area_diff_rate",END_AREA_DIFF_RATE,0.5);//
    nh.param<double>("end_area_diff_averate",END_AREA_DIFF_AVERATE,0.5);//
    // static parameters
    nh.param<std::string>("exmng_parameter_file_path",EXMNG_PARAMETER_FILE_PATH,"exmng_last_parameters.yaml");
    nh.param<bool>("output_exmng_parameters",OUTPUT_EXMNG_PARAMETERS,true);
}

void ExplorationManager::dynamicParamsCB(exploration_support::exploration_manager_parameter_reconfigureConfig &cfg, uint32_t level){
    END_AREA = cfg.end_area;
    END_FRONTIER = cfg.end_frontier;
    END_TIME = cfg.end_time;
    END_AREA_DIFF = cfg.end_area_diff;
    END_AREA_DIFF_INTERVAL = cfg.end_area_diff_interval;
    END_AREA_DIFF_RATE = cfg.end_area_diff_rate;
    END_AREA_DIFF_AVERATE = cfg.end_area_diff_averate;
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
    ofs << "end_area_diff: " << END_AREA_DIFF << std::endl;
    ofs << "end_area_diff_interval: " << END_AREA_DIFF_INTERVAL << std::endl;
    ofs << "end_area_diff_rate: " << END_AREA_DIFF_RATE << std::endl;
    ofs << "end_area_diff_averate: " << END_AREA_DIFF_AVERATE << std::endl;
 }