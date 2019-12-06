#include <exploration_support/visualization.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/struct.h>
#include <exploration_msgs/FrontierArray.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <actionlib_msgs/GoalStatusArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <exploration_msgs/AvoidanceStatus.h>

namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;

Visualization::Visualization()
    :pose_(new ExStc::subStructSimple("pose",1,&Visualization::poseCB, this))
    ,posePath_(new ExStc::pubStruct<nav_msgs::Path>("visualization/pose", 1))
    ,pp_(new nav_msgs::Path())
    ,goal_(new ExStc::subStructSimple("goal",1,&Visualization::goalCB, this))
    ,goalMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/goal", 1, true))
    ,gm_(new visualization_msgs::Marker())
    ,goSt_(new ExStc::subStructSimple("move_base/status",1,&Visualization::goalStatusCB, this))
    ,gmMutex_(new std::mutex())
    ,branch_(new ExStc::subStructSimple("branch",1,&Visualization::branchCB, this))
    ,branchMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/branch", 1, true))
    ,bm_(new visualization_msgs::Marker())
    ,dupBranch_(new ExStc::subStructSimple("duplicated_branch",1,&Visualization::dupBranchCB, this))
    ,dupBranchMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/duplicated_branch", 1, true))
    ,dbm_(new visualization_msgs::Marker())
    ,omBranch_(new ExStc::subStructSimple("on_map_branch",1,&Visualization::omBranchCB, this))
    ,omBranchMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/on_map_branch", 1, true))
    ,obm_(new visualization_msgs::Marker())
    ,frontier_(new ExStc::subStructSimple("frontier",1,&Visualization::frontierCB, this))
    ,frontierMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/frontier", 1, true))
    ,fm_(new visualization_msgs::Marker())
    ,useFro_(new ExStc::subStructSimple("useful_frontier",1,&Visualization::useFroCB, this))
    ,useFroMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/useful_frontier", 1, true))
    ,ufm_(new visualization_msgs::Marker())
    ,road_(new ExStc::subStructSimple("road",1,&Visualization::roadCB, this))
    ,roadMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/road", 1, true))
    ,rm_(new visualization_msgs::Marker())
    ,rmMutex_(new std::mutex())
    ,avoSta_(new ExStc::subStructSimple("movement_status",1,&Visualization::avoStaCB, this))
    ,avoStaMarker_(new ExStc::pubStruct<visualization_msgs::MarkerArray>("visualization/avoidance_status", 1, true))
    ,asmt_(new visualization_msgs::Marker())
    ,asm_(new visualization_msgs::MarkerArray())
    ,caGoals_(new ExStc::subStructSimple("canceled_goals",1,&Visualization::caGoalsCB, this))
    ,caGoalsMarker_(new ExStc::pubStruct<visualization_msgs::Marker>("visualization/canceled_goals", 1, true))
    ,cgm_(new visualization_msgs::Marker()){
    loadParams();
    *gm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,1.0,0.0,1.0);
    *bm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,1.0,1.0,0.0);
    *dbm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,1.0,0.85,0.85);
    *obm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,0.0,1.0,0.0);
    *fm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,0.0,1.0,1.0);
    *ufm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,1.0,0.5,0.5);
    *rm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,0.5,0.5,1.0);
    *asmt_ = ExCos::msgMarker(INIT_FRAME_ID,0.1,0.5,1.0,0.5,1.0,visualization_msgs::Marker::LINE_STRIP);
    *cgm_ = ExCos::msgMarker(INIT_FRAME_ID,0.5,0.5,0.5,0.5);
}

Visualization::~Visualization(){};

void Visualization::multiThreadMain(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread ppThread([this]() { posePathPublisher(); });
    std::thread gmThread([this]() { goalMarkerPublisher(); });
    std::thread bmThread([this]() { branchMarkerPublisher(); });
    std::thread dbmThread([this]() { dupBranchMarkerPublisher(); });
    std::thread obmThread([this]() { omBranchMarkerPublisher(); });
    std::thread fmThread([this]() { frontierMarkerPublisher(); });
    std::thread ufmThread([this]() { useFroMarkerPublisher(); });
    std::thread rmThread([this]() { roadMarkerPublisher(); });
    std::thread asmThread([this]() { avoStaMarkerPublisher(); });
    std::thread cgmThread([this]() { caGoalsMarkerPublisher(); });
    ros::spin();
    ppThread.join();//スレッドの終了を待つ
    gmThread.join();
    bmThread.join();
    dbmThread.join();
    obmThread.join();
    fmThread.join();
    ufmThread.join();
    rmThread.join();
    asmThread.join();
    cgmThread.join();
    ROS_INFO_STREAM("end main loop\n");
}

void Visualization::poseCB(const geometry_msgs::PoseStampedConstPtr& msg){
    pp_->poses.push_back(*msg);
    pp_->header.frame_id = msg->header.frame_id;
    pp_->header.stamp = ros::Time::now();
}

void Visualization::goalCB(const geometry_msgs::PointStampedConstPtr& msg){
    std::lock_guard<std::mutex> lock(*gmMutex_);
    gm_->points = ExCos::oneFactorVector(msg->point);
    gm_->header.frame_id = msg->header.frame_id;
    gm_->header.stamp = ros::Time::now();
}

void Visualization::goalStatusCB(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
    if(msg->status_list.empty()||(!msg->status_list.empty() && (msg->status_list.back().status > 2))){
        std::lock_guard<std::mutex> lock(*gmMutex_);
        gm_->points = std::vector<geometry_msgs::Point>();
        gm_->header.stamp = ros::Time::now();
    }
    else{
        std::lock_guard<std::mutex> lock(*rmMutex_);
        rm_->points = std::vector<geometry_msgs::Point>();
        rm_->header.stamp = ros::Time::now();
    }
}

void Visualization::branchCB(const exploration_msgs::PointArrayConstPtr& msg){
    bm_->points = msg->points;
    bm_->header.frame_id = msg->header.frame_id;
    bm_->header.stamp = ros::Time::now();
}

void Visualization::dupBranchCB(const exploration_msgs::PointArrayConstPtr& msg){
    dbm_->points = msg->points;
    dbm_->header.frame_id = msg->header.frame_id;
    dbm_->header.stamp = ros::Time::now();
}

void Visualization::omBranchCB(const exploration_msgs::PointArrayConstPtr& msg){
    obm_->points = msg->points;
    obm_->header.frame_id = msg->header.frame_id;
    obm_->header.stamp = ros::Time::now();
}

void Visualization::frontierCB(const exploration_msgs::FrontierArrayConstPtr& msg){
    auto replace = [&msg]{
        std::vector<geometry_msgs::Point> p;
        p.reserve(msg->frontiers.size());
        for(auto&& f : msg->frontiers) p.emplace_back(f.point); 
        return p;
    };
    fm_->points = replace();
    fm_->header.frame_id = msg->header.frame_id;
    fm_->header.stamp = ros::Time::now();
}

void Visualization::useFroCB(const exploration_msgs::FrontierArrayConstPtr& msg){
    auto replace = [&msg]{
        std::vector<geometry_msgs::Point> p;
        p.reserve(msg->frontiers.size());
        for(auto&& f : msg->frontiers) p.emplace_back(f.point); 
        return p;
    };
    ufm_->points = replace();
    ufm_->header.frame_id = msg->header.frame_id;
    ufm_->header.stamp = ros::Time::now();
}

void Visualization::roadCB(const geometry_msgs::PointStampedConstPtr& msg){
    std::lock_guard<std::mutex> lock(*rmMutex_);
    rm_->points = msg->header.frame_id != "" ? ExCos::oneFactorVector(msg->point) : std::vector<geometry_msgs::Point>();
    rm_->header.frame_id = msg->header.frame_id != "" ? msg->header.frame_id : INIT_FRAME_ID;
    rm_->header.stamp = ros::Time::now();
}

void Visualization::avoStaCB(const exploration_msgs::AvoidanceStatusConstPtr& msg){
    // 回避する距離のラインを表示したい
    visualization_msgs::MarkerArray ta;
    
    int SIZE = (msg->scan_angle_max - msg->scan_angle_min)/msg->scan_angle_increment;
    // tasm_のpoints部分を書き換えてemplacebackする
    for(int i=0,ie=msg->range_pattern.size();i!=ie;++i){
        std::vector<geometry_msgs::Point> tpv;
        if(msg->status=="VFH" || msg->status=="EMERGENCY"){
            // ~pointsを書き換える処理~
            // std::vector<geometry_msgs::Point> tpv;
            tpv.reserve(SIZE);
            for(int j=0;j!=SIZE;++j){
                // 表示したい距離 msg->range_pattern[i];
                geometry_msgs::Point tp;
                double theta = msg->scan_angle_min+msg->scan_angle_increment*j;
                switch (msg->calc_range_method){
                    case exploration_msgs::AvoidanceStatus::NORMAL:
                        // 距離にcosをかけた値がx
                        tp = ExCos::msgPoint(msg->range_pattern[i]*cos(theta),msg->range_pattern[i]*sin(theta));
                        break;
                    case exploration_msgs::AvoidanceStatus::COS:
                        // 距離がそのままx, 
                        tp = ExCos::msgPoint(msg->range_pattern[i],msg->range_pattern[i]*tan(theta));
                        break;
                    default:
                        ROS_INFO_STREAM("Invalid calc_range_method");
                        break;
                }
                tpv.emplace_back(std::move(tp));
            }
        }
        asmt_->points = tpv;
        asmt_->text = asmt_->ns = msg->descriptions[i];
        asmt_->header.frame_id = msg->scan_frame_id != "" ? msg->scan_frame_id : INIT_FRAME_ID;
        asmt_->header.stamp = ros::Time::now();
        asmt_->color.g = float(float(i+1)/msg->range_pattern.size());
        ta.markers.emplace_back(*asmt_);
    }
    *asm_ = ta;
}

void Visualization::caGoalsCB(const exploration_msgs::PointArrayConstPtr& msg){
    cgm_->points = msg->points;
    cgm_->header.frame_id = msg->header.frame_id != "" ? msg->header.frame_id : INIT_FRAME_ID;
    cgm_->header.stamp = ros::Time::now();
}

void Visualization::posePathPublisher(void){
    ros::Rate rate(POSE_PUBLISH_RATE);
    while(ros::ok()){
        posePath_->pub.publish(*pp_);
        rate.sleep();
    }
}

void Visualization::goalMarkerPublisher(void){
    ros::Rate rate(GOAL_PUBLISH_RATE);
    while(ros::ok()){
        goalMarker_->pub.publish(*gm_);
        rate.sleep();
    }
}

void Visualization::branchMarkerPublisher(void){
    ros::Rate rate(BRANCH_PUBLISH_RATE);
    while(ros::ok()){
        branchMarker_->pub.publish(*bm_);
        rate.sleep();
    }
}

void Visualization::dupBranchMarkerPublisher(void){
    ros::Rate rate(DUPLICATED_BRANCH_PUBLISH_RATE);
    while(ros::ok()){
        dupBranchMarker_->pub.publish(*dbm_);
        rate.sleep();
    }
}

void Visualization::omBranchMarkerPublisher(void){
    ros::Rate rate(ON_MAP_BRANCH_PUBLISH_RATE);
    while(ros::ok()){
        omBranchMarker_->pub.publish(*obm_);
        rate.sleep();
    }
}

void Visualization::frontierMarkerPublisher(void){
    ros::Rate rate(FRONTIER_PUBLISH_RATE);
    while(ros::ok()){
        frontierMarker_->pub.publish(*fm_);
        rate.sleep();
    }
}

void Visualization::useFroMarkerPublisher(void){
    ros::Rate rate(USEFUL_FRONTIER_PUBLISH_RATE);
    while(ros::ok()){
        useFroMarker_->pub.publish(*ufm_);
        rate.sleep();
    }
}

void Visualization::roadMarkerPublisher(void){
    ros::Rate rate(ROAD_PUBLISH_RATE);
    while(ros::ok()){
        roadMarker_->pub.publish(*rm_);
        rate.sleep();
    }
}

void Visualization::avoStaMarkerPublisher(void){
    ros::Rate rate(AVOIDANCE_STATUS_PUBLISH_RATE);
    while(ros::ok()){
        avoStaMarker_->pub.publish(*asm_);
        rate.sleep();
    }
}

void Visualization::caGoalsMarkerPublisher(void){
    ros::Rate rate(CANCELED_GOALS_PUBLISH_RATE);
    while(ros::ok()){
        caGoalsMarker_->pub.publish(*cgm_);
        rate.sleep();
    }
}

void Visualization::loadParams(void){
    ros::NodeHandle nh("~");
    // static parameters
    nh.param<std::string>("init_frame_id", INIT_FRAME_ID, "robot1/map");
    nh.param<double>("pose_publish_rate", POSE_PUBLISH_RATE, 10.0);
    nh.param<double>("goal_publish_rate", GOAL_PUBLISH_RATE, 10.0);
    nh.param<double>("branch_publish_rate", BRANCH_PUBLISH_RATE, 10.0);
    nh.param<double>("duplicated_branch_publish_rate", DUPLICATED_BRANCH_PUBLISH_RATE, 10.0);
    nh.param<double>("on_map_branch_publish_rate", ON_MAP_BRANCH_PUBLISH_RATE, 10.0);
    nh.param<double>("frontier_publish_rate", FRONTIER_PUBLISH_RATE, 10.0);
    nh.param<double>("useful_frontier_publish_rate", USEFUL_FRONTIER_PUBLISH_RATE, 10.0);
    nh.param<double>("road_publish_rate", ROAD_PUBLISH_RATE, 10.0);
    nh.param<double>("avoidance_status_publish_rate", AVOIDANCE_STATUS_PUBLISH_RATE, 10.0);
    nh.param<double>("canceled_goals_publish_rate", CANCELED_GOALS_PUBLISH_RATE, 10.0);
}