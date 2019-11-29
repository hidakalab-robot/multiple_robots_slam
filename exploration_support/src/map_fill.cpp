#include <exploration_support/map_fill.h>
#include <exploration_libraly/struct.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/map_fill_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;

MapFill::MapFill()
    :map_("map", 1, &MapFill::mapCB, this)
    ,fillMap_("fill_map",1)
    ,drs_(ros::NodeHandle("~/map_fill")){
    loadParams();
    drs_.setCallback(boost::bind(&MapFill::dynamicParamsCB,this, _1, _2));
};

MapFill::~MapFill(){
    if(OUTPUT_FILL_PARAMETERS) outputParams();
}

void MapFill::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO_STREAM("map input");
    //マップから画像に変換 //-1 -> 255 , 100 -> 100 , 0 -> 0
    cv::Mat image(msg->info.height,msg->info.width,CV_8UC1,const_cast<signed char*>(msg->data.data()));

    //二値化
    cv::Mat binImage;
    cv::threshold(image,binImage,254,255,cv::THRESH_BINARY);
    
    //輪郭検出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

    //輪郭の面積を計算して、面積がある範囲内の場合のみ輪郭として抽出する // 大きすぎる領域を埋めないように
    std::vector<std::vector<cv::Point>> contoursRect;
    contoursRect.reserve(contours.size());
    for(int i=0,e=contours.size();i!=e;++i){
        double area = cv::contourArea(contours[i]);
        if(FILL_SIZE_MIN <= area && area <= FILL_SIZE_MAX) contoursRect.emplace_back(contours[i]);
    }

    //輪郭から塗りつぶし用のマスク画像を作成
    cv::Mat mask(image.size(),CV_8UC1,255);
    cv::drawContours(mask,contoursRect,-1,cv::Scalar(0),-1);

    //マスクで塗りつぶし
    cv::Mat result;
    image.copyTo(result,mask);

    //画像からマップに変換
    nav_msgs::OccupancyGrid map;
    map.info = msg->info;
    map.header = msg -> header;
    map.data = result.reshape(0,1);//画像データを一行に変換
    //未知領域部分の色を塗り替え
    std::replace(map.data.begin(),map.data.end(),127,-1);

    ROS_INFO_STREAM("map_image publish");
    fillMap_.pub.publish(map);
}

void MapFill::loadParams(void){
    ros::NodeHandle nh("~/map_fill");
    // dynamic parameters
    nh.param<double>("fill_size_min",FILL_SIZE_MIN,5);//px
    nh.param<double>("fill_size_max",FILL_SIZE_MAX,2000);//px
    // static parameters
    nh.param<std::string>("fill_parameter_file_path",FILL_PARAMETER_FILE_PATH,"fill_last_parameters.yaml");
    nh.param<bool>("output_fill_parameters",OUTPUT_FILL_PARAMETERS,true);
}

void MapFill::dynamicParamsCB(exploration_support::map_fill_parameter_reconfigureConfig &cfg, uint32_t level){
    FILL_SIZE_MIN = cfg.fill_size_min;
    FILL_SIZE_MAX = cfg.fill_size_max;
}

void MapFill::outputParams(void){
    std::cout << "writing fill last parameters ... ..." << std::endl;
    std::ofstream ofs(FILL_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "fill param file open succeeded" << std::endl;
    else {
        std::cout << "fill param file open failed" << std::endl;
        return;
    }

    ofs << "fill_size_min: " << FILL_SIZE_MIN << std::endl;
    ofs << "fill_size_max: " << FILL_SIZE_MAX << std::endl;
 }