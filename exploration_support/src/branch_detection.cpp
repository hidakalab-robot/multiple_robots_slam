#include <exploration_support/branch_detection.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/utility.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/branch_detection_parameter_reconfigureConfig.h>
#include <fstream>
#include <tf/transform_listener.h>
#include <Eigen/Core>

namespace ExStc = ExpLib::Struct;
namespace ExUtl = ExpLib::Utility;
namespace ExCos = ExpLib::Construct;

BranchDetection::BranchDetection()
    :scan_(new ExStc::subStructSimple("scan", 1, &BranchDetection::scanCB, this))
    ,pose_(new ExStc::subStruct<geometry_msgs::PoseStamped>("pose", 1))
    ,branch_(new ExStc::pubStruct<exploration_msgs::PointArray>("branch", 1))
    ,filteredScan_(new ExStc::pubStruct<sensor_msgs::LaserScan>("filtered_scan", 1))
    ,drs_(new dynamic_reconfigure::Server<exploration_support::branch_detection_parameter_reconfigureConfig>(ros::NodeHandle("~/branch"))){
    loadParams();
    drs_->setCallback(boost::bind(&BranchDetection::dynamicParamsCB,this, _1, _2));
}

BranchDetection::~BranchDetection(){
    if(OUTPUT_BRANCH_PARAMETERS) outputParams();
}

void BranchDetection::scanCB(const sensor_msgs::LaserScanConstPtr& msg){
    sensor_msgs::LaserScan scan = SCAN_FILTER ? scanFilter(*msg) : *msg;

    if(pose_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        publishBranch(std::vector<geometry_msgs::Point>(),pose_->data.header.frame_id);
        return;
    }

    static bool initialized = false;
    static tf::TransformListener listener;
    if(!initialized){
        // listener.waitForTransform(pose_->data.header.frame_id, msg->header.frame_id, ros::Time(), ros::Duration(1.0));
        listener.waitForTransform(pose_->data.header.frame_id, scan.header.frame_id, ros::Time(), ros::Duration(1.0));
        initialized = true;
    }

    // センサと障害物の距離が近い時は検出を行わない
    // for(int t=OBSTACLE_CHECK_ANGLE/msg->angle_increment,i=(msg->ranges.size()/2)-1-t,ie=(msg->ranges.size()/2)+t;i!=ie;++i){
    for(int t=OBSTACLE_CHECK_ANGLE/scan.angle_increment,i=(scan.ranges.size()/2)-1-t,ie=(scan.ranges.size()/2)+t;i!=ie;++i){
		// if(!std::isnan(msg->ranges[i]) && msg->ranges[i] < OBSTACLE_RANGE_THRESHOLD){
		if(!std::isnan(scan.ranges[i]) && scan.ranges[i] < OBSTACLE_RANGE_THRESHOLD){
            publishBranch(std::vector<geometry_msgs::Point>(),pose_->data.header.frame_id);
            return;
		}
    }

    // nan のデータを取り除いて 距離と角度のデータに分ける
    // ExStc::scanStruct ss(msg->ranges.size());
    ExStc::scanStruct ss(scan.ranges.size());

    // for(int i=0,e=msg->ranges.size();i!=e;++i){
	// 	if(!std::isnan(msg->ranges[i])){
    //         float temp = msg->angle_min+(msg->angle_increment*i);
	// 		ss.ranges.emplace_back(msg->ranges[i]);
    //         ss.x.emplace_back(msg->ranges[i]*cos(temp));
    //         ss.y.emplace_back(msg->ranges[i]*sin(temp));
	// 		ss.angles.emplace_back(std::move(temp));
	// 	}
    // }
    for(int i=0,e=scan.ranges.size();i!=e;++i){
		if(!std::isnan(scan.ranges[i])){
            float temp = scan.angle_min+(scan.angle_increment*i);
			ss.ranges.emplace_back(scan.ranges[i]);
            ss.x.emplace_back(scan.ranges[i]*cos(temp));
            ss.y.emplace_back(scan.ranges[i]*sin(temp));
			ss.angles.emplace_back(std::move(temp));
		}
    }
    if(ss.ranges.size() < 2){
		ROS_ERROR_STREAM("Scan data is insufficient");
        publishBranch(std::vector<geometry_msgs::Point>(),pose_->data.header.frame_id);
        return;
    }

    // 分岐検出部
    std::vector<geometry_msgs::Point> branches; // 検出した分岐領域を入れる
    for(int i=0,e=ss.ranges.size()-1;i!=e;++i){
		if(ss.angles[i] * ss.angles[i+1] < 0) continue; // 二つの角度の符号が違うときスキップ/
        if(ss.ranges[i] > BRANCH_RANGE_MAX || ss.ranges[i+1] > BRANCH_RANGE_MAX) continue; // 距離が遠いのは信用できないのでだめ
        if(ss.ranges[i] < BRANCH_RANGE_MIN || ss.ranges[i+1] < BRANCH_RANGE_MIN) continue; // 距離が近すぎるのもだめ
        if(ss.y[i] <= ss.y[i+1]) continue; // yの大きさが i < i+1 だとNG
        if((ss.angles[i] < 0 && ss.x[i] >= ss.x[i+1]) || (ss.angles[i] >= 0 && ss.x[i] <= ss.x[i+1])) continue; // xの差を判定
        double diffX = std::abs(ss.x[i+1] - ss.x[i]);
        if(BRANCH_DIFF_X_MIN > diffX || diffX > BRANCH_DIFF_X_MAX) continue; //x座標の差(分岐の幅)が範囲内じゃないと分岐と認めない
        double diffY = std::abs(ss.y[i+1] - ss.y[i]);
        if(BRANCH_DIFF_Y_MIN > diffY || diffY > BRANCH_DIFF_Y_MAX) continue; //y座標の差が範囲内じゃないと分岐と認めない
        // 検出した座標をpose座標系に変換して input
        // branches.emplace_back(ExUtl::coordinateConverter2d<geometry_msgs::Point>(listener, pose_->data.header.frame_id, msg->header.frame_id, ExCos::msgPoint((ss.x[i+1] + ss.x[i])/2, (ss.y[i+1] + ss.y[i])/2)));
        branches.emplace_back(ExUtl::coordinateConverter2d<geometry_msgs::Point>(listener, pose_->data.header.frame_id, scan.header.frame_id, ExCos::msgPoint((ss.x[i+1] + ss.x[i])/2, (ss.y[i+1] + ss.y[i])/2)));
	}

    ROS_INFO_STREAM("Branch Found : " << branches.size());
    
    if(BRANCH_FILTER){
        branchFilter(branches);
        ROS_INFO_STREAM("filtered Branch size: " << branches.size());
    }
    
    publishBranch(branches,pose_->data.header.frame_id);
}

sensor_msgs::LaserScan BranchDetection::scanFilter(const sensor_msgs::LaserScan& scan){
    static std::vector<sensor_msgs::LaserScan> scanLog;
    scanLog.emplace_back(scan);
    if(scanLog.size()<SCAN_FILTER_ORDER) return scan;

    sensor_msgs::LaserScan filteredScan = scan;

    for(int i=0,ie=filteredScan.ranges.size();i!=ie;++i){
        float sum = 0;
        int nan = 0;
        for(int j=scanLog.size()-1,je=j-SCAN_FILTER_ORDER;j!=je;--j){
            if(!std::isnan(scanLog[j].ranges[i])) sum += scanLog[j].ranges[i];
            else ++nan;
        }
        if(nan == SCAN_FILTER_ORDER) continue;
        filteredScan.ranges[i] = sum/(SCAN_FILTER_ORDER-nan);
    }

    filteredScan_->pub.publish(filteredScan);
    return filteredScan;
}

void BranchDetection::branchFilter(std::vector<geometry_msgs::Point>& branches){
    static std::vector<std::vector<geometry_msgs::Point>> branchLog;
    branchLog.emplace_back(branches);
    if(branchLog.size()<BRANCH_FILTER_ORDER) return;

    // 過去BRANCH_FILTER_ORDER個前までのデータに同じくらいのやつが出続けてないとだめ
    // 一個でもなかった時点で削除
    branches.erase(std::remove_if(branches.begin(),branches.end(),[&,this](geometry_msgs::Point& p){
        for(int i=branchLog.size()-2,ie=i-(BRANCH_FILTER_ORDER-1);i!=ie;--i){
            for(int j=0,je=branchLog[i].size();j!=je;++j){
                if(Eigen::Vector2d(p.x - branchLog[i][j].x, p.y - branchLog[i][j].y).norm()>BRANCH_FILTER_TOLERANCE) return true;
            }
        }
        return false;
    }),branches.end());
}

void BranchDetection::publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId){
    exploration_msgs::PointArray msg;
    msg.points = branches;
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    branch_->pub.publish(msg);
    ROS_INFO_STREAM("Publish branch");
}

void BranchDetection::loadParams(void){
    ros::NodeHandle nh("~/branch");
    // dynamic parameters
    nh.param<double>("obstacle_check_angle", OBSTACLE_CHECK_ANGLE, 0.04);
    nh.param<double>("obstacle_range_threshold", OBSTACLE_RANGE_THRESHOLD,2.5);
    nh.param<double>("branch_range_max", BRANCH_RANGE_MAX, 6.0);
    nh.param<double>("branch_range_min", BRANCH_RANGE_MIN, 0.2);
	nh.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
    nh.param<double>("branch_diff_x_max", BRANCH_DIFF_X_MAX, 10.0);
    nh.param<double>("branch_diff_y_min", BRANCH_DIFF_Y_MIN, 0.0);
    nh.param<double>("branch_diff_y_max", BRANCH_DIFF_Y_MAX, 10.0);
    nh.param<bool>("scan_filter", SCAN_FILTER, false);
    nh.param<int>("scan_filter_order", SCAN_FILTER_ORDER, 3);
    nh.param<bool>("branch_filter", BRANCH_FILTER, false);
    nh.param<int>("branch_filter_order", BRANCH_FILTER_ORDER, 3);
    nh.param<double>("branch_filter_tolerance", BRANCH_FILTER_TOLERANCE, 0.2);
    // static parameters
    nh.param<std::string>("branch_parameter_file_path",BRANCH_PARAMETER_FILE_PATH,"branch_last_parameters.yaml");
    nh.param<bool>("output_branch_parameters",OUTPUT_BRANCH_PARAMETERS,true);
}

void BranchDetection::dynamicParamsCB(exploration_support::branch_detection_parameter_reconfigureConfig &cfg, uint32_t level){
    OBSTACLE_CHECK_ANGLE = cfg.obstacle_check_angle;
    OBSTACLE_RANGE_THRESHOLD = cfg.obstacle_range_threshold;
    BRANCH_RANGE_MAX = cfg.branch_range_max;
    BRANCH_RANGE_MIN = cfg.branch_range_min;
    BRANCH_DIFF_X_MIN = cfg.branch_diff_x_min;
    BRANCH_DIFF_X_MAX = cfg.branch_diff_x_max;
    BRANCH_DIFF_Y_MIN = cfg.branch_diff_y_min;
    BRANCH_DIFF_Y_MAX = cfg.branch_diff_y_max;
    SCAN_FILTER = cfg.scan_filter;
    SCAN_FILTER_ORDER = cfg.scan_filter_order;
    BRANCH_FILTER = cfg.branch_filter;
    BRANCH_FILTER_ORDER = cfg.branch_filter_order;
    BRANCH_FILTER_TOLERANCE = cfg.branch_filter_tolerance;
}

void BranchDetection::outputParams(void){
    std::cout << "writing branch last parameters ... ..." << std::endl;
    std::ofstream ofs(BRANCH_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "branch param file open succeeded" << std::endl;
    else {
        std::cout << "branch param file open failed" << std::endl;
        return;
    }

    ofs << "obstacle_check_angle: " << OBSTACLE_CHECK_ANGLE << std::endl;
    ofs << "obstacle_range_threshold: " << OBSTACLE_RANGE_THRESHOLD << std::endl;
    ofs << "branch_range_max: " << BRANCH_RANGE_MAX << std::endl;
    ofs << "branch_range_min: " << BRANCH_RANGE_MIN << std::endl;
    ofs << "branch_diff_x_min: " << BRANCH_DIFF_X_MIN << std::endl;
    ofs << "branch_diff_x_max: " << BRANCH_DIFF_X_MAX << std::endl;
    ofs << "branch_diff_y_min: " << BRANCH_DIFF_Y_MIN << std::endl;
    ofs << "branch_diff_y_max: " << BRANCH_DIFF_Y_MAX << std::endl;
    ofs << "scan_filter: " << (SCAN_FILTER ? "true" : "false")<< std::endl;
    ofs << "scan_filter_order: " << SCAN_FILTER_ORDER << std::endl;
    ofs << "branch_filter: " << (BRANCH_FILTER  ? "true" : "false")<< std::endl;
    ofs << "branch_filter_order: " << BRANCH_FILTER_ORDER << std::endl;
    ofs << "branch_filter_tolerance: " << BRANCH_FILTER_TOLERANCE << std::endl;
 }