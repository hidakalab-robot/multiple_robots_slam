#ifndef BRANCH_DETECTION_HPP
#define BRANCH_DETECTION_HPP

#include <exploration_libraly/construct.hpp>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/utility.hpp>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <dynamic_reconfigure/server.h>
#include <exploration_support/branch_detection_parameter_reconfigureConfig.h>
#include <fstream>

class BranchDetection
{
private:
    double OBSTACLE_CHECK_ANGLE;
    double OBSTACLE_RANGE_MIN;
    double SCAN_RANGE_THRESHOLD;
    // double BRANCH_MIN_X;
    // double BRANCH_MAX_X;
	double BRANCH_DIFF_X_MIN;
    double BRANCH_DIFF_X_MAX;
    // double BRANCH_MIN_Y;
    // double BRANCH_MAX_Y;
	double BRANCH_DIFF_Y_MIN;
    double BRANCH_DIFF_Y_MAX;

    tf::TransformListener listener_;

    ExpLib::Struct::subStructSimple scan_;
    ExpLib::Struct::subStruct<geometry_msgs::PoseStamped> pose_;
    ExpLib::Struct::pubStruct<exploration_msgs::PointArray> branch_;

    ros::NodeHandle nh;
    dynamic_reconfigure::Server<exploration_support::branch_detection_parameter_reconfigureConfig> server;
    dynamic_reconfigure::Server<exploration_support::branch_detection_parameter_reconfigureConfig>::CallbackType cbt;
    bool OUTPUT_BRANCH_PARAMETERS;
    std::string BRANCH_PARAMETER_FILE_PATH;

    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId);
    void dynamicParamCallback(exploration_support::branch_detection_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);
public:
    BranchDetection();
    ~BranchDetection(){if(OUTPUT_BRANCH_PARAMETERS) outputParams();};
};

BranchDetection::BranchDetection()
    :scan_("scan", 1, &BranchDetection::scanCB, this)
    ,pose_("pose", 1)
    ,branch_("branch", 1)
    ,nh("~/branch")
    ,server(nh){
    
    nh.param<double>("obstacle_check_angle", OBSTACLE_CHECK_ANGLE, 0.04);
    nh.param<double>("obstacle_range_min", OBSTACLE_RANGE_MIN,2.5);
    nh.param<double>("scan_range_threshold", SCAN_RANGE_THRESHOLD, 6.0);
    // nh.param<double>("branch_max_x", BRANCH_MAX_X, 5.5);
	nh.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
    nh.param<double>("branch_diff_x_max", BRANCH_DIFF_X_MAX, 10.0);
    nh.param<double>("branch_diff_y_min", BRANCH_DIFF_Y_MIN, 0.0);
    nh.param<double>("branch_diff_y_max", BRANCH_DIFF_Y_MAX, 10.0);
    nh.param<bool>("output_branch_parameters",OUTPUT_BRANCH_PARAMETERS,true);
    nh.param<std::string>("branch_parameter_file_path",BRANCH_PARAMETER_FILE_PATH,"branch_last_parameters.yaml");

    cbt = boost::bind(&BranchDetection::dynamicParamCallback,this, _1, _2);
    server.setCallback(cbt);
}

void BranchDetection::dynamicParamCallback(exploration_support::branch_detection_parameter_reconfigureConfig &cfg, uint32_t level){
    OBSTACLE_CHECK_ANGLE = cfg.obstacle_check_angle;
    OBSTACLE_RANGE_MIN = cfg.obstacle_range_min;
    SCAN_RANGE_THRESHOLD = cfg.scan_range_threshold;
    // BRANCH_MAX_X = cfg.branch_max_x;
    BRANCH_DIFF_X_MIN = cfg.branch_diff_x_min;
    BRANCH_DIFF_X_MAX = cfg.branch_diff_x_max;
    BRANCH_DIFF_Y_MIN = cfg.branch_diff_y_min;
    BRANCH_DIFF_Y_MAX = cfg.branch_diff_y_max;
}

void BranchDetection::outputParams(void){
    std::cout << "writing last parameters ... ..." << std::endl;
    std::ofstream ofs(BRANCH_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "file open succeeded" << std::endl;
    else {
        std::cout << "file open failed" << std::endl;
        return;
    }
    ofs << "obstacle_check_angle: " << OBSTACLE_CHECK_ANGLE << std::endl;
    ofs << "obstacle_range_mix: " << OBSTACLE_RANGE_MIN << std::endl;
    ofs << "scan_range_threshold: " << SCAN_RANGE_THRESHOLD << std::endl;
    // ofs << "branch_max_x: " << BRANCH_MAX_X << std::endl;
    ofs << "branch_diff_x_min: " << BRANCH_DIFF_X_MIN << std::endl;
    ofs << "branch_diff_x_max: " << BRANCH_DIFF_X_MAX << std::endl;
    ofs << "branch_diff_y_min: " << BRANCH_DIFF_Y_MIN << std::endl;
    ofs << "branch_diff_y_max: " << BRANCH_DIFF_Y_MAX << std::endl;
 }

void BranchDetection::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(pose_.q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        publishBranch(std::vector<geometry_msgs::Point>(),pose_.data.header.frame_id);
        return;
    }

    static bool initialized = false;
    if(!initialized){
        listener_.waitForTransform(pose_.data.header.frame_id, msg->header.frame_id, ros::Time(), ros::Duration(1.0));
        initialized = true;
    }

    // センサと障害物の距離が近い時は検出を行わない
    for(int t=OBSTACLE_CHECK_ANGLE/msg->angle_increment,i=(msg->ranges.size()/2)-1-t,ie=(msg->ranges.size()/2)+t;i!=ie;++i){
		if(!std::isnan(msg->ranges[i]) && msg->ranges[i] < OBSTACLE_RANGE_MIN){
			// ROS_ERROR_STREAM("May be close to obstacles");
            publishBranch(std::vector<geometry_msgs::Point>(),pose_.data.header.frame_id);
            return;
		}
    }

    // nan のデータを取り除いて 距離と角度のデータに分ける
    // ExpLib::Struct::scanStruct ss(msg->ranges.size(),msg->angle_max);
    ExpLib::Struct::scanStruct ss(msg->ranges.size());

    for(int i=0,e=msg->ranges.size();i!=e;++i){
		if(!std::isnan(msg->ranges[i])){
            float temp = msg->angle_min+(msg->angle_increment*i);
			ss.ranges.emplace_back(msg->ranges[i]);
            ss.x.emplace_back(msg->ranges[i]*cos(temp));
            ss.y.emplace_back(msg->ranges[i]*sin(temp));
			ss.angles.emplace_back(std::move(temp));
		}
    }

    if(ss.ranges.size() < 2){
		ROS_ERROR_STREAM("Scan data is insufficient");
        publishBranch(std::vector<geometry_msgs::Point>(),pose_.data.header.frame_id);
        return;
    }

    // 分岐検出部
    std::vector<geometry_msgs::Point> branches; // 検出した分岐領域を入れる

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	// const float BRANCH_MIN_Y = BRANCH_DIFF_X_MIN*tan(ss.angleMax);//1.0 * tan(0.52) = 0.57
	// const float BRANCH_MAX_Y = BRANCH_MAX_X*tan(ss.angleMax);//5.0 * tan(0.52) = 2.86 //この二つの差が正しいのでは // 6.0/tan(1.05)


    for(int i=0,e=ss.ranges.size()-1;i!=e;++i){
        // double diffX = std::abs(ss.x[i+1] - ss.x[i]);
        // double diffY = std::abs(ss.y[i+1] - ss.y[i]);
		if(ss.angles[i] * ss.angles[i+1] < 0){
            // ROS_INFO_STREAM("Branch continue : 1");
            // ROS_INFO_STREAM("ss.angles[i] : " << ss.angles[i] << ", ss.angles[i+1] : " << ss.angles[i+1] << ", ss.x[i] : " << ss.x[i] << ", ss.x[i+1] : " << ss.x[i+1] << ", ss.y[i] : " << ss.y[i] << ", ss.y[i+1] : " << ss.y[i+1] << ", diffX : " << diffX << ", diffY : " << diffY);
            continue; // 二つの角度の符号が違うときスキップ/
        }
        if(ss.ranges[i] > SCAN_RANGE_THRESHOLD || ss.ranges[i+1] > SCAN_RANGE_THRESHOLD){
            // ROS_INFO_STREAM("Branch continue : 2");
            // ROS_INFO_STREAM("ss.angles[i] : " << ss.angles[i] << ", ss.angles[i+1] : " << ss.angles[i+1] << ", ss.x[i] : " << ss.x[i] << ", ss.x[i+1] : " << ss.x[i+1] << ", ss.y[i] : " << ss.y[i] << ", ss.y[i+1] : " << ss.y[i+1] << ", diffX : " << diffX << ", diffY : " << diffY);
            continue; // 距離が遠いのは信用できないのでだめ
        }
        if(ss.y[i] <= ss.y[i+1]){
            // ROS_INFO_STREAM("Branch continue : 3");
            // ROS_INFO_STREAM("ss.angles[i] : " << ss.angles[i] << ", ss.angles[i+1] : " << ss.angles[i+1] << ", ss.x[i] : " << ss.x[i] << ", ss.x[i+1] : " << ss.x[i+1] << ", ss.y[i] : " << ss.y[i] << ", ss.y[i+1] : " << ss.y[i+1] << ", diffX : " << diffX << ", diffY : " << diffY);
            continue; // yの大きさが i < i+1 だとNG
        }
        if((ss.angles[i] < 0 && ss.x[i] >= ss.x[i+1]) || (ss.angles[i] >= 0 && ss.x[i] <= ss.x[i+1])){
            // ROS_INFO_STREAM("Branch continue : 4");
            // ROS_INFO_STREAM("ss.angles[i] : " << ss.angles[i] << ", ss.angles[i+1] : " << ss.angles[i+1] << ", ss.x[i] : " << ss.x[i] << ", ss.x[i+1] : " << ss.x[i+1] << ", ss.y[i] : " << ss.y[i] << ", ss.y[i+1] : " << ss.y[i+1] << ", diffX : " << diffX << ", diffY : " << diffY);
            continue; // xの差を判定
        }
        
		// if(ss.ranges[i] > SCAN_RANGE_THRESHOLD || ss.ranges[i+1] > SCAN_RANGE_THRESHOLD) continue; //距離が遠いのは信用できないのでだめ
        // if(ss.y[i] >= ss.y[i+1]) continue; // yの大きさが i < i+1 だとNG
        // if((ss.angles[i] < 0 && ss.x[i] >= ss.x[i+1]) || (ss.angles[i] >= 0 && ss.x[i] <= ss.x[i+1])) continue; // xの差を判定
        double diffX = std::abs(ss.x[i+1] - ss.x[i]);
        if(BRANCH_DIFF_X_MIN > diffX || diffX > BRANCH_DIFF_X_MAX){
            // ROS_INFO_STREAM("Branch continue : 5");
            // ROS_INFO_STREAM("ss.angles[i] : " << ss.angles[i] << ", ss.angles[i+1] : " << ss.angles[i+1] << ", ss.x[i] : " << ss.x[i] << ", ss.x[i+1] : " << ss.x[i+1] << ", ss.y[i] : " << ss.y[i] << ", ss.y[i+1] : " << ss.y[i+1] << ", diffX : " << diffX << ", diffY : " << diffY);
            continue; //x座標の差(分岐の幅)が範囲内じゃないと分岐と認めない
        }
        // if(BRANCH_DIFF_X_MIN > diffX || diffX > BRANCH_DIFF_X_MAX) continue; //x座標の差(分岐の幅)が範囲内じゃないと分岐と認めない
        double diffY = std::abs(ss.y[i+1] - ss.y[i]);
        if(BRANCH_DIFF_Y_MIN > diffY || diffY > BRANCH_DIFF_Y_MAX){
            // ROS_INFO_STREAM("Branch continue : 6");
            // ROS_INFO_STREAM("ss.angles[i] : " << ss.angles[i] << ", ss.angles[i+1] : " << ss.angles[i+1] << ", ss.x[i] : " << ss.x[i] << ", ss.x[i+1] : " << ss.x[i+1] << ", ss.y[i] : " << ss.y[i] << ", ss.y[i+1] : " << ss.y[i+1] << ", diffX : " << diffX << ", diffY : " << diffY);
            continue; //y座標の差が範囲内じゃないと分岐と認めない
        }
        // if(BRANCH_DIFF_Y_MIN > diffY || diffY > BRANCH_DIFF_Y_MAX) continue; 
        // 検出した座標をpose座標系に変換して input
        branches.emplace_back(ExpLib::Utility::coordinateConverter2d<geometry_msgs::Point>(listener_, pose_.data.header.frame_id, msg->header.frame_id, ExpLib::Construct::msgPoint((ss.x[i+1] + ss.x[i])/2, (ss.y[i+1] + ss.y[i])/2)));
	}

    ROS_INFO_STREAM("Branch Found : " << branches.size());

    publishBranch(branches,pose_.data.header.frame_id);
}

void BranchDetection::publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId){
    exploration_msgs::PointArray msg;
    msg.points = branches;
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    branch_.pub.publish(msg);
    ROS_INFO_STREAM("Publish branch");
}

#endif // BRANCH_DETECTION_HPP