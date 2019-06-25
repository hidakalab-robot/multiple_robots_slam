#include <ros/ros.h>
#include <exploration_msgs/Frontier.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <exploration/common_lib.hpp>
#include <exploration/path_planning.hpp>
#include <navfn/navfn_ros.h>
#include <geometry_msgs/Vector3.h>
#include <exploration_msgs/RobotInfoArray.h>

class SeamlessHybrid
{
private:
    struct maxValue{
        double distance;
        double angle;
        maxValue(){};
        maxValue(double d, double a):distance(d),angle(a){};
    };

    struct eachValue{
        double distance;
        double angle;
        eachValue(){};
        eachValue(const double d,const double a):distance(d),angle(a){};
    };

    //計算を行う領域ごとに管理した方が楽
    struct areaInfo{ //これのサイズ : 分岐の数 + 1(直進) + 自分以外のロボットの数 = 分岐の数 + ロボットの数
        exploration_msgs::RobotInfo robot;
        std::vector<eachValue> values; //各未探査領域に対する情報 //values.size() == frontiers.size()
        areaInfo(){};
        areaInfo(const std::string& n,const geometry_msgs::Point& p,const Eigen::Vector2d& v):robot(CommonLib::msgRobotInfo(n,p,CommonLib::msgVector(v[0],v[1]))){};
    };

    std::string ROBOT_NAME;
    std::string MAP_FRAME_ID;
	double THROUGH_TOLERANCE;
    double COVARIANCE_THRESHOLD;
    double VARIANCE_THRESHOLD;
    double ANGLE_WEIGHT;
    double PATH_WEIGHT;
    double ROBOT_WEIGHT;

    int SIMULATE_ROBOT_INDEX;

    CommonLib::subStruct<exploration_msgs::RobotInfoArray> *robotArray_;

    std::vector<CommonLib::listStruct> inputBranches;
    std::vector<exploration_msgs::Frontier> inputFrontiers;
    geometry_msgs::Pose pose;

    std::vector<geometry_msgs::Point> branches;
    std::vector<geometry_msgs::Point> frontiers;
    std::vector<exploration_msgs::RobotInfo> robotList;

    maxValue mVal;

    std::vector<areaInfo> mainRobotInfo; // mainRobotInfo.size() == branches.size() + 1
    std::vector<areaInfo> subRobotsInfo; // subRobotInfo.size() == mainRobot 以外の ロボットの数

    PathPlanning<navfn::NavfnROS>* pp_;
    
    static std::vector<geometry_msgs::Point> throughBranches; //一度重複探査を無視して行った座標（二回目は行けない）

public:
    SeamlessHybrid(PathPlanning<navfn::NavfnROS>& pp);
    SeamlessHybrid(const std::vector<CommonLib::listStruct>& b, const std::vector<exploration_msgs::Frontier>& f, const geometry_msgs::Pose& p, PathPlanning<navfn::NavfnROS>& pp, CommonLib::subStruct<exploration_msgs::RobotInfoArray>& ria);
    bool initialize(void);
    bool dataFilter(void);
    void evaluationInitialize(void);
    bool result(geometry_msgs::Point& goal);
    void simulatorFunction(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f);
};

std::vector<geometry_msgs::Point> SeamlessHybrid::throughBranches;

// for simulator constructor
SeamlessHybrid::SeamlessHybrid(PathPlanning<navfn::NavfnROS>& pp){
    pp_ = &pp;
    ros::NodeHandle ph("~");
	ph.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    ph.param<double>("covariance_threshold", COVARIANCE_THRESHOLD, 0.7);
    ph.param<double>("variance_threshold", VARIANCE_THRESHOLD, 1.5);
    ph.param<double>("angle_weight", ANGLE_WEIGHT, 1.5);
    ph.param<double>("path_weight", PATH_WEIGHT, 2.5);
    ph.param<double>("robot_weight", ROBOT_WEIGHT, 1.0); 
    ph.param<int>("simulate_robot_index", SIMULATE_ROBOT_INDEX, 1); 
}

SeamlessHybrid::SeamlessHybrid(const std::vector<CommonLib::listStruct>& b, const std::vector<exploration_msgs::Frontier>& f, const geometry_msgs::Pose& p, PathPlanning<navfn::NavfnROS>& pp, CommonLib::subStruct<exploration_msgs::RobotInfoArray>& ria)
    :inputBranches(b)
    ,inputFrontiers(f)
    ,pose(p)
    ,mVal(-DBL_MAX,-DBL_MAX){

    pp_ = &pp;
    robotArray_ = &ria;    
    ros::NodeHandle ph("~");
    ph.param<std::string>("robot_name", ROBOT_NAME, "robot1");
	ph.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
    ph.param<double>("through_tolerance", THROUGH_TOLERANCE, 1.0);
    ph.param<double>("covariance_threshold", COVARIANCE_THRESHOLD, 0.7);
    ph.param<double>("variance_threshold", VARIANCE_THRESHOLD, 1.5);
    ph.param<double>("angle_weight", ANGLE_WEIGHT, 1.5);
    ph.param<double>("path_weight", PATH_WEIGHT, 2.5);
    ph.param<double>("robot_weight", ROBOT_WEIGHT, 1.0); 
}

bool SeamlessHybrid::initialize(void){
    if(!dataFilter()) {
        ROS_WARN_STREAM("initialize missing !!");
        return false;
    }
    evaluationInitialize();
    return true;
}   

bool SeamlessHybrid::dataFilter(void){

    //分岐領域のフィルタ
    ROS_INFO_STREAM("through branch size : " << throughBranches.size());
    ROS_INFO_STREAM("before branches size : " << inputBranches.size());

    branches.reserve(inputBranches.size());

    for(const auto& i : inputBranches){
        //duplication filter
        if(i.duplication == CommonLib::DuplicationStatus::NEWER){
            ROS_INFO_STREAM("newer duplication!!");
            continue;
        }
        //throught filter
        auto through = [&,this]{
            for(const auto& t : throughBranches) {
                if(Eigen::Vector2d(i.point.x-t.x,i.point.y-t.y).norm() < THROUGH_TOLERANCE) return true;
            }
            return false;
        };
        if(through()){
            ROS_INFO_STREAM("throught branch!!");
            continue;
        }
        branches.emplace_back(i.point);
    }
    
    ROS_INFO_STREAM("after branches size : " << branches.size());

    if(branches.size()==0) return false;

    //フロンティア領域のフィルタ

    ROS_INFO_STREAM("before frontiers size : " << inputFrontiers.size());

    frontiers.reserve(inputFrontiers.size());

    for(const auto& i : inputFrontiers){
        if(i.variance.x>i.variance.y ? i.variance.x : i.variance.y > VARIANCE_THRESHOLD || std::abs(i.covariance) > COVARIANCE_THRESHOLD) frontiers.emplace_back(i.coordinate);
    }

    ROS_INFO_STREAM("after frontiers size : " << frontiers.size());

    if(frontiers.size()==0) return false;

    //ロボットリストのフィルタ
    if(!robotArray_->q.callOne(ros::WallDuration(1))){
        ROS_INFO_STREAM("before robotList size : " << robotArray_->data.list.size());
        for(const auto& r : robotArray_->data.list){
            if(!(r.name == "/" + ROBOT_NAME)) robotList.emplace_back(r);
        }
        ROS_INFO_STREAM("after robotList size : " << robotList.size());
        return true;
    }
    
    return false;
}

void SeamlessHybrid::evaluationInitialize(void){

    auto calc = [this](const geometry_msgs::Point& p, const Eigen::Vector2d& v1, const std::string& name){
        // ROS_DEBUG_STREAM("calc p : (" << p.x << "," << p.y << ")");
        // ROS_DEBUG_STREAM("v1 : (" << v1[0] << "," << v1[1] << ")");
        SeamlessHybrid::areaInfo ai(name,p,v1);
        ai.values.reserve(frontiers.size());
        for(const auto& f : frontiers){
            // 目標地点での向きをpathの最後の方の移動で決めたい
            Eigen::Vector2d v2;
            double distance;
            if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f,MAP_FRAME_ID),distance,v2)){
                v2 = Eigen::Vector2d(f.x - p.x, f.y - p.y).normalized();
                if(!pp_->getDistance(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f,MAP_FRAME_ID),distance)){
                    //最終手段で直線距離を計算
                    distance = Eigen::Vector2d(f.x - p.x, f.y - p.y).norm();
                }                
            }

            // ROS_DEBUG_STREAM("v2 : (" << v2[0] << "," << v2[1] << ")");

            double angle = std::abs(acos(v1.dot(v2)));

            // ROS_INFO_STREAM("distance to frontier[m]: " << distance);
            // ROS_INFO_STREAM("angle to frontier[deg]: " << (angle*180)/M_PI);

            ai.values.emplace_back(eachValue(distance,angle));

            if(angle > mVal.angle) mVal.angle = std::move(angle);
            if(distance > mVal.distance) mVal.distance = std::move(distance);
        }
        return ai;
    };

    mainRobotInfo.reserve(branches.size()+1);
    subRobotsInfo.reserve(robotList.size());

    double forward = 0;
    for(const auto& b : branches){
        double distance;
        Eigen::Vector2d v1;
        if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(pose.position,MAP_FRAME_ID),CommonLib::pointToPoseStamped(b,MAP_FRAME_ID),distance,v1)){
            v1 = Eigen::Vector2d(b.x - pose.position.x, b.y - pose.position.y);
            distance = v1.lpNorm<1>();
            v1.normalize();
        }
        // ROS_INFO_STREAM("distance to branch[m]: " << distance);
        mainRobotInfo.emplace_back(calc(b,v1,ROBOT_NAME));
        forward += distance;
    }
    forward /= branches.size();

    // ROS_INFO_STREAM("forward: " << forward);

    //直進時の計算
    double yaw = CommonLib::qToYaw(pose.orientation);
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);
    // ROS_INFO_STREAM("yaw: " << yaw << ", cos: " << cosYaw << ", sin:" << sinYaw);
    mainRobotInfo.emplace_back(calc(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw),ROBOT_NAME));

    //他のロボットに関する計算
    // forward する版
    // for(const auto& r : robotList) subRobotsInfo.emplace_back(calc(CommonLib::msgPoint(r.coordinate.x+forward*r.vector.x,r.coordinate.y+forward*r.vector.y),CommonLib::msgVectorToVector2d(r.vector),r.name));
    // forward しない番
    for(const auto& r : robotList) subRobotsInfo.emplace_back(calc(CommonLib::msgPoint(r.coordinate.x,r.coordinate.y),CommonLib::msgVectorToVector2d(r.vector),r.name));
}

bool SeamlessHybrid::result(geometry_msgs::Point& goal){
    // mainRobotInfoとsubRobotInfoが必要

    ROS_DEBUG_STREAM("result");

    auto evaluation = [this](const double d, const double a){return PATH_WEIGHT * d / mVal.distance + ANGLE_WEIGHT * a / mVal.angle;};

    double minE = DBL_MAX;

    for(int m=0,me=mainRobotInfo.size();m!=me;++m){
        double e = 1;
        for(int i=0,ie=mainRobotInfo[m].values.size();i!=ie;++i){
            double subE = 0;
            if(subRobotsInfo.size()==0) subE = DBL_MAX;
            else for(const auto& s : subRobotsInfo) subE += evaluation(s.values[i].distance, s.values[i].angle);
            e *= evaluation(mainRobotInfo[m].values[i].distance, mainRobotInfo[m].values[i].angle) + (ROBOT_WEIGHT/subE);
        }
        ROS_DEBUG_STREAM("position : (" << mainRobotInfo[m].robot.coordinate.x << "," << mainRobotInfo[m].robot.coordinate.y << "), sum : " << e);
        if(e < minE){
            minE = std::move(e);
            goal = mainRobotInfo[m].robot.coordinate;
            if(m == me -1) return false;
        }
    }
    throughBranches.emplace_back(goal);
    return true;
}

void SeamlessHybrid::simulatorFunction(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f){
    // ロボットの姿勢、分岐領域の座標、フロンティアの座標が入力されたときに分岐領域にどちらに行くかの判定を行う
    static PathPlanning<navfn::NavfnROS> pSim("simulator_goal_costmap","simulator_goal_path");

    //ここで評価の重みをリロード
    ros::NodeHandle p("~");
    p.param<double>("angle_weight", ANGLE_WEIGHT, 1.5);
    p.param<double>("path_weight", PATH_WEIGHT, 2.5);
    p.param<double>("robot_weight", ROBOT_WEIGHT, 1.0); 
    p.param<int>("simulate_robot_index", SIMULATE_ROBOT_INDEX, 1); 

    if(SIMULATE_ROBOT_INDEX > r.size()) SIMULATE_ROBOT_INDEX = 1;

    // vector reset
    mainRobotInfo = std::vector<areaInfo>();
    subRobotsInfo = std::vector<areaInfo>();

    mVal.angle = mVal.distance = -DBL_MAX;
    pose = r[SIMULATE_ROBOT_INDEX-1]; // ロボット1が分岐領域を見つけた設定
    ROBOT_NAME = "/robot"+std::to_string(SIMULATE_ROBOT_INDEX);
    branches = b;
    frontiers = f;
    // robotList用にデータを整形する r の2番めから最後までを整形して代入
    robotList.resize(r.size()-1);
    
    for(int i=0,ie=r.size(),index=0;i!=ie;++i){
        if(i+1 == SIMULATE_ROBOT_INDEX) continue;
        double yaw = CommonLib::qToYaw(r[i].orientation);
        robotList[index++] = CommonLib::msgRobotInfo("/robot"+std::to_string(i+1),r[i].position,CommonLib::msgVector(cos(yaw),sin(yaw)));
    }
    evaluationInitialize();
    geometry_msgs::Point goal;
    
    result(goal);

    pSim.createPath(CommonLib::poseToPoseStamped(pose,MAP_FRAME_ID),CommonLib::pointToPoseStamped(goal,MAP_FRAME_ID));
    
}