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
    // struct sumValue{
    //     double sumDistnance;
    //     double sumAngle;
    //     geometry_msgs::Point coordinate;
    // };

    struct maxValue{
        double distance;
        double angle;
        maxValue(double d, double a):distance(d),angle(a){};
    };

    struct eachValue{
        double distance;
        double angle;
        eachValue(){};
        eachValue(const double d,const double a):distance(d),angle(a){};
    };

    // struct robotInfo{
    //     std::string name;
    //     geometry_msgs::Point coordinate;
    //     geometry_msgs::Vector3 vector;
    //     robotInfo(){};
    //     robotInfo(const std::string& n, const geometry_msgs::Point& p, const double x, const double y):name(n),coordinate(p),vector(CommonLib::msgVector(x,y)){};
    // };

    //計算を行う領域ごとに管理した方が楽
    struct areaInfo{ //これのサイズ : 分岐の数 + 1(直進) + 自分以外のロボットの数 = 分岐の数 + ロボットの数
        // geometry_msgs::PoseStamped pose; // 対象領域でのロボットのポーズ etc. 分岐領域でのポーズ, 直進方向のポーズ, メイン以外のロボットのポーズ
        // geometry_msgs::Point coordinate; //領域の座標
        exploration_msgs::RobotInfo robot;
        std::vector<eachValue> values; //各未探査領域に対する情報 //values.size() == frontiers.size()
        //sumValueTwo sumValue; //各未探査領域に対する情報の和(ロボットが一台のとき用)//計算できるので要らないかも
        areaInfo(){};
        areaInfo(const std::string& n,const geometry_msgs::Point& p,const Eigen::Vector2d& v):robot(CommonLib::msgRobotInfo(n,p,CommonLib::msgVector(v[0],v[1]))){};
    };

    std::string ROBOT_NAME;
    std::string MAP_FRAME_ID;
    // bool MULTI_EXPLORATION_MODE; //一台でもトピックが存在するようにするのでいらない
	double THROUGH_TOLERANCE;
    double COVARIANCE_THRESHOLD;
    double VARIANCE_THRESHOLD;
    double ANGLE_WEIGHT;
    double PATH_WEIGHT;
    double ROBOT_WEIGHT;

    std::vector<CommonLib::listStruct> inputBranches;
    std::vector<exploration_msgs::Frontier> frontiers;
    geometry_msgs::Pose pose;

    std::vector<geometry_msgs::Point> branches;

    // std::vector<sumValue> sVal;
    maxValue mVal;

    CommonLib::subStruct<exploration_msgs::RobotInfoArray> *robotArray_; //これは違うところでコールしたほうがいいかも // このクラスは使い捨てにしたいので

    //eachValを領域の個数とかに分けて作成しないと行けない
    std::vector<areaInfo> mainRobotInfo; // mainRobotInfo.size() == branches.size() + 1
    // std::vector<robotPose> mainRobotPoses;   // mainRobotPoses.size() == branches.size() + 1 //分岐領域の座標と直進方向のポーズ
    std::vector<areaInfo> subRobotsInfo; // subRobotInfo.size() == mainRobot 以外の ロボットの数
    // std::vector<robotPose> subRobotPoses;   // subrobotPoses.size() == mainRobot 以外の ロボットの数 = robotList.size()-1 // 各ロボットのポーズ // フィルタしたあとはrobotList.size()


    // std::vector<explation> robotList; // 外部から取得する, 全ロボットのポーズ

    PathPlanning<navfn::NavfnROS>* pp_;
    
    static std::vector<geometry_msgs::Point> throughBranches; //一度重複探査を無視して行った座標（二回目は行けない）

public:
    SeamlessHybrid(const std::vector<CommonLib::listStruct>& b, const std::vector<exploration_msgs::Frontier>& f, const geometry_msgs::Pose& p, PathPlanning<navfn::NavfnROS>& pp, CommonLib::subStruct<exploration_msgs::RobotInfoArray>& ria);
    bool initialize(void);
    bool dataFilter(void);
    // bool dataFilterMulti(void);
    void evaluationInitialize(void);
    // void evaluationInitializeMulti(void);
    bool result(geometry_msgs::Point& goal);
    // bool resultMulti(geometry_msgs::Point& goal);
};

std::vector<geometry_msgs::Point> SeamlessHybrid::throughBranches;

SeamlessHybrid::SeamlessHybrid(const std::vector<CommonLib::listStruct>& b, const std::vector<exploration_msgs::Frontier>& f, const geometry_msgs::Pose& p, PathPlanning<navfn::NavfnROS>& pp, CommonLib::subStruct<exploration_msgs::RobotInfoArray>& ria)
    :inputBranches(b)
    ,frontiers(f)
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
    // ph.param<double>("multi_exploration_mode", MULTI_EXPLORATION_MODE, false); 
}

bool SeamlessHybrid::initialize(void){
    if(!dataFilter()) {
        ROS_WARN_STREAM("initialize missing !!");
        return false;
    }
    evaluationInitialize();
    return true;
}

// bool SeamlessHybrid::dataFilter(void){

//     ROS_INFO_STREAM("before frontiers size : " << frontiers.size());
//     auto eraseIndex = std::remove_if(frontiers.begin(), frontiers.end(),[this](exploration_msgs::Frontier& f){
//         double v = f.variance.x>f.variance.y ? f.variance.x : f.variance.y;
//         return v < VARIANCE_THRESHOLD && std::abs(f.covariance) < COVARIANCE_THRESHOLD;
//     });
//     frontiers.erase(eraseIndex,frontiers.end());

//     ROS_INFO_STREAM("after frontiers size : " << frontiers.size());

//     if(frontiers.size()==0) return false;

//     ROS_INFO_STREAM("through branch size : " << throughBranches.size());

//     ROS_INFO_STREAM("before branches size : " << inputBranches.size());

//     branches.reserve(inputBranches.size());

//     for(const auto& i : inputBranches){
//         //duplication filter
//         if(i.duplication == CommonLib::DuplicationStatus::NEWER){
//             ROS_INFO_STREAM("newer duplication!!");
//             continue;
//         }
//         //throught filter
//         auto through = [&,this]{
//             for(const auto& t : throughBranches) {
//                 if(Eigen::Vector2d(i.point.x-t.x,i.point.y-t.y).norm() < THROUGH_TOLERANCE) return true;
//             }
//             return false;
//         };
//         if(through()){
//             ROS_INFO_STREAM("throught branch!!");
//             continue;
//         }
//         branches.emplace_back(i.point);
//     }
//     //filteredListとfrontiers
//     ROS_INFO_STREAM("after branches size : " << branches.size());

//     return branches.size()==0 ? false : true;
// }

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

    ROS_INFO_STREAM("before frontiers size : " << frontiers.size());
    auto eraseFrontierIndex = std::remove_if(frontiers.begin(), frontiers.end(),[this](exploration_msgs::Frontier& f){
        double v = f.variance.x>f.variance.y ? f.variance.x : f.variance.y;
        return v < VARIANCE_THRESHOLD && std::abs(f.covariance) < COVARIANCE_THRESHOLD;
    });
    frontiers.erase(eraseFrontierIndex,frontiers.end());

    ROS_INFO_STREAM("after frontiers size : " << frontiers.size());

    if(frontiers.size()==0) return false;

    //ロボットリストのフィルタ
    
    // if(MULTI_EXPLORATION_MODE && !robotArray_.q.callOne(ros::WallDuration(1))){
    if(!robotArray_->q.callOne(ros::WallDuration(1))){
        ROS_INFO_STREAM("before robotList size : " << robotArray_->data.list.size());
        auto eraseRobotIndex = std::remove_if(robotArray_->data.list.begin(), robotArray_->data.list.end(),[this](exploration_msgs::RobotInfo& r){return r.name == ROBOT_NAME;});
        robotArray_->data.list.erase(eraseRobotIndex,robotArray_->data.list.end());
        ROS_INFO_STREAM("after robotList size : " << robotArray_->data.list.size());
    }    
    

    return true;
}

// void SeamlessHybrid::evaluationInitialize(void){
//     auto calc = [this](const geometry_msgs::Point& p, const Eigen::Vector2d& v1){
//         ROS_DEBUG_STREAM("calc p : (" << p.x << "," << p.y << ")");
//         ROS_DEBUG_STREAM("v1 : (" << v1[0] << "," << v1[1] << ")");
//         sumValue s{0,0,p};
//         for(const auto& f : frontiers){
//             // 目標地点での向きをpathの最後の方の移動で決めたい
//             Eigen::Vector2d v2;
//             double distance;
//             if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f.coordinate,MAP_FRAME_ID),distance,v2)){
//                 v2 = Eigen::Vector2d(f.coordinate.x - p.x, f.coordinate.y - p.y).normalized();
//                 distance = pp_->getDistance(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f.coordinate,MAP_FRAME_ID));
//             }

//             ROS_DEBUG_STREAM("v2 : (" << v2[0] << "," << v2[1] << ")");

//             double angle = std::abs(acos(v1.dot(v2)));

//             ROS_INFO_STREAM("distance to frontier[m]: " << distance);
//             ROS_INFO_STREAM("angle to frontier[deg]: " << (angle*180)/M_PI);

//             s.sumAngle += angle;
//             s.sumDistnance += distance;

//             if(angle > mVal.angle) mVal.angle = std::move(angle);
//             if(distance > mVal.distance) mVal.distance = std::move(distance);
//         }
//         return s;
//     };

//     sVal.reserve(branches.size()+1);

//     double forward = 0;
//     for(const auto& b : branches){
//         double distance;
//         Eigen::Vector2d v1;
//         // if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(pose.pose.position,MAP_FRAME_ID),CommonLib::pointToPoseStamped(b,MAP_FRAME_ID),distance,v1)){
//         if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(pose.position,MAP_FRAME_ID),CommonLib::pointToPoseStamped(b,MAP_FRAME_ID),distance,v1)){
//             v1 = Eigen::Vector2d(b.x - pose.position.x, b.y - pose.position.y);
//             // forward += v1.norm();
//             distance = v1.lpNorm<1>();
//             v1.normalize();
//         }
//         ROS_INFO_STREAM("distance to branch[m]: " << distance);
//         sVal.emplace_back(calc(b,v1));
//         forward += distance;
//     }
//     forward /= branches.size();

//     ROS_INFO_STREAM("forward: " << forward);

//     //直進時の計算
//     double yaw = CommonLib::qToYaw(pose.orientation);
//     double cosYaw = cos(yaw);
//     double sinYaw = sin(yaw);

//     ROS_INFO_STREAM("yaw: " << yaw << ", cos: " << cosYaw << ", sin:" << sinYaw);

//     sVal.emplace_back(calc(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw)));
// }

void SeamlessHybrid::evaluationInitialize(void){

    auto calc = [this](const geometry_msgs::Point& p, const Eigen::Vector2d& v1, const std::string& name){
        ROS_DEBUG_STREAM("calc p : (" << p.x << "," << p.y << ")");
        ROS_DEBUG_STREAM("v1 : (" << v1[0] << "," << v1[1] << ")");
        // sumValue s{0,0,p};
        SeamlessHybrid::areaInfo ai(name,p,v1);
        ai.values.reserve(frontiers.size());
        for(const auto& f : frontiers){
            // 目標地点での向きをpathの最後の方の移動で決めたい
            Eigen::Vector2d v2;
            double distance;
            if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f.coordinate,MAP_FRAME_ID),distance,v2)){
                v2 = Eigen::Vector2d(f.coordinate.x - p.x, f.coordinate.y - p.y).normalized();
                distance = pp_->getDistance(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f.coordinate,MAP_FRAME_ID));
            }

            ROS_DEBUG_STREAM("v2 : (" << v2[0] << "," << v2[1] << ")");

            double angle = std::abs(acos(v1.dot(v2)));

            ROS_INFO_STREAM("distance to frontier[m]: " << distance);
            ROS_INFO_STREAM("angle to frontier[deg]: " << (angle*180)/M_PI);

            ai.values.emplace_back(eachValue(distance,angle));

            // s.sumAngle += angle;
            // s.sumDistnance += distance;

            if(angle > mVal.angle) mVal.angle = std::move(angle);
            if(distance > mVal.distance) mVal.distance = std::move(distance);
        }
        return ai;
    };

    // 事前準備

    // struct eachValue{
    //     double distance;
    //     double angle;
    // };

    // struct robotInfo{
    //     std::string name;
    //     geometry_msgs::Point coordinate;
    //     geometry_msgs::Vector3 vector
    // };

    //計算を行う領域ごとに管理した方が楽
    // struct areaInfo{ //これのサイズ : 分岐の数 + 1(直進) + 自分以外のロボットの数 = 分岐の数 + ロボットの数
    //     robotInfo robot;// 対象領域でのロボットのポーズ etc. 分岐領域でのポーズ, 直進方向のポーズ, メイン以外のロボットのポーズ
    //     std::vector<eachValue> values; //各未探査領域に対する情報 //values.size() == frontiers.size()
    // };

    mainRobotInfo.reserve(branches.size()+1);
    // for(auto&& i : mainRobotInfo) i.values.reserve(frontiers.size());
    subRobotsInfo.reserve(robotArray_->data.list.size());
    // for(auto&& i : subRobotsInfo) i.values.reserve(frontiers.size());

    // sVal.reserve(branches.size()+1);

    double forward = 0;
    for(const auto& b : branches){
        double distance;
        Eigen::Vector2d v1;
        if(!pp_->getDistanceAndVec(CommonLib::pointToPoseStamped(pose.position,MAP_FRAME_ID),CommonLib::pointToPoseStamped(b,MAP_FRAME_ID),distance,v1)){
            v1 = Eigen::Vector2d(b.x - pose.position.x, b.y - pose.position.y);
            // forward += v1.norm();
            distance = v1.lpNorm<1>();
            v1.normalize();
        }
        ROS_INFO_STREAM("distance to branch[m]: " << distance);
        mainRobotInfo.emplace_back(calc(b,v1,ROBOT_NAME));
        forward += distance;
    }
    forward /= branches.size();

    ROS_INFO_STREAM("forward: " << forward);

    //直進時の計算
    double yaw = CommonLib::qToYaw(pose.orientation);
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);
    ROS_INFO_STREAM("yaw: " << yaw << ", cos: " << cosYaw << ", sin:" << sinYaw);
    mainRobotInfo.emplace_back(calc(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw),ROBOT_NAME));

    //他のロボットに関する計算
    //このへんでコールしたい // forwardつける
    for(const auto& r : robotArray_->data.list) subRobotsInfo.emplace_back(calc(CommonLib::msgPoint(r.coordinate.x+forward*r.vector.x,r.coordinate.y+forward*r.vector.y),CommonLib::msgVectorToVector2d(r.vector),r.name));

}

// bool SeamlessHybrid::result(geometry_msgs::Point& goal){
//     ROS_DEBUG_STREAM("sVal size : " << sVal.size());

//     double minE = DBL_MAX;

//     for(int i=0,ie=sVal.size();i!=ie;++i){
//         double e = PATH_WEIGHT * sVal[i].sumDistnance / mVal.distance + ANGLE_WEIGHT * sVal[i].sumAngle / mVal.angle;
//         ROS_DEBUG_STREAM("position : (" << sVal[i].coordinate.x << "," << sVal[i].coordinate.y << "), sum : " << e);
//         if(e < minE){
//             if(i == ie-1) return false;
//             minE = std::move(e);
//             goal = sVal[i].coordinate;
//         }
//     }
//     throughBranches.emplace_back(goal);
//     return true;
// }

bool SeamlessHybrid::result(geometry_msgs::Point& goal){

    auto evaluation = [this](const double d, const double a){return PATH_WEIGHT * d / mVal.distance + ANGLE_WEIGHT * a / mVal.angle;};

    double minE = DBL_MAX;

    for(int m=0,me=mainRobotInfo.size();m!=me;++m){
        double e = 1;
        for(int i=0,ie=mainRobotInfo[m].values.size();i!=ie;++i){
            double subE = 0;
            if(subRobotsInfo.size()==0) subE = DBL_MAX;
            else {
                for(const auto& s : subRobotsInfo) subE += evaluation(s.values[i].distance, s.values[i].angle);
            }
            e *= evaluation(mainRobotInfo[m].values[i].distance, mainRobotInfo[m].values[i].angle) + (ROBOT_WEIGHT/subE);
        }
        if(e < minE){
            if(m == me -1) return false;
            minE = std::move(e);
            goal = mainRobotInfo[m].robot.coordinate;
        }
    }
    throughBranches.emplace_back(goal);
    return true;
}