#include <exploration/movement.h>
#include <actionlib/client/simple_action_client.h>
#include <exploration_libraly/convert.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/utility.h>
#include <Eigen/Geometry>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <exploration/movement_parameter_reconfigureConfig.h>
#include <exploration_libraly/struct.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <exploration_libraly/path_planning.h>
#include <navfn/navfn_ros.h>
#include <exploration_msgs/AvoidanceStatus.h>

namespace ExStc = ExpLib::Struct;
namespace ExUtl = ExpLib::Utility;
namespace ExCos = ExpLib::Construct;
namespace ExCov = ExpLib::Convert;

Movement::Movement()
    :scan_(new ExStc::subStruct<sensor_msgs::LaserScan>("scan",1)) // sub
    ,pose_(new ExStc::subStruct<geometry_msgs::PoseStamped>("pose",1)) // sub
    ,bumper_(new ExStc::subStruct<kobuki_msgs::BumperEvent>("bumper",1)) // sub
    ,velocity_(new ExStc::pubStruct<geometry_msgs::Twist>("velocity", 1)) //. pub
    ,previousOrientation_(1.0)
    ,pp_(new ExpLib::PathPlanning<navfn::NavfnROS>("movement_costmap","movement_planner")) //クラス名
    ,goal_(new ExStc::pubStruct<geometry_msgs::PointStamped>("goal", 1, true)) // pub
    ,road_(new ExStc::pubStruct<geometry_msgs::PointStamped>("road", 1)) // pub
    ,gCostmap_(new ExStc::subStruct<nav_msgs::OccupancyGrid>("global_costmap",1)) // pub
    ,avoStatus_(new ExStc::pubStruct<exploration_msgs::AvoidanceStatus>("movement_status",1))
    ,drs_(new dynamic_reconfigure::Server<exploration::movement_parameter_reconfigureConfig>(ros::NodeHandle("~/movement"))){
    loadParams();
    drs_->setCallback(boost::bind(&Movement::dynamicParamsCB,this, _1, _2));
}

Movement::~Movement(){
    if(OUTPUT_MOVEMENT_PARAMETERS) outputParams();
}

void Movement::moveToGoal(geometry_msgs::PointStamped goal,bool sleep){
    static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(MOVEBASE_NAME, true);
    
    while(!ac.waitForServer(ros::Duration(1.0)) && ros::ok()) ROS_INFO_STREAM("wait for action server << " << MOVEBASE_NAME);

    if(pose_->q.callOne(ros::WallDuration(1.0))) return;

    if(pose_->data.header.frame_id != goal.header.frame_id){
        static bool initialized = false;
        static tf::TransformListener listener;
        if(!initialized){
            listener.waitForTransform(pose_->data.header.frame_id, goal.header.frame_id, ros::Time(), ros::Duration(1.0));
            initialized = true;
        }
        ExUtl::coordinateConverter2d<void>(listener, pose_->data.header.frame_id, goal.header.frame_id, goal.point);
    }

    if(lookupCostmap(pose_->data)){
        escapeFromCostmap(pose_->data);
        pose_->q.callOne(ros::WallDuration(1.0));
    } 

    move_base_msgs::MoveBaseGoal mbg;
    mbg.target_pose.header.frame_id = pose_->data.header.frame_id;
    mbg.target_pose.header.stamp = ros::Time::now();

    // 目標での姿勢
    Eigen::Vector2d startToGoal;
    // if(USE_ANGLE_BIAS || !pp_->getVec(pose_->data,ExCov::pointStampedToPoseStamped(goal),startToGoal)){
    //     // pathが取得できなかった場合の回転角度の補正値
    //     double yaw = ExCov::qToYaw(pose_->data.pose.orientation);
    //     Eigen::Vector3d cross = Eigen::Vector3d(cos(yaw),sin(yaw),0.0).normalized().cross(Eigen::Vector3d(goal.point.x-pose_->data.pose.position.x,goal.point.y-pose_->data.pose.position.y,0.0).normalized());
    //     double rotateTheta = ANGLE_BIAS * M_PI/180 * (cross.z() > 0 ? 1.0 : cross.z() < 0 ? -1.0 : 0);
    //     Eigen::Matrix2d rotation = ExCos::eigenMat2d(cos(rotateTheta),-sin(rotateTheta),sin(rotateTheta),cos(rotateTheta));
    //     // rotation << cos(rotateTheta), -sin(rotateTheta), sin(rotateTheta), cos(rotateTheta);
    //     startToGoal = rotation * Eigen::Vector2d(goal.point.x-pose_->data.pose.position.x,goal.point.y-pose_->data.pose.position.y);
    // }
    if(!pp_->getVec(pose_->data,ExCov::pointStampedToPoseStamped(goal),startToGoal)){
        startToGoal = Eigen::Vector2d(goal.point.x-pose_->data.pose.position.x,goal.point.y-pose_->data.pose.position.y);
    }

    if(USE_ANGLE_BIAS){
        double yaw = ExCov::qToYaw(pose_->data.pose.orientation);
        // Eigen::Vector3d cross = Eigen::Vector3d(cos(yaw),sin(yaw),0.0).normalized().cross(Eigen::Vector3d(goal.point.x-pose_->data.pose.position.x,goal.point.y-pose_->data.pose.position.y,0.0).normalized());
        Eigen::Vector3d cross = Eigen::Vector3d(cos(yaw),sin(yaw),0.0).normalized().cross(Eigen::Vector3d(startToGoal.x(),startToGoal.y(),0.0).normalized());
        double rotateTheta = ANGLE_BIAS * M_PI/180 * (cross.z() > 0 ? 1.0 : cross.z() < 0 ? -1.0 : 0);
        Eigen::Matrix2d rotation = ExCos::eigenMat2d(cos(rotateTheta),-sin(rotateTheta),sin(rotateTheta),cos(rotateTheta));
        startToGoal = rotation * startToGoal;
    }

    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),Eigen::Vector3d(startToGoal.x(),startToGoal.y(),0.0));
    mbg.target_pose.pose = ExCos::msgPose(goal.point,ExCov::eigenQuaToGeoQua(q));

    ROS_DEBUG_STREAM("goal pose : " << mbg.target_pose.pose);
    ROS_DEBUG_STREAM("goal yaw : " << ExCov::qToYaw(mbg.target_pose.pose.orientation));
    ROS_INFO_STREAM("send goal to move_base");
    ac.sendGoal(mbg);
    ROS_INFO_STREAM("wait for result");

    if(sleep){
        // sleep
        ros::Time start = ros::Time::now();
        double delay = 10.0;
        while(ros::Duration(ros::Time::now() - start).toSec()<delay){};
    }
    else{
        ros::Rate rate(GOAL_RESET_RATE);

        while(!ac.getState().isDone() && ros::ok()){
            publishMovementStatus("move_base");
            if(lookupCostmap(mbg.target_pose)){ //コストマップに被っているばあい
                // 目的地を再設定
                if(!resetGoal(mbg.target_pose)){ 
                    ROS_INFO_STREAM("current goal is canceled");
                    ac.cancelGoal(); //リセット出来ないばあいは目標をキャンセ留守る
                    ac.waitForResult();
                    break;
                    // return;
                }
                if(ac.getState().isDone()) break;
                // 大丈夫な目的地に変わっているので再設定
                ROS_INFO_STREAM("set a new goal pose : " << mbg.target_pose.pose);
                ROS_DEBUG_STREAM("new goal yaw : " << ExCov::qToYaw(mbg.target_pose.pose.orientation));
                ROS_INFO_STREAM("send new goal to move_base");
                ac.sendGoal(mbg);
                // ゴールtopicに再出力
                goal_->pub.publish(ExCov::poseStampedToPointStamped(mbg.target_pose));
                ROS_INFO_STREAM("wait for result");
            }
            rate.sleep();
        }

        ROS_INFO_STREAM("move_base was finished");
        ROS_INFO_STREAM((ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ? "I Reached Given Target" : "I did not Reach Given Target"));
    }
 }

void Movement::moveToForward(void){
    ROS_INFO_STREAM("Moving Straight");

    if(!bumper_->q.callOne(ros::WallDuration(1)) && bumperCollision(bumper_->data)) return; // 障害物に接触してないか確認
    
    if(pose_->q.callOne(ros::WallDuration(1))) return;

    if(lookupCostmap(pose_->data)){
        escapeFromCostmap(pose_->data);
        pose_->q.callOne(ros::WallDuration(1.0));
    } 

    if(scan_->q.callOne(ros::WallDuration(1))) return;

    if(APPROACH_WALL){
        double angle;
        if(forwardWallDetection(scan_->data, angle)) VFHMove(scan_->data,std::move(angle));
        else if(!roadCenterDetection(scan_->data)){
            if(!VFHMove(scan_->data)) emergencyAvoidance(scan_->data);
        }
    }
    else {
        if(!roadCenterDetection(scan_->data)){
            if(!VFHMove(scan_->data)) emergencyAvoidance(scan_->data);
        }
    }
}

void Movement::oneRotation(void){
    //ロボットがz軸周りに一回転する
    ROS_DEBUG_STREAM("rotation");

    if(pose_->q.callOne(ros::WallDuration(1))) return;

    double initYaw,yaw = ExCov::qToYaw(pose_->data.pose.orientation);
    double initSign = initYaw / std::abs(initYaw);

    if(std::isnan(initSign)) initSign = 1.0;

    //initYawが+の時は+回転
    //initYawが-の時は-回転
    geometry_msgs::Twist vel = ExCos::msgTwist(0,initSign * ROTATION_VELOCITY);
    
    for(int count=0;(count < 3 && (count < 2 || std::abs(yaw) < std::abs(initYaw))) && ros::ok();){
        double yawOld = yaw;
        velocity_->pub.publish(vel);
        pose_->q.callOne(ros::WallDuration(1));
        yaw = ExCov::qToYaw(pose_->data.pose.orientation);
        if(yawOld * yaw < 0) ++count;
    }
}

bool Movement::lookupCostmap(void){
    while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
    return lookupCostmap(pose_->data);
}

bool Movement::lookupCostmap(const geometry_msgs::PoseStamped& goal){
    // コストマップを更新
    while(gCostmap_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting global costmap ...");
    ROS_INFO_STREAM("get global costmap");
    return lookupCostmap(goal,gCostmap_->data);
}

bool Movement::lookupCostmap(const geometry_msgs::PoseStamped& goal, const nav_msgs::OccupancyGrid& map){
    // true:被ってる, false:被ってない
    ROS_INFO_STREAM("lookup global costmap");
    // コストマップの配列を二次元に変換
    std::vector<std::vector<int8_t>> lmap(ExUtl::mapArray1dTo2d(map.data,map.info));
    ExStc::mapSearchWindow msw(goal.pose.position,map.info,COSTMAP_MARGIN);
    for(int y=msw.top,ey=msw.bottom+1;y!=ey;++y){
        for(int x=msw.left,ex=msw.right+1;x!=ex;++x){
            // if(lmap[x][y] > 0){
            if(lmap[x][y] > 98){
                ROS_INFO_STREAM("current goal is over the costmap !!");
                return true; //被ってたら終了
            }
        }
    }
    ROS_INFO_STREAM("this goal is ok");
    return false;
}

void Movement::escapeFromCostmap(const geometry_msgs::PoseStamped& pose){
    // 目標設定前に足元にコストマップあったら外に出るようにする
    // true: 脱出成功, false: 脱出不可
    // ローカルコストマップを分割して安全そうなエリアに向かって脱出

    // // コストマップ
    while(gCostmap_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting global costmap ...");
    ROS_INFO_STREAM("get global costmap");
    std::vector<std::vector<int8_t>> gMap(ExUtl::mapArray1dTo2d(gCostmap_->data.data,gCostmap_->data.info));

    ExStc::mapSearchWindow msw(pose.pose.position,gCostmap_->data.info,ESC_MAP_WIDTH,ESC_MAP_HEIGHT);

    const int gw = (msw.right-msw.left+1) / ESC_MAP_DIV_X;
    const int gh = (msw.bottom-msw.top+1) / ESC_MAP_DIV_Y;

    struct escMap{
        Eigen::Vector2i cIndex; //中心のいんでっくす
        geometry_msgs::Pose pose;
        double risk; // コストマップの影響度
        double grad;
    };

    std::vector<std::vector<escMap>> gmm(ESC_MAP_DIV_X,std::vector<escMap>(ESC_MAP_DIV_Y));

    for(int dh=0, dhe=ESC_MAP_DIV_Y; dh!=dhe; ++dh){
        for(int dw=0, dwe=ESC_MAP_DIV_X; dw!=dwe; ++dw){
            double risk = 0;
            for(int h=msw.top+dh*gh,he=msw.top+(dh+1)*gh;h!=he;++h){
                for(int w=msw.left+dw*gw,we=msw.left+(dw+1)*gw;w!=we;++w) risk += gMap[w][h] >= 99 ? gMap[w][h] : 0;
            }
            gmm[dw][dh].cIndex = Eigen::Vector2i((msw.left*2+(2*dw+1)*gw)/2,(msw.top*2+(2*dh+1)*gh)/2);
            gmm[dw][dh].pose.position = ExUtl::mapIndexToCoordinate(gmm[dw][dh].cIndex.x(),gmm[dw][dh].cIndex.y(),gCostmap_->data.info);
            gmm[dw][dh].risk = risk / (gw*gh);
        }
    }

    Eigen::Vector2i mci(ESC_MAP_DIV_X/2,ESC_MAP_DIV_Y/2);
    
    for(int y=ESC_MAP_DIV_Y-1;y!=-1;--y){
        for(int x=0;x!=ESC_MAP_DIV_X;++x){
            gmm[x][y].grad = x!=mci.x()||y!=mci.y() ? gmm[x][y].risk - gmm[mci.x()][mci.y()].risk : 100;
            gmm[x][y].pose.orientation = ExCov::eigenQuaToGeoQua(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),Eigen::Vector3d(gmm[x][y].pose.position.x-gmm[mci.x()][mci.y()].pose.position.x,gmm[x][y].pose.position.y-gmm[mci.x()][mci.y()].pose.position.y,0.0))); 
        } 
    }
    
    Eigen::Quaterniond cAng = ExCov::geoQuaToEigenQua(pose.pose.orientation);

    double minad = DBL_MAX;
    double mingr = DBL_MAX;
    Eigen::Vector2i escIndex = mci; //回避する方向のインデックス
    for(int y=ESC_MAP_DIV_Y-1;y!=-1;--y){
        for(int x=0;x!=ESC_MAP_DIV_X;++x){
            if(gmm[x][y].grad <= mingr){
                double tempad = cAng.angularDistance(ExCov::geoQuaToEigenQua(gmm[x][y].pose.orientation));
                if(gmm[x][y].grad != mingr || (gmm[x][y].grad == mingr && tempad <= minad)){
                    minad = tempad;
                    mingr = gmm[x][y].grad;
                    escIndex << x,y;
                }
            }
        }
    }  

    if(escIndex.x()==mci.x()&&escIndex.y()==mci.y()) ROS_WARN_STREAM("Can't avoid !!");

    ROS_INFO_STREAM("avoid angle map");
    std::cout << "y↑\n  →\n  x" << std::endl;
    for(int y=ESC_MAP_DIV_Y-1;y!=-1;--y){
        std::cout << "|";
        for(int x=0;x!=ESC_MAP_DIV_X;++x) std::cout << (x == escIndex.x() && y == escIndex.y() ? "@" : x == mci.x() && y == mci.y() ? "R" : gmm[x][y].grad == gmm[escIndex.x()][escIndex.y()].grad ? "*" : " ") << "|";
        std::cout << std::endl;
    }

    static Eigen::Vector2i lastIndex(INT_MAX,INT_MAX);
    static bool loop = true;

    if(escIndex.x()!=lastIndex.x()||escIndex.y()!=lastIndex.y()){
        rotationFromTo(pose.pose.orientation,gmm[escIndex.x()][escIndex.y()].pose.orientation);
        lastIndex = escIndex;
        loop = true;
    }
    
    while(loop && lookupCostmap() && ros::ok()) {
        ROS_INFO_STREAM("escape to forward");
        publishMovementStatus("esc_costmap");
        velocity_->pub.publish(ExCos::msgTwist(FORWARD_VELOCITY,0));
        while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
        escapeFromCostmap(pose_->data);
    }
    loop = false;
    lastIndex << INT_MAX,INT_MAX;
}

void Movement::rotationFromTo(const geometry_msgs::Quaternion& from, const geometry_msgs::Quaternion& to){
    double rotation = ExUtl::shorterRotationAngle(from,to);
    previousOrientation_ = rotation;

    ROS_INFO_STREAM("need rotation : " << rotation);
    ROS_INFO_STREAM("from : " << ExCov::qToYaw(from) << ", from(rad) : " << ExCov::qToYaw(from)*180/M_PI);
    ROS_INFO_STREAM("to : " << ExCov::qToYaw(to) << ", to(rad) : " << ExCov::qToYaw(to)*180/M_PI);

    double sum = 0;
    double la = ExCov::qToYaw(from);

    if(rotation>=0){    
        while(ExCov::qToYaw(pose_->data.pose.orientation) > 0 && sum < rotation - ROTATION_TOLERANCE && ros::ok()){
            velocity_->pub.publish(ExCos::msgTwist(0,ROTATION_VELOCITY));
            while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
            sum += ExCov::qToYaw(pose_->data.pose.orientation) > 0 ? ExCov::qToYaw(pose_->data.pose.orientation)-la : ExCov::qToYaw(pose_->data.pose.orientation) + M_PI;
            la = ExCov::qToYaw(pose_->data.pose.orientation) > 0 ? ExCov::qToYaw(pose_->data.pose.orientation) : -M_PI;
        }
        while(sum < rotation - ROTATION_TOLERANCE && ros::ok()){
            velocity_->pub.publish(ExCos::msgTwist(0,ROTATION_VELOCITY));
            while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
            sum += ExCov::qToYaw(pose_->data.pose.orientation)-la;
            la = ExCov::qToYaw(pose_->data.pose.orientation);
        }
    }
    else{
        while(ExCov::qToYaw(pose_->data.pose.orientation) < 0 && sum > rotation + ROTATION_TOLERANCE && ros::ok()){
            velocity_->pub.publish(ExCos::msgTwist(0,-ROTATION_VELOCITY));
            while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
            sum += ExCov::qToYaw(pose_->data.pose.orientation) < 0 ? ExCov::qToYaw(pose_->data.pose.orientation)-la : ExCov::qToYaw(pose_->data.pose.orientation) - M_PI;
            la = ExCov::qToYaw(pose_->data.pose.orientation) < 0 ? ExCov::qToYaw(pose_->data.pose.orientation) : M_PI;
        }
        while(sum > rotation + ROTATION_TOLERANCE && ros::ok()){
            velocity_->pub.publish(ExCos::msgTwist(0,-ROTATION_VELOCITY));
            while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
            sum += ExCov::qToYaw(pose_->data.pose.orientation)-la;
            la = ExCov::qToYaw(pose_->data.pose.orientation);
        }
    }
}

bool Movement::resetGoal(geometry_msgs::PoseStamped& goal){
    // 現在のゴール地点までのパスを一定間隔だけ遡って新たな目的地にする
    // true:リセっと可能, false:リセット不可能
    //現在のパスを取
    ROS_INFO_STREAM("reset goal");

    std::vector<geometry_msgs::PoseStamped> path;
    int pc = 0;
    ros::Rate rate(RESET_GOAL_PATH_RATE);
    
    while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
    while(!pp_->createPath(pose_->data,goal,path) && ros::ok()){
        ROS_INFO_STREAM("Waiting path ..."); // 一生パスが作れない場合もあるので注意
        if(++pc >= RESET_GOAL_PATH_LIMIT){
            ROS_WARN_STREAM("create path limit");
            return false;
        }
        rate.sleep();
    }
    ROS_INFO_STREAM("get path");
    // パスを少し遡ったところを目的地にする
    ROS_INFO_STREAM("path size: " << path.size());
    ROS_INFO_STREAM("PATH_BACK_INTERVAL: " << PATH_BACK_INTERVAL);
    while(gCostmap_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting global costmap ...");

    // ここの中でこすとまっぷにかからなくなるまで再計算
    for(int i=1;PATH_BACK_INTERVAL*i < path.size() && ros::ok();++i){
        ROS_INFO_STREAM("goal reset try : " << i);
        if(!lookupCostmap(path[path.size() - PATH_BACK_INTERVAL*i],gCostmap_->data)){
            while(pose_->q.callOne(ros::WallDuration(1.0))&&ros::ok()) ROS_INFO_STREAM("Waiting pose ...");
            goal = path[path.size() - PATH_BACK_INTERVAL*i];
            Eigen::Vector2d vec;
            pp_->getVec(pose_->data,goal,vec,path);
            goal.pose.orientation = ExCov::eigenQuaToGeoQua(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),Eigen::Vector3d(vec.x(),vec.y(),0.0)));
            return true;
        }
    }
        
    ROS_INFO_STREAM("Can't reset goal");
    return false;
}

bool Movement::bumperCollision(const kobuki_msgs::BumperEvent& bumper){
    //壁に衝突してるかを確認して、してたらバック
    if(bumper.state){
        ROS_WARN_STREAM("Bumper Hit !!");
        ros::Time setTime = ros::Time::now();
        while(ros::Time::now()-setTime < ros::Duration(BACK_TIME)) velocity_->pub.publish(ExCos::msgTwist(BACK_VELOCITY,0));
        return true;
    }
    return false;
}

bool Movement::roadCenterDetection(const sensor_msgs::LaserScan& scan){
    ROS_INFO_STREAM("roadCenterDetection");

    static bool initialized = false;
    static tf::TransformListener listener;
    if(!initialized){
        listener.waitForTransform(pose_->data.header.frame_id, scan.header.frame_id, ros::Time(), ros::Duration(1.0));
        initialized = true;
    }

    ExStc::scanStruct ss(scan.ranges.size());

    for(int i=0,e=scan.ranges.size();i!=e;++i){
        if(!std::isnan(scan.ranges[i])){
            double tempAngle = scan.angle_min+(scan.angle_increment*i);
            if(scan.ranges[i]*cos(tempAngle) <= ROAD_CENTER_THRESHOLD){
                ss.ranges.emplace_back(scan.ranges[i]);
                ss.x.emplace_back(scan.ranges[i]*cos(tempAngle));
                ss.y.emplace_back(scan.ranges[i]*sin(tempAngle));
                ss.angles.emplace_back(std::move(tempAngle));
            }
        }
    }

    if(ss.ranges.size() < 2){
        road_->pub.publish(geometry_msgs::PointStamped());
        return false;
    }

    for(int i=0,e=ss.ranges.size()-1;i!=e;++i){
        if(std::abs(ss.y[i+1] - ss.y[i]) >= ROAD_THRESHOLD){
            ROS_DEBUG_STREAM("Road Center Found");
            //  通路中心座標pub
            road_->pub.publish(ExCov::pointToPointStamped(ExUtl::coordinateConverter2d<geometry_msgs::Point>(listener, pose_->data.header.frame_id, scan.header.frame_id, ExCos::msgPoint((ss.x[i+1] + ss.x[i])/2, (ss.y[i+1] + ss.y[i])/2)),pose_->data.header.frame_id));
            publishMovementStatus("ROAD_CENTER");
            velocity_->pub.publish(velocityGenerator((ss.angles[i]+ss.angles[i+1])/2,FORWARD_VELOCITY,ROAD_CENTER_GAIN));
            return true;
        }
    }
    ROS_DEBUG_STREAM("Road Center Do Not Found");
    road_->pub.publish(geometry_msgs::PointStamped());
    return false;
}

bool Movement::VFHMove(const sensor_msgs::LaserScan& scan, double angle){
    // 目標の周辺がNanになってればtrueでそのまま通す
    // 安全の確認ができなければその近くで安全になるアングルに行く
    // nan もしくは障害物距離がx以上であれば安全角度判定

    int ti; //target i
    // 中心の要素番号設定
    static Eigen::Vector2i cp = scan.ranges.size()%2==0 ? Eigen::Vector2i(scan.ranges.size()/2,scan.ranges.size()/2-3) : Eigen::Vector2i(scan.ranges.size()/2,scan.ranges.size()/2);//中心の位置調整
    std::swap(cp[0],cp[1]);

    // 目標角に一番近い要素番号を計算 0 radのときは特殊処理(要素サイズが偶数の場合))
    if(angle==0) ti = cp[0];
    else{
        double min = DBL_MAX;
        for(int i=0,ie=scan.ranges.size();i!=ie;++i){
            double diff = std::abs(angle - (scan.angle_min + scan.angle_increment * i));
            if(diff < min){
                min = std::move(diff);
                ti = i;
            }
        }
    }
    // その方向が安全であるかを見る   
    ROS_INFO_STREAM("angle : " << angle << ", ranges.size() : " << scan.ranges.size() << ", ti : " << ti << ", ti(rad) : " << scan.angle_min + ti*scan.angle_increment);

    // 距離の再計算
    std::vector<float> scanCalced;
    scanCalced.reserve(scan.ranges.size());
    for(int i=0,e=scan.ranges.size();i!=e;++i)
        scanCalced.emplace_back(CALC_RANGE_COS && !std::isnan(scan.ranges[i]) ? scan.ranges[i]*cos(scan.angle_min+(scan.angle_increment*i)) : scan.ranges[i]);

    // ここでrateがthreshold以下になるまでずらして計算
    int sw = 0;
    double rate;
    double nRate;
    double fRate;
    double aveDist;
    do{
        ti += sw;
        // tiをずらすごとにminusとplusを再計算
        int tPLUS = ti + (FORWARD_ANGLE/2)/scan.angle_increment;
        int tMINUS = ti - (FORWARD_ANGLE/2)/scan.angle_increment;
        int PLUS = tPLUS > scan.ranges.size() ? scan.ranges.size() : tPLUS;
        int MINUS = tMINUS < 0 ? 0 : tMINUS;

        if(tMINUS >= scan.ranges.size() || tPLUS < 0){
            ROS_INFO_STREAM("VFH search is failed");
            return false;
        }
        double dist = 0;
        int fc = 0;
        int nc = 0;
        // これだと結局近いところの障害物しか見てないのと同じ？
        for(int i=MINUS;i!=PLUS;++i){
            // nan or over far
            if(std::isnan(scanCalced[i])||scanCalced[i]>=VFH_FAR_RANGE_THRESHOLD){
                dist += VFH_FAR_RANGE_THRESHOLD;
                ++fc;
            }
            // far > range > near
            else if(scanCalced[i]>=VFH_NEAR_RANGE_THRESHOLD){
                dist += scanCalced[i];
                ++nc;
            }
        }
        rate = (double)(fc+nc) / (PLUS-MINUS);
        fRate = (double)fc / (PLUS-MINUS);
        nRate = (double)nc / (PLUS-MINUS);
        aveDist = dist / (fc+nc);
        sw = sw > 0 ? -sw-1 : -sw+1;
    }while(fRate < VFH_RATE_THRESHOLD && nRate < VFH_RATE_THRESHOLD);

    double gain = ti==cp[0] ? 0 : (aveDist - VFH_NEAR_RANGE_THRESHOLD) * (FAR_AVOIDANCE_GAIN - NEAR_AVOIDANCE_GAIN)/(VFH_FAR_RANGE_THRESHOLD - VFH_NEAR_RANGE_THRESHOLD) + NEAR_AVOIDANCE_GAIN;
    ROS_INFO_STREAM("ti : " << ti <<  ", angle : " << scan.angle_min + ti * scan.angle_increment << ", rate : " << rate << ", fRate : " << fRate << ", nRate : " << nRate << ", gain : " << gain << ", aveDist : " << aveDist);
    ROS_INFO_STREAM("this angle is safety");
    publishMovementStatus("VFH");
    velocity_->pub.publish(velocityGenerator(scan.angle_min+ti*scan.angle_increment,FORWARD_VELOCITY,gain));
    return true;
}

bool Movement::emergencyAvoidance(const sensor_msgs::LaserScan& scan){
    ROS_INFO_STREAM("emergencyAvoidance");

    // 距離の再計算
    std::vector<float> scanCalced;
    scanCalced.reserve(scan.ranges.size());
    for(int i=0,e=scan.ranges.size();i!=e;++i)
        scanCalced.emplace_back(CALC_RANGE_COS && !std::isnan(scan.ranges[i]) ? scan.ranges[i]*cos(scan.angle_min+(scan.angle_increment*i)) : scan.ranges[i]);

    double NAN_RATE = 0.8;

    //minus側の平均
    double aveM=0;
    int nanM = 0;
    for(int i=0,e=scan.ranges.size()/2;i!=e;++i){
        if(!std::isnan(scanCalced[i])) aveM += scanCalced[i];
        else ++nanM;
    }
    aveM = nanM > (scan.ranges.size()/2)*NAN_RATE ? DBL_MAX : aveM / (scan.ranges.size()/2-nanM);

    //plus側
    double aveP=0;
    int nanP = 0;
    for(int i=scan.ranges.size()/2,e=scan.ranges.size();i!=e;++i){
        if(!std::isnan(scanCalced[i])) aveP += scanCalced[i];
        else ++nanP;
    }
    aveP = nanP > (scan.ranges.size()/2)*NAN_RATE ? DBL_MAX : aveP / (scan.ranges.size()/2-nanP);

    //左右の差がそんなにないなら前回避けた方向を採用する
    //一回目に避けた方向に基本的に従う
    //一回避けたら大きく差が出ない限りおなじほうこうに避ける

    ROS_DEBUG_STREAM("aveP : " << aveP << ", aveM : " << aveM <<  ", nanP : " << nanP << ", nanM : " << nanM);

    //まずよけれる範囲か見る
    if(aveP > EMERGENCY_THRESHOLD || aveM > EMERGENCY_THRESHOLD){
        //センサの安全領域の大きさが変わった時の処理//大きさがほとんど同じだった時の処理//以前避けた方向に避ける
        if(std::abs(aveM-aveP) > EMERGENCY_DIFF_THRESHOLD) previousOrientation_ = aveP > aveM ? 1.0 : -1.0;
        ROS_INFO_STREAM((previousOrientation_ > 0 ? "Avoidance to Left" : "Avoidance to Right"));
        velocity_->pub.publish(velocityGenerator((previousOrientation_ > 0 ? 1 : -1)*scan.angle_max/6, FORWARD_VELOCITY/2, EMERGENCY_AVOIDANCE_GAIN));
        publishMovementStatus("EMERGENCY");
        return true;
    }
    else{
        ROS_WARN_STREAM("I can not avoid it");
        return false;
    }
}

bool Movement::forwardWallDetection(const sensor_msgs::LaserScan& scan, double& angle){
    //前方に壁があるかどうかを判定する
    //前方何度まで見るかを決める
    //その範囲にセンサデータが何割存在するかで壁かどうか決める
    //壁があった場合右と左のどっちに避けるか決定

    ROS_INFO_STREAM("forwardWallDetection");

    const int CENTER_SUBSCRIPT = scan.ranges.size()/2;
    int PLUS = CENTER_SUBSCRIPT + (int)((WALL_FORWARD_ANGLE/2)/scan.angle_increment);
    int MINUS = CENTER_SUBSCRIPT - (int)((WALL_FORWARD_ANGLE/2)/scan.angle_increment);

    ROS_INFO_STREAM("ranges.size() : " << scan.ranges.size() << ", CENTER_SUBSCRIPT : " << CENTER_SUBSCRIPT << ", calc : " << (int)((WALL_FORWARD_ANGLE/2)/scan.angle_increment));

    int count = 0;
    double wallDistance = 0;

    for(int i=MINUS;i!=PLUS;++i){
        if(!std::isnan(scan.ranges[i])){
            ++count;
            wallDistance += scan.ranges[i];
        }
    }
    ROS_INFO_STREAM("PLUS : " << PLUS << ", MINUS : " << MINUS << ", count : " << count << ", rate : " << (double)count/(PLUS-MINUS));

    wallDistance /= count;

    //壁の割合が少ないときと、壁までの距離が遠い時は検出判定をしない
    //追加で壁に近くなりすぎると判定ができなくなるので無効とする
    if((double)count/(PLUS-MINUS) > WALL_RATE_THRESHOLD && wallDistance < WALL_DISTANCE_UPPER_THRESHOLD && wallDistance > WALL_DISTANCE_LOWER_THRESHOLD){
        ROS_INFO_STREAM("Wall Found : " << wallDistance << " [m]");
        angle = sideSpaceDetection(scan,PLUS, MINUS);
        return true;
    }
    else{
        ROS_INFO_STREAM("Wall not Found or Out of Range");
        return false;
    }
}

double Movement::sideSpaceDetection(const sensor_msgs::LaserScan& scan, int plus, int minus){
    ROS_INFO_STREAM("sideSpaceDetection");
    //minus
    int countNanMinus = 0;
    double maxSpaceMinus = 0;
    double aveMinus = 0;
    for(int i=0;i!=minus;++i){
        if(!std::isnan(scan.ranges[i])){
            aveMinus += scan.ranges[i];
            double temp;
            for(int j=i+1;j!=minus;++j){
                if(!std::isnan(scan.ranges[j])){
                    temp = std::abs(scan.ranges[i]*cos(scan.angle_min + scan.angle_increment*i)-scan.ranges[j]*cos(scan.angle_min + scan.angle_increment*j));
                    break;
                }
            }
            if(temp > maxSpaceMinus) maxSpaceMinus = std::move(temp);
        }
        else ++countNanMinus;
    }

    //plus
    double avePlus = 0;
    int countNanPlus = 0;
    double maxSpacePlus = 0;
    for(int i=plus,e=scan.ranges.size();i!=e;++i){
        if(!std::isnan(scan.ranges[i])){
            avePlus += scan.ranges[i];
            double temp;
            for(int j=i-1;j!=-1;--j){
                if(!std::isnan(scan.ranges[j])){
                    temp = std::abs(scan.ranges[i]*cos(scan.angle_min + scan.angle_increment*i)-scan.ranges[j]*cos(scan.angle_min + scan.angle_increment*j));
                    break;
                }
            }
            if(temp > maxSpacePlus) maxSpacePlus = std::move(temp);
        }
        else ++countNanPlus;
    }

    aveMinus /= (minus - countNanMinus);
    avePlus /= (scan.ranges.size() - plus - countNanPlus);

    //不確定 //壁までの距離が遠いときは平均距離が長いほうが良い、近いときは開いてる領域が大きい方が良い
    if(maxSpaceMinus > maxSpacePlus && aveMinus > EMERGENCY_THRESHOLD){
        ROS_INFO_STREAM("Found Right Space");
        return (scan.angle_min + (scan.angle_increment * (scan.ranges.size()/2+minus)/2))/2;
    }
    else if(maxSpacePlus > maxSpaceMinus && avePlus > EMERGENCY_THRESHOLD){
        ROS_INFO_STREAM("Found Left Space");
        return (scan.angle_min + (scan.angle_increment * (plus + scan.ranges.size()/2)/2))/2;
    }
    else{
        ROS_INFO_STREAM("Not Found Space");
        return 0;
    }
}

geometry_msgs::Twist Movement::velocityGenerator(double theta,double v,double gain){
    if(theta != 0) previousOrientation_ = theta;
    return ExCos::msgTwist(v,gain*CURVE_GAIN*theta);
}

void Movement::publishMovementStatus(const std::string& status){
    const int PATTERN = 3; 
    exploration_msgs::AvoidanceStatus msg;
    msg.status = status;
    msg.calc_range_method = CALC_RANGE_COS ? exploration_msgs::AvoidanceStatus::COS : exploration_msgs::AvoidanceStatus::NORMAL;
    msg.range_pattern.reserve(PATTERN);
    msg.range_pattern.emplace_back(VFH_FAR_RANGE_THRESHOLD);
    msg.range_pattern.emplace_back(VFH_NEAR_RANGE_THRESHOLD);
    msg.range_pattern.emplace_back(EMERGENCY_THRESHOLD);
    msg.descriptions.reserve(PATTERN);
    msg.descriptions.emplace_back("VFH_FAR_RANGE_THRESHOLD");
    msg.descriptions.emplace_back("VFH_NEAR_RANGE_THRESHOLD");
    msg.descriptions.emplace_back("EMERGENCY_THRESHOLD");
    msg.scan_frame_id = scan_->data.header.frame_id;
    msg.scan_angle_min = scan_->data.angle_min;
    msg.scan_angle_max = scan_->data.angle_max;
    msg.scan_angle_increment = scan_->data.angle_increment;
    msg.header.stamp = ros::Time::now();
    avoStatus_->pub.publish(msg);
}

void Movement::loadParams(void){
    ros::NodeHandle nh("~/movement");
    // dynamic parameters
    nh.param<double>("forward_velocity", FORWARD_VELOCITY, 0.2);
    nh.param<double>("rotation_velocity", ROTATION_VELOCITY, 0.5);
    nh.param<double>("curve_gain", CURVE_GAIN, 2.0);
    nh.param<bool>("use_angle_bias", USE_ANGLE_BIAS, false);
    nh.param<double>("angle_bias", ANGLE_BIAS, 10.0);
    nh.param<double>("costmap_margin", COSTMAP_MARGIN, 0.4);
    nh.param<int>("esc_map_div_x", ESC_MAP_DIV_X, 3);
    nh.param<int>("esc_map_div_y", ESC_MAP_DIV_Y, 3);
    nh.param<double>("esc_map_width", ESC_MAP_WIDTH, 0.9);
    nh.param<double>("esc_map_height", ESC_MAP_HEIGHT, 0.9);
    nh.param<double>("rotation_tolerance", ROTATION_TOLERANCE, 0.05);
    nh.param<double>("goal_reset_rate", GOAL_RESET_RATE, 1);
    nh.param<int>("path_back_interval", PATH_BACK_INTERVAL, 5);
    nh.param<int>("reset_goal_path_limit", RESET_GOAL_PATH_LIMIT, 30);
    nh.param<double>("reset_goal_path_rate", RESET_GOAL_PATH_RATE, 0.5);
    nh.param<double>("back_velocity", BACK_VELOCITY, -0.2);
    nh.param<double>("back_time", BACK_TIME, 1.0);
    nh.param<double>("road_center_threshold", ROAD_CENTER_THRESHOLD, 5.0);
    nh.param<double>("road_threshold", ROAD_THRESHOLD, 1.5);
    nh.param<double>("road_center_gain", ROAD_CENTER_GAIN, 1.25);
    nh.param<double>("forward_angle", FORWARD_ANGLE, 0.35);
    nh.param<double>("vfh_far_range_threshold", VFH_FAR_RANGE_THRESHOLD, 5.0);
    nh.param<double>("vfh_near_range_threshold", VFH_NEAR_RANGE_THRESHOLD, 1.5);
    nh.param<double>("vfh_rate_threshold", VFH_RATE_THRESHOLD, 0.9);
    nh.param<double>("far_avoidance_gain", FAR_AVOIDANCE_GAIN, 2.5);
    nh.param<double>("near_avoidance_gain", NEAR_AVOIDANCE_GAIN, 2.5);
    nh.param<bool>("calc_range_cos", CALC_RANGE_COS, true);
    nh.param<double>("emergency_threshold", EMERGENCY_THRESHOLD, 0.1);
    nh.param<double>("emergency_diff_threshold", EMERGENCY_DIFF_THRESHOLD, 0.1);
    nh.param<double>("emergency_avoidance_gain", EMERGENCY_AVOIDANCE_GAIN, 2.5);
    nh.param<bool>("approach_wall", APPROACH_WALL, false);
    nh.param<double>("wall_forward_angle", WALL_FORWARD_ANGLE, 0.35);
    nh.param<double>("wall_rate_threshold", WALL_RATE_THRESHOLD, 0.8);
    nh.param<double>("wall_distance_upper_threshold", WALL_DISTANCE_UPPER_THRESHOLD, 5.0);
    nh.param<double>("wall_distance_lower_threshold", WALL_DISTANCE_LOWER_THRESHOLD, 0.5);
    // static parameters
    nh.param<std::string>("movebase_name", MOVEBASE_NAME, "move_base");
    nh.param<std::string>("movement_parameter_file_path",MOVEMENT_PARAMETER_FILE_PATH,"movement_last_parameters.yaml");
    nh.param<bool>("output_movement_parameters",OUTPUT_MOVEMENT_PARAMETERS,true);
}

void Movement::dynamicParamsCB(exploration::movement_parameter_reconfigureConfig &cfg, uint32_t level){
    FORWARD_VELOCITY = cfg.forward_velocity;
    ROTATION_VELOCITY = cfg.rotation_velocity;
    CURVE_GAIN = cfg.curve_gain;
    USE_ANGLE_BIAS = cfg.use_angle_bias;
    ANGLE_BIAS = cfg.angle_bias;
    COSTMAP_MARGIN = cfg.costmap_margin;
    ESC_MAP_DIV_X = cfg.esc_map_div_x;
    ESC_MAP_DIV_Y = cfg.esc_map_div_y;
    ESC_MAP_WIDTH = cfg.esc_map_width;
    ESC_MAP_HEIGHT = cfg.esc_map_height;
    ROTATION_TOLERANCE = cfg.rotation_tolerance;
    GOAL_RESET_RATE = cfg.goal_reset_rate;
    PATH_BACK_INTERVAL = cfg.path_back_interval;
    RESET_GOAL_PATH_LIMIT = cfg.reset_goal_path_limit;
    RESET_GOAL_PATH_RATE = cfg.reset_goal_path_rate;
    BACK_VELOCITY = cfg.back_velocity;
    BACK_TIME = cfg.back_time;
    ROAD_CENTER_THRESHOLD = cfg.road_center_threshold;
    ROAD_THRESHOLD = cfg.road_threshold;
    ROAD_CENTER_GAIN = cfg.road_center_gain;
    FORWARD_ANGLE = cfg.forward_angle;
    VFH_FAR_RANGE_THRESHOLD = cfg.vfh_far_range_threshold;
    VFH_NEAR_RANGE_THRESHOLD = cfg.vfh_near_range_threshold;
    VFH_RATE_THRESHOLD = cfg.vfh_rate_threshold;
    FAR_AVOIDANCE_GAIN = cfg.far_avoidance_gain;
    NEAR_AVOIDANCE_GAIN = cfg.near_avoidance_gain;
    CALC_RANGE_COS = cfg.calc_range_cos;
    EMERGENCY_THRESHOLD = cfg.emergency_threshold;
    EMERGENCY_DIFF_THRESHOLD = cfg.emergency_diff_threshold;
    EMERGENCY_AVOIDANCE_GAIN = cfg.emergency_avoidance_gain;
    APPROACH_WALL = cfg.approach_wall;
    WALL_FORWARD_ANGLE = cfg.wall_forward_angle;
    WALL_RATE_THRESHOLD = cfg.wall_rate_threshold;
    WALL_DISTANCE_UPPER_THRESHOLD = cfg.wall_distance_upper_threshold;
    WALL_DISTANCE_LOWER_THRESHOLD = cfg.wall_distance_lower_threshold;
}

void Movement::outputParams(void){
    std::cout << "writing movement last parameters ... ..." << std::endl;
    std::ofstream ofs(MOVEMENT_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "momement param file open succeeded" << std::endl;
    else {
        std::cout << "momement param file open failed" << std::endl;
        return;
    }

    ofs << "forward_velocity: " << FORWARD_VELOCITY << std::endl;
    ofs << "rotation_velocity: " << ROTATION_VELOCITY << std::endl;
    ofs << "curve_gain: " << CURVE_GAIN << std::endl;
    ofs << "use_angle_bias: " << (USE_ANGLE_BIAS ? "true" : "false") << std::endl; 
    ofs << "angle_bias: " << ANGLE_BIAS << std::endl;
    ofs << "costmap_margin: " << COSTMAP_MARGIN << std::endl;
    ofs << "esc_map_div_x: " << ESC_MAP_DIV_X << std::endl;
    ofs << "esc_map_div_y: " << ESC_MAP_DIV_Y << std::endl;
    ofs << "esc_map_width: " << ESC_MAP_WIDTH << std::endl;
    ofs << "esc_map_height: " << ESC_MAP_HEIGHT << std::endl;
    ofs << "rotation_tolerance: " << ROTATION_TOLERANCE << std::endl;    
    ofs << "goal_reset_rate: " << GOAL_RESET_RATE << std::endl;
    ofs << "path_back_interval: " << PATH_BACK_INTERVAL << std::endl;
    ofs << "reset_goal_path_limit: " << RESET_GOAL_PATH_LIMIT << std::endl;
    ofs << "reset_goal_path_rate: " << RESET_GOAL_PATH_RATE << std::endl;
    ofs << "back_velocity: " << BACK_VELOCITY << std::endl;
    ofs << "back_time: " << BACK_TIME << std::endl;
    ofs << "road_center_threshold: " << ROAD_CENTER_THRESHOLD << std::endl;
    ofs << "road_threshold: " << ROAD_THRESHOLD << std::endl;
    ofs << "road_center_gain: " << ROAD_CENTER_GAIN << std::endl;
    ofs << "forward_angle: " << FORWARD_ANGLE << std::endl;
    ofs << "vfh_far_range_threshold: " << VFH_FAR_RANGE_THRESHOLD << std::endl;
    ofs << "vfh_near_range_threshold: " << VFH_NEAR_RANGE_THRESHOLD << std::endl;
    ofs << "vfh_rate_threshold: " << VFH_RATE_THRESHOLD << std::endl;
    ofs << "far_avoidance_gain: " << FAR_AVOIDANCE_GAIN << std::endl;
    ofs << "near_avoidance_gain: " << NEAR_AVOIDANCE_GAIN << std::endl;
    ofs << "calc_range_cos: " << (CALC_RANGE_COS ? "true" : "false") << std::endl;
    ofs << "emergency_threshold: " << EMERGENCY_THRESHOLD << std::endl;
    ofs << "emergency_diff_threshold: " << EMERGENCY_DIFF_THRESHOLD << std::endl;
    ofs << "emergency_avoidance_gain: " << EMERGENCY_AVOIDANCE_GAIN << std::endl;
    ofs << "approach_wall: " << (APPROACH_WALL ? "true" : "false") << std::endl;
    ofs << "wall_forward_angle: " << WALL_FORWARD_ANGLE << std::endl;
    ofs << "wall_rate_threshold: " << WALL_RATE_THRESHOLD << std::endl;
    ofs << "wall_distance_upper_threshold: " << WALL_DISTANCE_UPPER_THRESHOLD << std::endl;
    ofs << "wall_distance_lower_threshold: " << WALL_DISTANCE_LOWER_THRESHOLD << std::endl;
}
