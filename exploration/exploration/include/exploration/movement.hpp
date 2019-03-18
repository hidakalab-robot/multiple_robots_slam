#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>
#include <exploration_msgs/ToGoal.h>
#include <exploration_msgs/MoveAngle.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <exploration/path_planning.hpp>
#include <navfn/navfn_ros.h>
#include <voronoi_planner/planner_core.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//センサーデータを受け取った後にロボットの動作を決定する
//障害物回避を含む

/*
Using function Example

if you want move to goal with move_base
    moveToGoal(goal,true);

if you want move to goal without move_base
    moveToGoal(goal,false);
        or
    moveToGoal(goal);

if you want to create path and move to goal
    moveToGoal(createPath(goal));

if you have path and use it
    moveToGoal(path);

if you want to move forward without goal
    moveToForward();

if you want to one rotations
    oneRotation();

if you want to get path only
    createPath(goal);  //std::vector<geometry_msgs::PoseStamped> is returned 
*/

class Movement 
{
private:
    ros::NodeHandle p;
    double GOAL_TOLERANCE;
    double GRAVITY_FORCE_ENABLE;
    double GRAVITY_GAIN;
    double GRAVITH_DIFF_THRESHOLD;
    double SAFE_DISTANCE;
    double SAFE_SPACE;
    double SCAN_THRESHOLD;
    double FORWARD_VELOCITY;
    double BACK_VELOCITY;
    double BACK_TIME;
    double BUMPER_ROTATION_TIME;
    double ROTATION_VELOCITY;
    double EMERGENCY_THRESHOLD;
    double ROAD_CENTER_THRESHOLD;
    double ROAD_THRESHOLD;
    double CURVE_GAIN;
    int TRY_COUNT;
    bool AVOIDANCE_TO_GOAL;
    double VELOCITY_GAIN;

    double AVOIDANCE_GAIN;
    double VFH_GAIN;
    double ROAD_CENTER_GAIN;

    double MATCH_ANGLE_THRESHOLD;
    double MATCH_ROTATION_VELOCITY;

    double PATH_RADIUS;
    double LOCAL_TOLERANCE;

    double ROTATION_GAIN;

    std::string COSTMAP_NAME;
    std::string PLANNER_NAME;
    int PLANNER_METHOD;

    std::string MOVEBASE_NAME;
    std::string MAP_FRAME_ID;

    bool PUBLISH_MY_VORONOI;

    int INT_INFINITY;
    double DOUBLE_INFINITY;

    double FORWARD_ANGLE;
    double WALL_RATE_THRESHOLD;
    double WALL_DISTANCE_UPPER_THRESHOLD;
    double WALL_DISTANCE_LOWER_THRESHOLD;

    ros::NodeHandle ss;
    ros::Subscriber subScan;
    ros::CallbackQueue qScan;
    sensor_msgs::LaserScan scanData;
    sensor_msgs::LaserScan scanDataOrigin;

    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    geometry_msgs::PoseStamped poseData;

    ros::NodeHandle sb;
    ros::Subscriber subBumper;
    ros::CallbackQueue qBumper;
    kobuki_msgs::BumperEvent bumperData;

    ros::NodeHandle pv;
    ros::Publisher pubVelocity;

    ros::NodeHandle ptg;
    ros::Publisher pubToGoal;

    ros::NodeHandle ptgd;
    ros::Publisher pubToGoalDel;

    ros::NodeHandle pma;
    ros::Publisher pubMoveAngle;

    ros::NodeHandle pvm;
    ros::Publisher pubVoronoiMap;

    double previousOrientation;
    double goalDirection;
    bool existGoal;
    
    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void approx(std::vector<float>& scan);
    void vfhMovement(bool isStraight, const geometry_msgs::Point& goal);
    void vfhMovement(bool isStraight, double angle);
    bool bumperCollision(const kobuki_msgs::BumperEvent& bumper);
    double vfhCalculation(const sensor_msgs::LaserScan& scan, bool isCenter, double goalAngle = 0.0);
    double localAngleCalculation(const geometry_msgs::Point& goal,const geometry_msgs::PoseStamped& pose);
    double qToYaw(const geometry_msgs::Quaternion& q);
    bool emergencyAvoidance(const sensor_msgs::LaserScan& scan);
    void recoveryRotation(void);
    void velocityPublisher(double theta, double v, double t);
    bool roadCenterDetection(const sensor_msgs::LaserScan& scan);
    void publishToGoal(const geometry_msgs::Pose& pose, const geometry_msgs::Point& goal);
    void publishMoveAngle(double angle, const geometry_msgs::Pose& pose, const geometry_msgs::Twist& vel);
    void publishToGoalDelete(void);

    void publishVoronoiMap(const nav_msgs::OccupancyGrid& map);

    bool callPathPlanner(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path);

    bool callVoronoiPlanner(const std::string& costmapName, const std::string& plannerName, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path);
    bool callNavfn(const std::string& costmapName, const std::string& plannerName, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path);

    void directionFitting(double targetYaw);
    void directionFitting(const geometry_msgs::Point& target);

    //moveToForwardのときの障害物回避で、前方に壁があったときの処理
    bool forwardWallDetection(const sensor_msgs::LaserScan& scan, double& angle);
    double sideSpaceDetection(const sensor_msgs::LaserScan& scan, int plus, int minus);

public:
    Movement();
    ~Movement(){};

    std::vector<geometry_msgs::PoseStamped> createPath(const geometry_msgs::Point& goal);
    void moveToGoal(const geometry_msgs::Point& goal,bool movebase);
    void moveToGoal(const geometry_msgs::Point& goal);
    void moveToGoal(const std::vector<geometry_msgs::PoseStamped>& path); // not stable

    void moveToForward(void);
    void oneRotation(void);

    void functionCallTester(void);
};

Movement::Movement():p("~"){
    p.param<double>("goal_tolerance", GOAL_TOLERANCE, 0.5);
    p.param<double>("gravity_gain", GRAVITY_GAIN, 1.2);
    p.param<double>("gravity_force_enable", GRAVITY_FORCE_ENABLE, 6.0);
    p.param<double>("gravity_diff_threshold", GRAVITH_DIFF_THRESHOLD, 0.1);
    p.param<double>("safe_distance", SAFE_DISTANCE, 0.75);
    p.param<double>("safe_space", SAFE_SPACE, 0.6);
    p.param<double>("scan_threshold", SCAN_THRESHOLD, 1.2);
    p.param<double>("forward_velocity", FORWARD_VELOCITY, 0.2);
    p.param<double>("back_velocity", BACK_VELOCITY, -0.2);
    p.param<double>("back_time", BACK_TIME, 0.5);
    p.param<double>("bumper_rotation_time", BUMPER_ROTATION_TIME, 1.5);
    p.param<double>("rotation_velocity", ROTATION_VELOCITY, 0.5);
    p.param<double>("emergency_threshold", EMERGENCY_THRESHOLD, 1.0);
    p.param<double>("road_center_threshold", ROAD_CENTER_THRESHOLD, 5.0);
    p.param<double>("road_threshold", ROAD_THRESHOLD, 1.5);
    p.param<double>("curve_gain", CURVE_GAIN, 2.0);
    p.param<int>("try_count", TRY_COUNT, 1);
    p.param<bool>("avoidance_to_goal", AVOIDANCE_TO_GOAL, true);
    p.param<double>("velocity_gain", VELOCITY_GAIN, 1.0);
    p.param<double>("rotation_gain", ROTATION_GAIN, 1.0);
    p.param<double>("avoidance_gain", AVOIDANCE_GAIN, 0.3);
    p.param<double>("vfh_gain", VFH_GAIN, 0.5);
    p.param<double>("road_center_gain", ROAD_CENTER_GAIN, 0.8);

    p.param<std::string>("costmap_name", COSTMAP_NAME, "global_costmap");
    p.param<std::string>("planner_name", PLANNER_NAME, "path_planner");
    p.param<int>("planner_method", PLANNER_METHOD, 0);

    p.param<double>("match_angle_threshold", MATCH_ANGLE_THRESHOLD, 0.1);
    p.param<double>("match_rotation_velocity", MATCH_ROTATION_VELOCITY, 0.2);

    p.param<double>("path_radius", PATH_RADIUS, 0.5);
    p.param<double>("local_tolerance", LOCAL_TOLERANCE, 0.2);

    p.param<std::string>("movebase_name", MOVEBASE_NAME, "move_base");
    p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");

    p.param<double>("forward_angle", FORWARD_ANGLE, 0.17);
    p.param<double>("wall_rate_threshold", WALL_RATE_THRESHOLD, 0.8);
    p.param<double>("wall_distance_upper_threshold", WALL_DISTANCE_UPPER_THRESHOLD, 5.0);
    p.param<double>("wall_distance_lower_threshold", WALL_DISTANCE_LOWER_THRESHOLD, 3.0);

    ss.setCallbackQueue(&qScan);
    subScan = ss.subscribe("scan",1,&Movement::scanCB, this);

    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&Movement::poseCB,this);

    sb.setCallbackQueue(&qBumper);
    subBumper = sb.subscribe("bumper",1,&Movement::bumperCB,this);

    pubVelocity = pv.advertise<geometry_msgs::Twist>("velocity", 1);
    pubToGoal = ptg.advertise<exploration_msgs::ToGoal>("to_goal", 1,true);
    pubMoveAngle = pma.advertise<exploration_msgs::MoveAngle>("move_angle", 1);
    pubToGoalDel = ptgd.advertise<std_msgs::Empty>("to_goal/delete", 1,true);

    p.param<bool>("publish_my_voronoi", PUBLISH_MY_VORONOI, false);
    if(PUBLISH_MY_VORONOI){
        pubVoronoiMap = pvm.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);
    }

    INT_INFINITY = 1000000;
    DOUBLE_INFINITY = 100000.0;

    goalDirection = 0;
    existGoal = false;
}

void Movement::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    scanData = *msg;
    scanDataOrigin = *msg;
    approx(scanData.ranges);
}

void Movement::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

void Movement::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumperData = *msg;
}

void Movement::approx(std::vector<float>& scan){
    float depth,depth1,depth2;
    depth = depth1 = depth2 =0;

    for(int j=0,count=0;j<scan.size()-1;j++){
        depth=scan[j];
        //|val|nan|のとき
        if(!std::isnan(depth) && std::isnan(scan[j+1])){
            depth1=depth;
            count++;
        }
        if(std::isnan(depth)){
            //|nan|nan|の区間
            if(std::isnan(scan[j+1])){
                count++;
            }
            //|nan|val|のとき
            else{
                depth2=scan[j+1];
                //左端がnanのとき
                if(std::isnan(depth1)){
                    for(int k=0;k<count+1;k++)
                        scan[j-k]=0.01;//depth2;
                }
                else{
                    for(int k=0;k<count;k++)
                        scan[j-k]=depth2-(depth2-depth1)/(count+1)*(k+1);
                }
                count=0;
            }
        }
        //右端がnanのとき
        if(j==(scan.size()-1)-1 && std::isnan(scan[j+1])){
            for(int k=0;k<count;k++)
                scan[j+1-k]=0.01;//depth1;
            count=0;
        }
    }		
    if(std::isnan(scan[0])){
        scan[0] = scan[1] - (scan[2] - scan[1]);
        if(scan[0] < 0){
            scan[0] = 0;
        }
    }
}

std::vector<geometry_msgs::PoseStamped> Movement::createPath(const geometry_msgs::Point& goal){
    qPose.callOne(ros::WallDuration(1));
    geometry_msgs::PoseStamped start;
    start = poseData;
    geometry_msgs::PoseStamped goalStamped;
    goalStamped.header.frame_id = poseData.header.frame_id;
    goalStamped.pose.position.x = goal.x;
    goalStamped.pose.position.y = goal.y;
    std::vector<geometry_msgs::PoseStamped> path;
    if(callPathPlanner(start,goalStamped,path)){
        ROS_INFO_STREAM("Path was Found\n");
        //ROS_DEBUG_STREAM("path size : " << path.size() << "\n");
    }
    else{
        ROS_INFO_STREAM("Path was not Found\n");
    }
    return path;
}

void Movement::moveToGoal(const geometry_msgs::Point& goal,bool movebase){
    if(!movebase){
        moveToGoal(goal);
    }
    else{
        static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(MOVEBASE_NAME, true);
        
        while(!ac.waitForServer(ros::Duration(1.0)) && ros::ok()){
            ROS_INFO_STREAM("wait for action server << " << MOVEBASE_NAME << "\n");
        }

        move_base_msgs::MoveBaseGoal movebaseGoal;
        movebaseGoal.target_pose.header.frame_id = MAP_FRAME_ID;
        movebaseGoal.target_pose.header.stamp = ros::Time::now();
        movebaseGoal.target_pose.pose.position.x =  goal.x;
        movebaseGoal.target_pose.pose.position.y =  goal.y;
        movebaseGoal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO_STREAM("send goal to move_base\n");
        ac.sendGoal(movebaseGoal);

        ROS_INFO_STREAM("wait for result\n");
        ac.waitForResult();

        ROS_INFO_STREAM("move_base was finished\n");

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO_STREAM("I Reached Given Target");
        }
        else{
            ROS_WARN_STREAM("I do not Reached Given Target");
        }

        //return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
}

void Movement::moveToGoal(const geometry_msgs::Point& goal){
    ROS_INFO_STREAM("Goal Recieved : (" << goal.x << "," << goal.y << ")\n");

    existGoal = true;

    qPose.callOne(ros::WallDuration(1));

    const double GRAVITY_ENABLE = std::abs(goal.y - poseData.pose.position.y) + GRAVITY_GAIN;
    double distToGoal;
    double distToGoalOld;
    double diff = 0;
    int count = 0;
    const int end = TRY_COUNT * 2 - 1; 
    int sign = -1;

    distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));

    ROS_INFO_STREAM("Distance to Goal : " << distToGoal << " [m]\n");

    geometry_msgs::Point nullGoal;

    //このループいらないかも要検討
    while(GRAVITY_ENABLE < distToGoal && distToGoal < GRAVITY_FORCE_ENABLE && ros::ok()){
        publishToGoal(poseData.pose, goal);
        distToGoalOld = distToGoal;
        vfhMovement(true,nullGoal);
        qPose.callOne(ros::WallDuration(1));
        distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));
        diff += distToGoal - distToGoalOld;
        ROS_INFO_STREAM("Refresh Distance to Goal : " << distToGoal << " [m]\n");
        if(std::abs(diff) > GRAVITH_DIFF_THRESHOLD){
            if(diff*sign < 0){
                sign *= -1;
                count++;
                if(count == end){
                    diff = 0;
                    count = 0;
                    sign = -1;
                    ROS_WARN_STREAM("This Goal Can Not Reach !\n");
                    //return; 
                    break;
                }
            }
            diff = 0;
        }
    }
    
    while(GOAL_TOLERANCE < distToGoal && ros::ok()){
        publishToGoal(poseData.pose, goal);
        distToGoalOld = distToGoal;
        vfhMovement(false,goal);
        qPose.callOne(ros::WallDuration(1));
        distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));
        diff += distToGoal - distToGoalOld;
        ROS_INFO_STREAM("Refresh Distance to Goal : " << distToGoal << " [m]\n");
        if(std::abs(diff) > GRAVITH_DIFF_THRESHOLD){
            if(diff*sign < 0){
                sign *= -1;
                count++;
                if(count == end){
                    diff = 0;
                    count = 0;
                    sign = -1;
                    ROS_WARN_STREAM("This Goal Can Not Reach !\n");
                    //return;
                    break;
                }
            }
            diff = 0;
        }
    }

    existGoal = false;
    publishToGoalDelete();
}

void Movement::moveToGoal(const std::vector<geometry_msgs::PoseStamped>& path){
    //planの中で半径x[m]より外で一番近いところを探す
    //初めに目標と角度合わせる
    //目標近くについたらplanの中で半径x[m]より外で次に近いところを探す
    //以下ループ

    ROS_INFO_STREAM("Path Recieved\n");

    qPose.callOne(ros::WallDuration(1));

    //double yaw = qToYaw(poseData.pose.orientation);

    if(path.size() == 0){
        ROS_ERROR_STREAM("Path is Empty\n");
        return;
    }

    //double PATH_RADIUS = 0.5;

    //目標位置との角度

    int tempI = INT_INFINITY;

    double localDistance;
    //planの中で半径x[m]より外で一番近いところを探す
    for(int i=0;i<path.size();i++){
        localDistance = sqrt(pow(path[i].pose.position.x - poseData.pose.position.x,2) + pow(path[i].pose.position.y - poseData.pose.position.y,2));
        if(localDistance > PATH_RADIUS){
            tempI = i;
            break;
        }
    }
    if(tempI == INT_INFINITY){
        //すでに目的地近辺にいるとき
        ROS_INFO_STREAM("Maybe Robot Got to the Target\n");
        return;
    }

    existGoal = true;

    //初めに目標と角度合わせる
    
    //目標の角度計算(多分できるはず)
    double goalYaw = qToYaw(path[tempI].pose.orientation);
    // double yaw = qToYaw(poseData.pose.orientation);

    // double MATCH_ANGLE_THRESHOLD = 0.1;
    // double MATCH_ROTATION_VELOCITY = 0.2;

    // geometry_msgs::Twist vel;
    // vel.angular.z = MATCH_ROTATION_VELOCITY;

    directionFitting(goalYaw);

    //移動する
    geometry_msgs::Point tempGoal;
    tempGoal.x = path[tempI].pose.position.x;
    tempGoal.y = path[tempI].pose.position.y;

    //移動の終了条件 pathの最後の座標との距離が閾値以下
    geometry_msgs::Point goal;
    goal.x = path[path.size()-1].pose.position.x;
    goal.y = path[path.size()-1].pose.position.y;

    double goalDistance;
    goalDistance = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));
    //localDistance = sqrt(pow(tempGoal.x - poseData.pose.position.x,2) + pow(tempGoal.y - poseData.pose.position.y,2));

    //const double LOCAL_TOLERANCE = 0.2;

    int tempIOld;

    while(GOAL_TOLERANCE < goalDistance){//最終目標との距離を見る
        while(LOCAL_TOLERANCE < localDistance){//一時的な目標との距離
            //publishToGoal(poseData.pose, tempGoal);
            vfhMovement(false,tempGoal);//目標方向に一回だけpublish
            qPose.callOne(ros::WallDuration(1));
            localDistance = sqrt(pow(path[tempI].pose.position.x - poseData.pose.position.x,2) + pow(path[tempI].pose.position.y - poseData.pose.position.y,2));
        }
        //ここまでで一時的な目標にたどり着いたはず
        //次の目標を検索
        tempIOld = tempI;
        for(int i=tempI+1;i<path.size();i++){
            localDistance = sqrt(pow(path[i].pose.position.x - poseData.pose.position.x,2) + pow(path[i].pose.position.y - poseData.pose.position.y,2));
            if(localDistance > PATH_RADIUS){
                tempI = i;
                break;
            }
        }
        if(tempI == tempIOld){
            //すでに目的地近辺にいる
            ROS_INFO_STREAM("Maybe Robot Got to the Target\n");
            existGoal = false;
            return;
        }
        //新たに一時的な目標をセット
        tempGoal.x = path[tempI].pose.position.x;
        tempGoal.y = path[tempI].pose.position.y;

        goalDistance = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));
    }

    ROS_INFO_STREAM("Maybe Robot Got to the Target\n");

    //publishToGoalDelete();
    existGoal = false;

}

void Movement::directionFitting(double targetYaw){
    geometry_msgs::Twist vel;
    vel.angular.z = MATCH_ROTATION_VELOCITY;

    double yaw = qToYaw(poseData.pose.orientation);

    //角度合わせ
    while(std::abs(targetYaw - yaw) > MATCH_ANGLE_THRESHOLD){
        //回転動作
        pubVelocity.publish(vel);
        qPose.callOne(ros::WallDuration(1));
        yaw = qToYaw(poseData.pose.orientation);
    }
}

void Movement::vfhMovement(bool isStraight, const geometry_msgs::Point& goal){
    double resultAngle;

    qBumper.callOne(ros::WallDuration(1));
    if(!bumperCollision(bumperData)){
        qScan.callOne(ros::WallDuration(1));
        if(isStraight){
            resultAngle = vfhCalculation(scanData,true);
        }
        else{
            resultAngle = vfhCalculation(scanData,false,localAngleCalculation(goal,poseData));
        }
        if((int)resultAngle == INT_INFINITY){
            if(!emergencyAvoidance(scanData)){
                recoveryRotation();
            }
        }
        else{
            velocityPublisher(resultAngle * VELOCITY_GAIN, FORWARD_VELOCITY * VELOCITY_GAIN, VFH_GAIN);
        }
    }
}

void Movement::vfhMovement(bool isStraight, double angle){
    double resultAngle;

    qBumper.callOne(ros::WallDuration(1));
    if(!bumperCollision(bumperData)){
        qScan.callOne(ros::WallDuration(1));
        if(isStraight){
            resultAngle = vfhCalculation(scanData,true);
        }
        else{
            resultAngle = vfhCalculation(scanData,false,angle);
        }
        if((int)resultAngle == INT_INFINITY){
            if(!emergencyAvoidance(scanData)){
                recoveryRotation();
            }
        }
        else{
            velocityPublisher(resultAngle * VELOCITY_GAIN, FORWARD_VELOCITY * VELOCITY_GAIN, VFH_GAIN);
        }
    }
}

bool Movement::bumperCollision(const kobuki_msgs::BumperEvent& bumper){
    //壁に衝突してるかを確認して、してたらバック
    //バックの後に回転動作をさせる
    if(bumperData.state){
        //バック部分
        ROS_WARN_STREAM("Bumper Hit !!\n");
        geometry_msgs::Twist vel;
        vel.linear.x = BACK_VELOCITY;
        ros::Duration duration(BACK_TIME);
        ros::Time setTime = ros::Time::now();

        while(ros::Time::now()-setTime < duration){
            pubVelocity.publish(vel);
        }

        //回転部分
        vel.linear.x = 0;
        switch (bumperData.bumper){
            case 0:
                vel.angular.z = -ROTATION_VELOCITY;
                break;
            case 1:
                vel.angular.z = -previousOrientation * ROTATION_VELOCITY;
                break;
            case 2:
                vel.angular.z = ROTATION_VELOCITY;
                break;
            default:
                break;
        }

        ros::Duration duration2(BUMPER_ROTATION_TIME);
        setTime = ros::Time::now();
        while(ros::Time::now()-setTime < duration2){
            pubVelocity.publish(vel);
        }

        return true;
    }
    return false;
}

double Movement::vfhCalculation(const sensor_msgs::LaserScan& scan, bool isCenter, double goalAngle){
    //要求角度と最も近くなる配列の番号を探索
    //安全な角度マージンの定義
    //要求角度に最も近くなる右側と左側の番号を探索
    ROS_DEBUG_STREAM("VFH Calculation\n");

    ROS_DEBUG_STREAM("Goal Angle : " << goalAngle << " [deg]\n");

    static int centerPosition = 0;
    double diff;
    double min;
    int goalI;

    if(isCenter){
        goalI = scan.ranges.size() / 2 - centerPosition;
        if(centerPosition == 0){
		    centerPosition = 1;
	    }
	    else{
		    centerPosition = 0;
        }
    }
    else{
        min = DOUBLE_INFINITY;
        for(int i=0;i<scan.ranges.size();i++){
		    diff = std::abs(goalAngle - (scan.angle_min + scan.angle_increment * i));
		    if(diff < min){
			    min = diff;
			    goalI = i;
		    }
        }
    }
    const int SAFE_NUM = (asin((SAFE_SPACE)/(2*SAFE_DISTANCE))) / scan.angle_increment ;
    int start;
    int k;
    int count;
    int plus = INT_INFINITY;
    int minus = INT_INFINITY;

    for(int i=goalI;i<scan.ranges.size();i++){
		if(scan.ranges[i] > SCAN_THRESHOLD){
			start = i;
			k = i;
			count = 0;
			while(scan.ranges[k] > SCAN_THRESHOLD && count < SAFE_NUM && k < scan.ranges.size()-1){
				count++;
				k++;
			}
			if(count == SAFE_NUM && start >= SAFE_NUM){
				count = 0;
				for(int j=start;j>start-SAFE_NUM;j--){
					if(scan.ranges[j] > SCAN_THRESHOLD && count < SAFE_NUM){
						count++;
					}
				}
				if(count == SAFE_NUM){
					plus = start;
					break;
				}	
			}
		}
    }

	for(int i=goalI;i>=0;i--){
		if(scan.ranges[i] > SCAN_THRESHOLD){
			start = i;
			k = i;
			count = 0;
			while(scan.ranges[k] > SCAN_THRESHOLD && count < SAFE_NUM && k > 0){
				count++;
				k--;
			}
			if(count == SAFE_NUM && start <= scan.ranges.size()-SAFE_NUM){
				count = 0;
				for(int j=start;j<start+SAFE_NUM;j++){
					if(scan.ranges[j] > SCAN_THRESHOLD && count < SAFE_NUM){
						count++;
					}
				}
				if(count == SAFE_NUM){
					minus = start;
					break;
				}	
			}
		}
	}

    double moveAngle;

    if(plus != INT_INFINITY || minus != INT_INFINITY){
		double pd;
		double md;

		if(plus == INT_INFINITY){
			pd = INT_INFINITY;
		}
        else{
            pd = std::abs((scan.angle_min + scan.angle_increment * goalI) - (scan.angle_min + scan.angle_increment * plus));
        }

		if(minus == INT_INFINITY){
			md = INT_INFINITY;
		}
        else{
            md = std::abs((scan.angle_min + scan.angle_increment * goalI) - (scan.angle_min + scan.angle_increment * minus));
        }

		if(pd<=md){
			moveAngle = scan.angle_min + scan.angle_increment * plus;
		}
		else{
			moveAngle = scan.angle_min + scan.angle_increment * minus;
		}
        ROS_DEBUG_STREAM("Move Angle : " << moveAngle << " [deg]\n");
    }
    else{
        moveAngle = INT_INFINITY;
        ROS_DEBUG_STREAM("Move Angle : Not Found\n");
    }

    return moveAngle;
}

double Movement::localAngleCalculation(const geometry_msgs::Point& goal, const geometry_msgs::PoseStamped& pose){
    double tempAngle;
    double localAngle;

    tempAngle = atan2(goal.y - pose.pose.position.y, goal.x - pose.pose.position.x);
    localAngle = tempAngle - qToYaw(pose.pose.orientation);

    if(localAngle < -M_PI){
        localAngle = 2*M_PI + localAngle;
    }
    if(localAngle > M_PI){
        localAngle = -2*M_PI + localAngle;
    }

    goalDirection = localAngle / std::abs(localAngle);

    return localAngle;
}

double Movement::qToYaw(const geometry_msgs::Quaternion& q){
    tf::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(tq).getRPY(roll,pitch,yaw);
    return yaw;
}

bool Movement::emergencyAvoidance(const sensor_msgs::LaserScan& scan){
    double aveP;
    double aveM;
    double sum = 0;
    static double sign = 0;

    //minus側の平均
    for(int i=0;i<scan.ranges.size()/2;i++){
        if(!std::isnan(scan.ranges[i])){
            sum += scan.ranges[i];
        }
    }
    aveM = sum / (scan.ranges.size()/2);

    //plus側
    sum = 0;
    for(int i=scan.ranges.size()/2;i<scan.ranges.size();i++){
        if(!std::isnan(scan.ranges[i])){
            sum += scan.ranges[i];
        }
    }
    aveP = sum / (scan.ranges.size()/2);

    if(aveP < EMERGENCY_THRESHOLD && aveM < EMERGENCY_THRESHOLD){
        ROS_WARN_STREAM("Close to Obstacles !!\n");
        if(sign > 0){
            ROS_INFO_STREAM("Avoidance to Left\n");
        }
        else{
            ROS_INFO_STREAM("Avoidance to Right\n");
        }
        velocityPublisher(sign * scan.angle_max/6 * VELOCITY_GAIN,0.0,AVOIDANCE_GAIN);
    }
    else{
        //両方向に回避が可能かつゴールが設定されているときはゴール方向に回避
        if(AVOIDANCE_TO_GOAL && existGoal && aveP >= EMERGENCY_THRESHOLD && aveM >= EMERGENCY_THRESHOLD){
            //ここでローカルでのゴールの角度を計算(local_angleの計算でやってる)
            sign = goalDirection;
            ROS_INFO_STREAM("Avoidance to Goal Derection\n");
        }
        else if(aveP > aveM){
            sign = 1.0;
            ROS_INFO_STREAM("Avoidance to Left\n");
        }
        else if(aveP < aveM){
            sign = -1.0;
            ROS_INFO_STREAM("Avoidance to Right\n");
        }
        else{
            return false;
        }
    }
    velocityPublisher(sign*scan.angle_max/6 * VELOCITY_GAIN, FORWARD_VELOCITY * VELOCITY_GAIN, AVOIDANCE_GAIN);

    return true;
}

void Movement::recoveryRotation(void){
    //どうしようもなくなった時に回転する
    ROS_WARN_STREAM("Recovery Rotation !\n");
    geometry_msgs::Twist vel;
    vel.angular.z = -1 * previousOrientation * ROTATION_VELOCITY * VELOCITY_GAIN;
    pubVelocity.publish(vel);
}

void Movement::velocityPublisher(double theta, double v, double t){
    double omega;

    previousOrientation = theta / std::abs(theta);
    t /= ROTATION_GAIN;
    omega = (CURVE_GAIN*theta)/t;
    geometry_msgs::Twist vel;
    vel.linear.x = v;
    vel.angular.z = omega;

    publishMoveAngle(theta,poseData.pose,vel);
    pubVelocity.publish(vel);
    ROS_INFO_STREAM("Publish Velocity\n");
}

void Movement::moveToForward(void){
    ROS_INFO_STREAM("Moving Straight\n");
    geometry_msgs::Point goal;
    double angle;

    qScan.callOne(ros::WallDuration(1));
    qPose.callOne(ros::WallDuration(1));

    if(forwardWallDetection(scanDataOrigin, angle)){
        vfhMovement(false,angle);
    }
    else{
        if(!roadCenterDetection(scanDataOrigin)){
            vfhMovement(true,goal);
        }
    }
    
}

bool Movement::roadCenterDetection(const sensor_msgs::LaserScan& scan){
    std::vector<float> fixRanges;
    std::vector<float> fixAngles;

    fixRanges.reserve(scan.ranges.size());
    fixRanges.reserve(scan.ranges.size());

    for(int i=0;i<scan.ranges.size();i++){
        if(!std::isnan(scan.ranges[i])){
            double tempAngle = scan.angle_min+(scan.angle_increment*i);
            if(scan.ranges[i]*cos(tempAngle) <= ROAD_CENTER_THRESHOLD){
                fixRanges.emplace_back(scan.ranges[i]);
                fixAngles.emplace_back(std::move(tempAngle));
            }
        }
    }

    if(fixRanges.size() < 2){
        return false;
    }

    double diffY;
    double centerAngle;

    for(int i=0;i<fixRanges.size()-1;i++){
        diffY = fixRanges[i+1]*sin(fixAngles[i+1]) - fixRanges[i]*sin(fixAngles[i]);
        if(std::abs(diffY) >= ROAD_THRESHOLD){
            centerAngle = (fixAngles[i]+fixAngles[i+1])/2;
            ROS_DEBUG_STREAM("Road Center Found\n");
            velocityPublisher(centerAngle*VELOCITY_GAIN,FORWARD_VELOCITY*VELOCITY_GAIN,ROAD_CENTER_GAIN);
            return true;
        }
    }
    ROS_DEBUG_STREAM("Road Center Do Not Found\n");
    return false;
}

void Movement::publishToGoal(const geometry_msgs::Pose& pose, const geometry_msgs::Point& goal){
    exploration_msgs::ToGoal msg;

    msg.pose = pose;
    msg.goal = goal;
    msg.header.stamp = ros::Time::now();

    pubToGoal.publish(msg);
    ROS_INFO_STREAM("Publish ToGoal\n");
}

void Movement::publishMoveAngle(double angle, const geometry_msgs::Pose& pose, const geometry_msgs::Twist& vel){
    exploration_msgs::MoveAngle msg;

    msg.local_angle = angle;
    msg.global_angle = angle + qToYaw(pose.orientation);
    msg.pose = pose;
    msg.velocity = vel;
    msg.header.stamp = ros::Time::now();

    pubMoveAngle.publish(msg);
    ROS_INFO_STREAM("Publish MoveAngle\n");
}

void Movement::publishVoronoiMap(const nav_msgs::OccupancyGrid& map){
    pubVoronoiMap.publish(map);
    ROS_DEBUG_STREAM("Publish My Voronoi Map\n");
}

void Movement::publishToGoalDelete(void){
    std_msgs::Empty msg;

    pubToGoalDel.publish(msg);
    //ROS_INFO_STREAM("Publish To Goal Delete\n");
}

void Movement::oneRotation(void){
    //ロボットがz軸周りに一回転する
    ROS_DEBUG_STREAM("rotation\n");

    qPose.callOne(ros::WallDuration(1));

    double initYaw = qToYaw(poseData.pose.orientation);
    double initSign = initYaw / std::abs(initYaw);
    double yaw = initYaw;

    if(std::isnan(initSign)){
	    initSign = 1.0;
    }

    //initYawが+の時は+回転
    //initYawが-の時は-回転
    geometry_msgs::Twist vel;
    vel.angular.z = initSign * ROTATION_VELOCITY;

    int count = 0;
    double yawOld;

    //ROS_DEBUG_STREAM("bool : " << (bool)(count < 3) << ", " << (bool)(count < 2) << ", " << (bool)(std::abs(yaw) < std::abs(initYaw)) << "\n");
    //ROS_DEBUG_STREAM("yaw : " << yaw << ", count : " << count << ", initYaw : " << initYaw << ", initSign : " << initSign << "\n");
    
    //yawの符号が３回変わる、または２回変わった後initYawより絶対値が大きくなったら
    while((count < 3 && (count < 2 || std::abs(yaw) < std::abs(initYaw))) && ros::ok()){
        yawOld = yaw;
        pubVelocity.publish(vel);
        qPose.callOne(ros::WallDuration(1));
        yaw = qToYaw(poseData.pose.orientation);
        if(yawOld * yaw < 0){
            count++;
        }
        //ROS_DEBUG_STREAM("bool : " << (bool)(count < 3) << ", " << (bool)(count < 2) << ", " << (bool)(std::abs(yaw) < std::abs(initYaw)) << "\n");
        //ROS_DEBUG_STREAM("yaw : " << yaw << ", count : " << count << ", initYaw : " << initYaw << ", initSign : " << initSign << "\n");
    }
}

bool Movement::callPathPlanner(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path){
    //navfnなどのグローバルパスプラナーを呼ぶ
    if(PLANNER_NAME == "NavfnROS"){
        ROS_INFO_STREAM("call Navfn\n");
        return callNavfn(COSTMAP_NAME,PLANNER_NAME,start,goal,path);
    }
    else if(PLANNER_NAME == "VoronoiPlanner"){
        ROS_INFO_STREAM("call VoronoiPlanner\n");
        return callVoronoiPlanner(COSTMAP_NAME,PLANNER_NAME,start,goal,path);
    }
    else{
        ROS_ERROR_STREAM("Unknown Planner Name!\n");
        return false;
    }
    // switch (PLANNER_NAME){
    //     case 'NavfnROS':
    //         ROS_INFO_STREAM("call Navfn\n");
    //         return callNavfn(COSTMAP_NAME,PLANNER_NAME,start,goal,path);
    //     case 'VoronoiPlanner':
    //         ROS_INFO_STREAM("call VoronoiPlanner\n");
    //         return callVoronoiPlanner(COSTMAP_NAME,PLANNER_NAME,start,goal,path);
    //     default:
    //         ROS_ERROR_STREAM("planner name is unknown\n");
    //         return false;
    // }
}

bool Movement::callNavfn(const std::string& costmapName,const std::string& plannerName,const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path){
    //static pathPlanning<navfn::NavfnROS> pp(plannerName);
    static pathPlanning<navfn::NavfnROS> pp(costmapName,plannerName);
    return pp.createPath(start,goal,path);
}

bool Movement::callVoronoiPlanner(const std::string& costmapName,const std::string& plannerName,const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& path){
    ////pathPlanning<voronoi_planner::VoronoiPlanner> pp(plannerName);
    //static pathPlanning<voronoi_planner::VoronoiPlanner> pp(costmapName,plannerName);
    pathPlanning<voronoi_planner::VoronoiPlanner> pp(costmapName,plannerName);
    if(PUBLISH_MY_VORONOI){
        nav_msgs::OccupancyGrid map;
        //bool success = pp.createPath(start,goal,path,map,true);
        bool success = pp.createPath(start,goal,path,map,false);
        publishVoronoiMap(map);
        return success;
    }
    else{
        //return pp.createPath(start,goal,path,true);
        return pp.createPath(start,goal,path,false);
    }
}

bool Movement::forwardWallDetection(const sensor_msgs::LaserScan& scan, double& angle){
    //前方に壁があるかどうかを判定する
    //前方何度まで見るかを決める
    //その範囲にセンサデータが何割存在するかで壁かどうか決める
    //壁があった場合右と左のどっちに避けるか決定


    const int CENTER_SUBSCRIPT = scan.ranges.size()/2;

    int PLUS = CENTER_SUBSCRIPT + (int)(FORWARD_ANGLE/scan.angle_increment);
    int MINUS = CENTER_SUBSCRIPT - (int)(FORWARD_ANGLE/scan.angle_increment);

    //ROS_INFO_STREAM("ranges.size() : " << scan.ranges.size() << ", CENTER_SUBSCRIPT : " << CENTER_SUBSCRIPT << ", calc : " << (int)(FORWARD_ANGLE/scan.angle_increment) << "\n");

    int count = 0;
    double sum = 0;

    for(int i=MINUS;i<PLUS;i++){
        if(!std::isnan(scan.ranges[i])){
            count++;
            sum += scan.ranges[i];
        }
    }
    //ROS_INFO_STREAM("PLUS : " << PLUS << ", MINUS : " << MINUS << ", count : " << count << ", rate : " << (double)count/(PLUS-MINUS) << "\n");

    double wallDistance = sum/count;

    //壁の割合が少ないときと、壁までの距離が遠い時は検出判定をしない
    //追加で壁に近くなりすぎると判定ができなくなるので無効とする
    if((double)count/(PLUS-MINUS) > WALL_RATE_THRESHOLD && wallDistance < WALL_DISTANCE_UPPER_THRESHOLD && wallDistance > WALL_DISTANCE_LOWER_THRESHOLD){
        ROS_INFO_STREAM("Wall Found : " << wallDistance << " [m]\n");
        angle = sideSpaceDetection(scan,PLUS, MINUS);
        return true;
    }
    else{
        ROS_INFO_STREAM("Wall not Found or Out of Range\n");
        return false;
    }
}

double Movement::sideSpaceDetection(const sensor_msgs::LaserScan& scan, int plus, int minus){
    //minus
    double sumMinus = 0;
    int countNanMinus = 0;
    double maxSpaceMinus = 0;
    double temp;
    for(int i=0;i<minus;++i){
        if(!std::isnan(scan.ranges[i])){
            sumMinus += scan.ranges[i];
            for(int j=i+1;j<minus;++j){
                if(!std::isnan(scan.ranges[j])){
                    temp = std::abs(cos(scan.ranges[i])-cos(scan.ranges[j]));
                    break;
                }
            }
            //temp = std::abs(cos(scan.ranges[i])-cos(scan.ranges[i+1]));//これだとi+1がnanで判定するのでダメ
            if(temp > maxSpaceMinus){
                maxSpaceMinus = temp;
            }
        }
        else{
            countNanMinus++;
        }
    }

    //plus
    double sumPlus = 0;
    int countNanPlus = 0;
    double maxSpacePlus = 0;
    for(int i=plus;i<scan.ranges.size();++i){
        if(!std::isnan(scan.ranges[i])){
            sumPlus += scan.ranges[i];
            for(int j=i-1;j>=0;--j){
                if(!std::isnan(scan.ranges[j])){
                    temp = std::abs(cos(scan.ranges[i])-cos(scan.ranges[j]));
                    break;
                }
            }
            //temp = std::abs(cos(scan.ranges[i])-cos(scan.ranges[i-1]));
            if(temp > maxSpacePlus){
                maxSpacePlus = temp;
            }
        }
        else{
            countNanPlus++;
        }
    }

    //ROS_INFO_STREAM("minus : " << minus << ", sum range : " << sumMinus << ", ave range : " << sumMinus/(minus - countNanMinus) << ", Nan count : " << countNanMinus << ", true count : " << minus - countNanMinus << ", space : " << maxSpaceMinus << "\n");    
    //ROS_INFO_STREAM("plus : " << plus << ", sum range : " << sumPlus << ", ave range : " << sumPlus/(scan.ranges.size() - plus - countNanPlus) << ", Nan count : " << countNanPlus << ", true count : " << scan.ranges.size() - plus - countNanPlus << ", space" << maxSpacePlus << "\n");

    double aveMinus = sumMinus/(minus - countNanMinus);
    double avePlus = sumPlus/(scan.ranges.size() - plus - countNanPlus);
    
    //不確定 //壁までの距離が遠いときは平均距離が長いほうが良い、近いときは開いてる領域が大きい方が良い
    if(maxSpaceMinus > maxSpacePlus && aveMinus > EMERGENCY_THRESHOLD){
        ROS_INFO_STREAM("Found Right Space\n");
        return (scan.angle_min + (scan.angle_increment * (scan.ranges.size()/2+minus)/2))/2;
    }
    else if(maxSpacePlus > maxSpaceMinus && avePlus > EMERGENCY_THRESHOLD){
        ROS_INFO_STREAM("Found Left Space\n");
        return (scan.angle_min + (scan.angle_increment * (plus + scan.ranges.size()/2)/2))/2;
    }
    else{
        ROS_INFO_STREAM("Not Found Space\n");
        return 0;
    }

    //平均距離が長い方にスペースがある//ダメでした
    // if(aveMinus > avePlus && aveMinus > EMERGENCY_THRESHOLD){
    //     ROS_INFO_STREAM("Found Right Space\n");
    //     return scan.angle_min + (scan.angle_increment * (scan.ranges.size()/2+minus)/2);
    // }
    // else if(avePlus > aveMinus && avePlus > EMERGENCY_THRESHOLD){
    //     ROS_INFO_STREAM("Found Left Space\n");
    //     return scan.angle_min + (scan.angle_increment * (plus + scan.ranges.size()/2)/2);
    // }
    // else{
    //     ROS_INFO_STREAM("Not Found Space\n");
    //     return 0;
    // }
    //不確定 //壁までの距離が遠いときは平均距離が長いほうが良い、近いときは開いてる領域が大きい方が良い
    //開いてる領域が大きい方
}

void Movement::functionCallTester(void){
    qScan.callOne(ros::WallDuration(1));

    double angle;
    if(forwardWallDetection(scanDataOrigin,angle)){
        ROS_INFO_STREAM("Return Angle : " << angle << " [rad], " << angle*180/M_PI << " [deg]" << "\n");
    }
    std::cout << std::endl; 
}
#endif //MOVEMENT_H
