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

//センサーデータを受け取った後にロボットの動作を決定する
//障害物回避を含む
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
    double ROTATION_VELOCITY;
    double EMERGENCY_THRESHOLD;
    double ROAD_CENTER_THRESHOLD;
    double ROAD_THRESHOLD;
    double CURVE_GAIN;
    int TRY_COUNT;
    bool AVOIDANCE_TO_GOAL;

    int INT_INFINITY;
    double DOUBLE_INFINITY;

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

    double previousOrientation;
    double goalDirection;
    bool existGoal;
    
    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void approx(std::vector<float>& scan);
    void vfhMovement(bool isStraight, geometry_msgs::Point goal);
    bool bumperCollision(kobuki_msgs::BumperEvent bumper);
    double vfhCalculation(sensor_msgs::LaserScan scan, bool isCenter, double goalAngle = 0.0);
    double localAngleCalculation(geometry_msgs::Point goal,geometry_msgs::PoseStamped pose);
    double qToYaw(geometry_msgs::Quaternion q);
    bool emergencyAvoidance(sensor_msgs::LaserScan scan, geometry_msgs::Point goal);
    void recoveryRotation(void);
    void velocityPublisher(double theta, double v, double t);
    bool roadCenterDetection(sensor_msgs::LaserScan scan);
    void publishToGoal(geometry_msgs::Pose pose, geometry_msgs::Point goal);
    void publishMoveAngle(double angle, geometry_msgs::Pose pose, geometry_msgs::Twist vel);
    void publishToGoalDelete(void);

public:
    Movement();
    ~Movement(){};

    void moveToGoal(geometry_msgs::Point goal);
    void moveToGoal(std::vector<geometry_msgs::PoseStamped> path);
    void moveToForward(void);
};

Movement::Movement():p("~"){
    p.param<double>("goal_margin", GOAL_TOLERANCE, 0.5);
    p.param<double>("gravity_gain", GRAVITY_GAIN, 1.2);
    p.param<double>("gravity_force_enable", GRAVITY_FORCE_ENABLE, 6.0);
    p.param<double>("gravity_diff_threshold", GRAVITH_DIFF_THRESHOLD, 0.1);
    p.param<double>("safe_distance", SAFE_DISTANCE, 0.75);
    p.param<double>("safe_space", SAFE_SPACE, 0.6);
    p.param<double>("scan_threshold", SCAN_THRESHOLD, 1.2);
    p.param<double>("forward_velocity", FORWARD_VELOCITY, 0.2);
    p.param<double>("back_velocity", BACK_VELOCITY, -0.2);
    p.param<double>("back_time", BACK_TIME, 0.5);
    p.param<double>("rotation_velocity", ROTATION_VELOCITY, 0.5);
    p.param<double>("emergency_threshold", EMERGENCY_THRESHOLD, 1.0);
    p.param<double>("road_center_threshold", ROAD_CENTER_THRESHOLD, 5.0);
    p.param<double>("road_threshold", ROAD_THRESHOLD, 1.5);
    p.param<double>("curve_gain", CURVE_GAIN, 2.0);
    p.param<int>("try_count", TRY_COUNT, 1);
    p.param<bool>("avoidance_to_goal", AVOIDANCE_TO_GOAL, true);

    ss.setCallbackQueue(&qScan);
    subScan = ss.subscribe("scan",1,&Movement::scanCB, this);

    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&Movement::poseCB,this);

    sb.setCallbackQueue(&qBumper);
    subBumper = sb.subscribe("bumper",1,&Movement::bumperCB,this);

    pubVelocity = pv.advertise<geometry_msgs::Twist>("velocity", 1);
    pubToGoal = ptg.advertise<exploration_msgs::ToGoal>("to_goal", 1);
    pubMoveAngle = pma.advertise<exploration_msgs::MoveAngle>("move_angle", 1);
    pubToGoalDel = ptgd.advertise<std_msgs::Empty>("to_goal/delete", 1);

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

void Movement::moveToGoal(geometry_msgs::Point goal){
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

void Movement::moveToGoal(std::vector<geometry_msgs::PoseStamped> path){
    //planの中で半径x[m]より外で一番近いところを探す
    //初めに目標と角度合わせる
    //目標近くについたらplanの中で半径x[m]より外で次に近いところを探す
    //以下ループ

    ROS_INFO_STREAM("Path Recieved\n");

    qPose.callOne(ros::WallDuration(1));

    //double yaw = qToYaw(poseData.pose.orientation);

    const double PATH_RADIUS = 0.5;

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
    double yaw = qToYaw(poseData.pose.orientation);

    const double MATCH_ANGLE_THRESHOLD = 0.1;
    const double MATCH_ROTATION_VELOCITY = 0.2;

    geometry_msgs::Twist vel;
    vel.angular.z = MATCH_ROTATION_VELOCITY;

    //角度合わせ
    while(std::abs(goalYaw - yaw) > MATCH_ANGLE_THRESHOLD){
        //回転動作
        pubVelocity.publish(vel);
        qPose.callOne(ros::WallDuration(1));
        yaw = qToYaw(poseData.pose.orientation);
    }

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

    const double LOCAL_TOLERANCE = 0.2;

    int tempIOld;

    while(GOAL_TOLERANCE < goalDistance){//最終目標との距離を見る
        while(LOCAL_TOLERANCE < localDistance){//一時的な目標との距離
            publishToGoal(poseData.pose, tempGoal);
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

    publishToGoalDelete();
    existGoal = false;

}

void Movement::vfhMovement(bool isStraight, geometry_msgs::Point goal){
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
            if(!emergencyAvoidance(scanData,goal)){
                recoveryRotation();
            }
        }
        else{
            velocityPublisher(resultAngle,FORWARD_VELOCITY,0.5);
        }
    }
}

bool Movement::bumperCollision(kobuki_msgs::BumperEvent bumper){
    //壁に衝突してるかを確認して、してたらバック
    if(bumperData.state){
        ROS_WARN_STREAM("Bumper Hit !!\n");
        geometry_msgs::Twist vel;
        vel.linear.x = BACK_VELOCITY;
        ros::Duration duration(BACK_TIME);
        ros::Time setTime = ros::Time::now();

        while(ros::Time::now()-setTime < duration){
            pubVelocity.publish(vel);
        }
        return true;
    }
    return false;
}

double Movement::vfhCalculation(sensor_msgs::LaserScan scan, bool isCenter, double goalAngle){
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

double Movement::localAngleCalculation(geometry_msgs::Point goal,geometry_msgs::PoseStamped pose){
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

double Movement::qToYaw(geometry_msgs::Quaternion q){
    tf::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(tq).getRPY(roll,pitch,yaw);
    return yaw;
}

bool Movement::emergencyAvoidance(sensor_msgs::LaserScan scan, geometry_msgs::Point goal){
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
        velocityPublisher(sign * scan.angle_max/6,0.0,0.3);
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
    velocityPublisher(sign*scan.angle_max/6,FORWARD_VELOCITY,0.3);

    return true;
}

void Movement::recoveryRotation(void){
    //どうしようもなくなった時に回転する
    ROS_WARN_STREAM("Recovery Rotation !\n");
    geometry_msgs::Twist vel;
    vel.angular.z = -1 * previousOrientation * ROTATION_VELOCITY;
    pubVelocity.publish(vel);
}

void Movement::velocityPublisher(double theta, double v, double t){
    double omega;

    previousOrientation = theta / std::abs(theta);
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
    geometry_msgs::Point nullGoal;

    qScan.callOne(ros::WallDuration(1));
    qPose.callOne(ros::WallDuration(1));
    if(!roadCenterDetection(scanDataOrigin)){
        vfhMovement(true,nullGoal);
    }
}

bool Movement::roadCenterDetection(sensor_msgs::LaserScan scan){
    std::vector<float> fixRanges;
    std::vector<float> fixAngles;

    for(int i=0;i<scan.ranges.size();i++){
        if(!std::isnan(scan.ranges[i])){
            if(scan.ranges[i]*cos(scan.angle_min+(scan.angle_increment*i)) <= ROAD_CENTER_THRESHOLD){
                fixRanges.push_back(scan.ranges[i]);
                fixAngles.push_back(scan.angle_min+(scan.angle_increment*i));
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
            velocityPublisher(centerAngle,FORWARD_VELOCITY,0.8);
            return true;
        }
    }
    ROS_DEBUG_STREAM("Road Center Do Not Found\n");
    return false;
}

void Movement::publishToGoal(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    exploration_msgs::ToGoal msg;

    msg.pose = pose;
    msg.goal = goal;
    msg.header.stamp = ros::Time::now();

    pubToGoal.publish(msg);
    ROS_INFO_STREAM("Publish ToGoal\n");
}

void Movement::publishMoveAngle(double angle, geometry_msgs::Pose pose, geometry_msgs::Twist vel){
    exploration_msgs::MoveAngle msg;

    msg.local_angle = angle;
    msg.global_angle = angle + qToYaw(pose.orientation);
    msg.pose = pose;
    msg.velocity = vel;
    msg.header.stamp = ros::Time::now();

    pubMoveAngle.publish(msg);
    ROS_INFO_STREAM("Publish MoveAngle\n");
}

void Movement::publishToGoalDelete(void){
    std_msgs::Empty msg;

    pubToGoalDel.publish(msg);
    ROS_INFO_STREAM("Publish To Goal Delete\n");
}

#endif //MOVEMENT_H