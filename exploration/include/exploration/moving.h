#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>

#include <exploration/ToGoal.h>
#include <exploration/MoveAngle.h>

//topic名はパラメータで渡すのではなくremapしても良いかも

//センサーデータを受け取った後にロボットの動作を決定する
//障害物回避を含む
class Moving 
{
private:
    ros::NodeHandle p;
    double GOAL_MARGIN;
    //double GRAVITY_ENABLE;
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

    int INFINITY_NUMBER;

    ros::NodeHandle ss;
    ros::Subscriber subScan;
    ros::CallbackQueue qScan;
    std::string scanTopic;
    sensor_msgs::LaserScan scanData;
    sensor_msgs::LaserScan scanDataOrigin;

    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    std::string poseTopic;
    geometry_msgs::PoseStamped poseData;

    ros::NodeHandle sb;
    ros::Subscriber subBumper;
    ros::CallbackQueue qBumper;
    std::string bumperTopic;
    kobuki_msgs::BumperEvent bumperData;

    ros::NodeHandle pv;
    ros::Publisher pubVelocity;
    std::string velocityTopic;

    ros::NodeHandle ptg;
    ros::Publisher pubToGoal;
    std::string ToGoalTopic;

    ros::NodeHandle pma;
    ros::Publisher pubMoveAngle;
    std::string moveAngleTopic;

    double previousOrientation;
    
    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void approx(std::vector<float>& scan);
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    double vfhCalculation(sensor_msgs::LaserScan scan, bool isCenter, double goalAngle);
    double localAngleCalculation(geometry_msgs::Point goal,geometry_msgs::PoseStamped pose);

    bool bumperCollision(kobuki_msgs::BumperEvent bumper);
    void rescueRotation(void);
    void velocityPublisher(double theta, double v, double t);
    bool emergencyAvoidance(sensor_msgs::LaserScan scan);
    bool roadCenterDetection(sensor_msgs::LaserScan scan);

    void vfhMovement(bool isStraight, geometry_msgs::Point goal);

    void publishMoveAngle(double angle, geometry_msgs::Pose pose, geometry_msgs::Twist vel);
    void publishToGoal(geometry_msgs::Pose pose, geometry_msgs::Point goal);

public:
    Moving();
    ~Moving(){};

    void moveToGoal(geometry_msgs::Point goal);
    void moveToForward(void);
};

Moving::Moving(){
    p.param<double>("goal_margin", GOAL_MARGIN, 0.5);
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

    p.param<std::string>("scan_topic", scanTopic, "scan");
    ss.setCallbackQueue(&qScan);
    subScan = ss.subscribe(scanTopic,1,&Moving::scanCB, this);

    p.param<std::string>("pose_topic", poseTopic, "pose");
    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe(poseTopic,1,&Moving::poseCB,this);

    p.param<std::string>("bumper_topic", bumperTopic, "bumper");
    sb.setCallbackQueue(&qBumper);
    subBumper = sb.subscribe(bumperTopic,1,&Moving::bumperCB,this);

    p.param<std::string>("velocity_topic", velocityTopic, "velocity");
    pubVelocity = pv.advertise<geometry_msgs::Twist>(velocityTopic, 1);

    p.param<std::string>("to_goal_topic", ToGoalTopic, "to_goal");
    pubToGoal = ptg.advertise<exploration::ToGoal>(ToGoalTopic, 1);

    p.param<std::string>("move_angle_topic", moveAngleTopic, "move_angle");
    pubMoveAngle = pma.advertise<exploration::MoveAngle>(moveAngleTopic, 1);

    INFINITY_NUMBER = 1000000;
}

void Moving::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    scanData = *msg;
    scanDataOrigin = *msg;
    approx(scanData.ranges);
}

void Moving::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

void Moving::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumperData = *msg;
}

void Moving::approx(std::vector<float>& scan){
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

void Moving::publishToGoal(geometry_msgs::Pose pose, geometry_msgs::Point goal){
    exploration::ToGoal msg;

    msg.pose = pose;
    msg.goal = goal;

    msg.header.stamp = ros::Time::now();

    pubToGoal.publish(msg);
}

void Moving::publishMoveAngle(double angle, geometry_msgs::Pose pose, geometry_msgs::Twist vel){
    exploration::MoveAngle msg;

    msg.localAngle = angle;
    msg.pose = pose;
    msg.velocity = vel;

    msg.header.stamp = ros::Time::now();

    pubMoveAngle.publish(msg);
}

void Moving::moveToGoal(geometry_msgs::Point goal){

    qPose.callOne(ros::WallDuration(1));

    const double GRAVITY_ENABLE = std::abs(goal.y - poseData.pose.position.y) + GRAVITY_GAIN;

    double distToGoal;
    double distToGoalOld;
    double diff = 0;
    int count = 0;
    const int END_COUNT = 1;
    int sign = -1;

    distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));

    geometry_msgs::Point nullGoal;

    //このループいらないかも要検討
    while(GRAVITY_ENABLE < distToGoal && distToGoal < GRAVITY_FORCE_ENABLE && ros::ok()){
        publishToGoal(poseData.pose, goal);
        distToGoalOld = distToGoal;
        vfhMovement(true,nullGoal);
        qPose.callOne(ros::WallDuration(1));
        distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));
        diff += distToGoal - distToGoalOld;
        if(std::abs(diff) > GRAVITH_DIFF_THRESHOLD){
            if(diff*sign < 0){
                sign *= -1;
                count++;
                if(count == END_COUNT){
                    diff = 0;
                    count = 0;
                    sign = -1;
                    break;
                }
            }
            diff = 0;
        }
    }

    while(GOAL_MARGIN < distToGoal && ros::ok()){
        publishToGoal(poseData.pose, goal);
        distToGoalOld = distToGoal;
        vfhMovement(false,goal);
        qPose.callOne(ros::WallDuration(1));
        distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));
        diff += distToGoal - distToGoalOld;
        if(std::abs(diff) > GRAVITH_DIFF_THRESHOLD){
            if(diff*sign < 0){
                sign *= -1;
                count++;
                if(count == END_COUNT){
                    diff = 0;
                    count = 0;
                    sign = -1;
                    break;
                }
            }
            diff = 0;
        }
    }
}

double Moving::vfhCalculation(sensor_msgs::LaserScan scan, bool isCenter, double goalAngle=0.0){
    //要求角度と最も近くなる配列の番号を探索
    //安全な角度マージンの定義
    //要求角度に最も近くなる右側と左側の番号を探索

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
    int plus = INFINITY_NUMBER;
    int minus = INFINITY_NUMBER;


    for(int i=goalI;i<scan.ranges.size();i++){
		//std::cout << "plusのfor,i:" << i << ",ranges[i]:" << ranges[i] << std::endl;
		if(scan.ranges[i] > SCAN_THRESHOLD){
			//std::cout << "正方向" << std::endl;
			start = i;
			k = i;
			count = 0;
			while(scan.ranges[k] > SCAN_THRESHOLD && count < SAFE_NUM && k < scan.ranges.size()-1){//iの限界
				//std::cout << "sp_ranges[i]:" << ranges[i] << std::endl;
				count++;
				k++;
			}
			//std::cout << "rad_counter1:" << rad_counter << std::endl;
			if(count == SAFE_NUM && start >= SAFE_NUM){//プラス側はOKなのでマイナス側を見に行く
				//std::cout << "逆方向" << std::endl;
				count = 0;
				for(int j=start;j>start-SAFE_NUM;j--){
					if(scan.ranges[j] > SCAN_THRESHOLD && count < SAFE_NUM){
						//std::cout << "tp_ranges[i]:" << ranges[i] << std::endl;
						count++;
					}
					//std::cout << "j:" << j << std::endl;
				}
				//std::cout << "rad_counter2:" << rad_counter << std::endl;
				if(count == SAFE_NUM){
					plus = start;
					//std::cout << "breakした" << std::endl;
					break;
				}	
			}
		}
    }

    //std::cout << "plusのfor抜けた" << std::endl;

	for(int i=goalI;i>=0;i--){
		//std::cout << "minusのfor,i:" << i << ",ranges[i]:" << ranges[i] << std::endl;
		if(scan.ranges[i] > SCAN_THRESHOLD){
			//std::cout << "正方向" << std::endl;
			start = i;
			k = i;
			count = 0;
			while(scan.ranges[k] > SCAN_THRESHOLD && count < SAFE_NUM && k > 0){
				//std::cout << "sm_ranges[i]:" << ranges[l] << std::endl;
				count++;
				k--;
			}
			//std::cout << "rad_counter1:" << rad_counter << std::endl;
			if(count == SAFE_NUM && start <= scan.ranges.size()-SAFE_NUM){//マイナス側はOKなのでプラス側を見に行く//ここやってない//やった
				//std::cout << "逆方向" << std::endl;
				count = 0;
				for(int j=start;j<start+SAFE_NUM;j++){
					if(scan.ranges[j] > SCAN_THRESHOLD && count < SAFE_NUM){
						//std::cout << "tm_ranges[i]:" << ranges[i] << std::endl;
						count++;
					}
				}
				//std::cout << "rad_counter2:" << rad_counter << std::endl;
				if(count == SAFE_NUM){
					minus = start;
					//std::cout << "breakした" << std::endl;
					break;
				}	
			}
		}
	}
    //std::cout << "minusのfor抜けた" << std::endl;

    double moveAngle;

    if(plus != INFINITY_NUMBER || minus != INFINITY_NUMBER){
		double pd;
		double md;

		if(plus == INFINITY_NUMBER){
			pd = INFINITY_NUMBER;
		}
        else{
            pd = std::abs((scan.angle_min + scan.angle_increment * goalI) - (scan.angle_min + scan.angle_increment * plus));
        }

		if(minus == INFINITY_NUMBER){
			md = INFINITY_NUMBER;
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
    }
    else{
        moveAngle = INFINITY_NUMBER;
    }

    return moveAngle;

}

double Moving::localAngleCalculation(geometry_msgs::Point goal,geometry_msgs::PoseStamped pose){
    double tempAngle;
    double localAngle;

    tempAngle = atan2(goal.y - pose.pose.position.y, goal.x - pose.pose.position.x);

    localAngle = tempAngle - 2*asin(pose.pose.orientation.z);

    if(localAngle < -M_PI){
        localAngle = 2*M_PI + localAngle;
    }
    if(localAngle > M_PI){
        localAngle = -2*M_PI + localAngle;
    }

    return localAngle;

}

bool Moving::bumperCollision(kobuki_msgs::BumperEvent bumper){
    //壁に衝突してるかを確認して、してたらバック
    if(bumperData.state){
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

void Moving::rescueRotation(void){
    //どうしようもなくなった時に回転する
    geometry_msgs::Twist vel;
    vel.angular.z = -1 * previousOrientation * ROTATION_VELOCITY;
    pubVelocity.publish(vel);
}

void Moving::velocityPublisher(double theta, double v, double t){
    double omega;

    previousOrientation = theta / std::abs(theta);

    omega = (2*theta)/t;

    geometry_msgs::Twist vel;

    vel.linear.x = v;
    vel.angular.z = omega;

    publishMoveAngle(theta,poseData.pose,vel);

    pubVelocity.publish(vel);
}

bool Moving::emergencyAvoidance(sensor_msgs::LaserScan scan){

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
        velocityPublisher(sign * scan.angle_max/6,0.0,0.3);
    }
    else{
        if(aveP > aveM){
            sign = 1.0;
            velocityPublisher(sign*scan.angle_max/6,FORWARD_VELOCITY,0.3);
        }
        else if(aveP < aveM){
            sign = -1.0;
            velocityPublisher(sign*scan.angle_max/6,FORWARD_VELOCITY,0.3);
        }
        else{
            return false;
        }
    }

    return true;
}

bool Moving::roadCenterDetection(sensor_msgs::LaserScan scan){

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
            velocityPublisher(centerAngle,FORWARD_VELOCITY,0.8);
            return true;
        }
    }

    return false;
}

void Moving::moveToForward(void){

    geometry_msgs::Point nullGoal;

    qScan.callOne(ros::WallDuration(1));
    if(!roadCenterDetection(scanDataOrigin)){
        vfhMovement(true,nullGoal);
    }
}

void Moving::vfhMovement(bool isStraight, geometry_msgs::Point goal){

    double resultAngle;

    qBumper.callOne(ros::WallDuration(1));
    if(!bumperCollision(bumperData)){
        qScan.callOne(ros::WallDuration(1));
        if(isStraight){
            resultAngle = vfhCalculation(scanData,true);
        }
        else{
            qPose.callOne(ros::WallDuration(1));
            resultAngle = vfhCalculation(scanData,false,localAngleCalculation(goal,poseData));
        }
        
        if((int)resultAngle == INFINITY_NUMBER){
            if(!emergencyAvoidance(scanData)){
                rescueRotation();
            }
        }
        else{
            velocityPublisher(resultAngle,FORWARD_VELOCITY,0.5);
        }
    }
}

// void Moving::movingLoop(geometry_msgs::Point goal){

//     qPose.callOne(ros::WallDuration(1));

//     const double GRAVITY_ENABLE = std::abs(goal.y - poseData.pose.position.y) + GRAVITY_GAIN;

//     double distToGoal;
//     double distToGoalOld;
//     double diff = 0;
//     int count = 0;
//     const int END_COUNT = 1;
//     int sign = -1;
//     double resultAngle;

//     distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));

//     //このループいらないかも要検討
//     while(GRAVITY_ENABLE < distToGoal && distToGoal < GRAVITY_FORCE_ENABLE && ros::ok()){
//         distToGoalOld = distToGoal;
//         qBumper.callOne(ros::WallDuration(1));
//         if(!bumperCollision(bumperData)){
//             qScan.callOne(ros::WallDuration(1));
//             resultAngle = vfhCalculation(scanData,true);
//             if((int)resultAngle == INFINITY_NUMBER){
//                 if(!emergencyAvoidance(scanData)){
//                     rescueRotation();
//                 }
//             }
//             else{
//                 velocityPublisher(resultAngle,FORWARD_VELOCITY,0.5);
//             }
//         }
//         qPose.callOne(ros::WallDuration(1));
//         distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));

//         diff += distToGoal - distToGoalOld;

//         if(std::abs(diff) > GRAVITH_DIFF_THRESHOLD){
//             if(diff*sign < 0){
//                 sign *= -1;
//                 count++;
//                 if(count == END_COUNT){
//                     diff = 0;
//                     count = 0;
//                     sign = -1;
//                     break;
//                 }
//             }
//             diff = 0;
//         }
//     }

//     while(GOAL_MARGIN < distToGoal && ros::ok()){
//         distToGoalOld = distToGoal;
//         qBumper.callOne(ros::WallDuration(1));
//         if(!bumperCollision(bumperData)){
//             qScan.callOne(ros::WallDuration(1));
//             qPose.callOne(ros::WallDuration(1));
//             resultAngle = vfhCalculation(scanData,false,localAngleCalculation(goal,poseData));
//             if((int)resultAngle == INFINITY_NUMBER){
//                 if(!emergencyAvoidance(scanData)){
//                     rescueRotation();
//                 }
//             }
//             else{
//                 velocityPublisher(resultAngle,FORWARD_VELOCITY,0.5);
//             }
//         }
//         qPose.callOne(ros::WallDuration(1));
//         distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));

//         diff += distToGoal - distToGoalOld;

//         if(std::abs(diff) > GRAVITH_DIFF_THRESHOLD){
//             if(diff*sign < 0){
//                 sign *= -1;
//                 count++;
//                 if(count == END_COUNT){
//                     diff = 0;
//                     count = 0;
//                     sign = -1;
//                     break;
//                 }
//             }
//             diff = 0;
//         }
//     }

// }