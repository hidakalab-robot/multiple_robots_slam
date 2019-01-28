#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>

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

    int INFINITY_NUM;

    ros::NodeHandle ss;
    ros::Subscriber subScan;
    ros::CallbackQueue qScan;
    std::string scanTopic;
    sensor_msgs::LaserScan scanData;


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

    double thetaOld;

    void movingLoop(geometry_msgs::Point goal);
    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void approx(std::vector<float>& scan);
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    double vfhCalculate(sensor_msgs::LaserScan scan, double goalAngle);

    double angleGtoR(double angle, geometry_msgs::PoseStamped poseG);

    void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg);

    bool bumperCollision(void);

    void rescueRotation(void);

    void velocityPublisher(double theta, double v, double t);

    bool emergencyAvoidance(void);

    bool roadCenterSearch(void);

public:
    Moving();
    ~Moving(){};
};

Moving::Moving(){
    p.param<double>("goal_margin", GOAL_MARGIN, 0.5);
    p.param<double>("gravity_gain", GRAVITY_GAIN, 1.2);
    p.param<double>("gravity_force_enable", GRAVITY_FORCE_ENABLE, 6.0);
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

    p.param<std::string>("velocity_topic", velocityTopic, "cmd_vel");
    pubVelocity = pv.advertise<geometry_msgs::Twist>(velocityTopic, 1);

    INFINITY_NUM = 1000000;
}

void Moving::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    scanData = *msg;
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

void Moving::movingLoop(geometry_msgs::Point goal){

    qPose.callOne(ros::WallDuration(1));

    const double GRAVITY_ENABLE = std::abs(goal.y - poseData.pose.position.y) + GRAVITY_GAIN;

    double distToGoal;
    double distToGoalOld;

    double diff = 0;
    int count = 0;
    const int END_COUNT = 1;
    int sign = -1;


    distToGoal = sqrt(pow(goal.x - poseData.pose.position.x,2) + pow(goal.y - poseData.pose.position.y,2));

    while(distToGoal > GRAVITY_ENABLE && distToGoal < GRAVITY_FORCE_ENABLE && ros::ok()){

    }



}

double Moving::vfhCalculate(sensor_msgs::LaserScan scan, double goalAngle){
    //要求角度と最も近くなる配列の番号を探索
    //安全な角度マージンの定義
    //要求角度に最も近くなる右側と左側の番号を探索

    static int centerPosition = 0;

    double diff;
    double min;
    int goalI;

    if((int)goalAngle == 0){
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
    int plus;
    int minus;


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

    if(plus != scan.ranges.size() || minus != scan.ranges.size()){
		double pd = std::abs((scan.angle_min + scan.angle_increment * goalI) - (scan.angle_min + scan.angle_increment * plus));
		double md = std::abs((scan.angle_min + scan.angle_increment * goalI) - (scan.angle_min + scan.angle_increment * minus));

		if(plus == scan.ranges.size()){
			pd = INFINITY_NUM;
		}

		if(minus == scan.ranges.size()){
			md = INFINITY_NUM;
		}

		if(pd<=md){
			moveAngle = scan.angle_min + scan.angle_increment * plus;
		}
		else{
			moveAngle = scan.angle_min + scan.angle_increment * minus;
		}
    }
    else{
        moveAngle = INFINITY_NUM;
    }

    return moveAngle;

}

double Moving::angleGtoR(double angle, geometry_msgs::PoseStamped poseG){
    double angleR;

    angleR = angle - 2*asin(poseG.pose.orientation.z);

    if(angleR < -M_PI){
        angleR = 2*M_PI + angleR;
    }
    if(angleR > M_PI){
        angleR = -2*M_PI + angleR;
    }

    return angleR;
}

bool Moving::bumperCollision(void){
    qBumper.callOne(ros::WallDuration(1));
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
    vel.angular.z = -1 * (thetaOld / std::abs(thetaOld)) * ROTATION_VELOCITY;
    pubVelocity.publish(vel);
}

void Moving::velocityPublisher(double theta, double v, double t){
    double omega;

    thetaOld = theta;

    omega = (2*theta)/t;

    geometry_msgs::Twist vel;

    vel.linear.x = v;
    vel.angular.z = omega;

    pubVelocity.publish(vel);
}

bool Moving::emergencyAvoidance(void){

    double aveP;
    double aveM;

    double sum = 0;

    static double sign = 0;

    //minus側の平均
    for(int i=0;i<scanData.ranges.size()/2;i++){
        if(!std::isnan(scanData.ranges[i])){
            sum += scanData.ranges[i];
        }
    }
    aveM = sum / (scanData.ranges.size()/2);

    //plus側
    sum = 0;
    for(int i=scanData.ranges.size()/2;i<scanData.ranges.size();i++){
        if(!std::isnan(scanData.ranges[i])){
            sum += scanData.ranges[i];
        }
    }
    aveP = sum / (scanData.ranges.size()/2);

    if(aveP < EMERGENCY_THRESHOLD && aveM < EMERGENCY_THRESHOLD){
        velocityPublisher(sign * scanData.angle_max/6,0.0,0.3);
    }
    else{
        if(aveP > aveM){
            sign = 1.0;
            velocityPublisher(sign*scanData.angle_max/6,FORWARD_VELOCITY,0.3);
        }
        else if(aveP < aveM){
            sign = -1.0;
            velocityPublisher(sign*scanData.angle_max/6,FORWARD_VELOCITY,0.3);
        }
        else{
            return false;
        }
    }

    return true;


}

bool Moving::roadCenterSearch(void){

    std::vector<float> fixRanges;
    std::vector<float> fixAngles;

    for(int i=0;i<scanData.ranges.size();i++){
        if(!std::isnan(scanData.ranges[i])){
            if(scanData.ranges[i]*cos(scanData.angle_min+(scanData.angle_increment*i)) <= ROAD_CENTER_THRESHOLD){
                fixRanges.push_back(scanData.ranges[i]);
                fixAngles.push_back(scanData.angle_min+(scanData.angle_increment*i));
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