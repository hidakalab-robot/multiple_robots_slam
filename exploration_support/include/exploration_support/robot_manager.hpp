#ifndef ROBOT_MANAGER
#define ROBOT_MANAGER

#include <ros/ros.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/constructor.hpp>
#include <exploration_libraly/convert.hpp>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <forward_list>
#include <iterator>
#include <exploration_msgs/PoseStampedArray.h>
#include <thread>

//複数台のロボットの情報を管理する
//
class RobotManager
{
private:
    struct robotStruct{
        std::mutex mutex;
        std::string name;
        geometry_msgs::PoseStamped pose;
        exploration_msgs::PoseStampedArray poseLog;
        ros::Subscriber sub;
        ros::Publisher pub;
    };

    ros::NodeHandle nhs;
    boost::shared_mutex robotListMutex;
    std::forward_list<robotStruct> robotList;

    std::string ROBOT_TOPIC;
    double RAGISTRATION_RATE;
    double CONVERT_RATE;
    double POSE_LOG_INTERVAL;
    std::string INDIVISUAL_POSE_LOG_TOPIC;

    ExpLib::pubStruct<exploration_msgs::RobotInfoArray> robotArray_;
    ExpLib::pubStruct<exploration_msgs::PoseStampedArray> poseArray_; 

    exploration_msgs::PoseStampedArray allPoseLog;

    void robotRegistration(void);//ロボットの情報を登録
    void update(const geometry_msgs::PoseStamped::ConstPtr& msg, RobotManager::robotStruct& robot); // データの更新
    bool isRobotTopic(const ros::master::TopicInfo& topic);
    void convertPoseToRobotInfo(void);//publish用のデータに整形
    void registrationLoop(void);
    void convertLoop(void);
public:
    RobotManager();
    void multiThreadMain(void);
};

RobotManager::RobotManager():robotArray_("robot_array",1,true),poseArray_("pose_log/all_robots",1,true){
    ros::NodeHandle p("~");
    p.param<std::string>("robot_topic",ROBOT_TOPIC,"robot_pose/global");//globalのポーズを撮ってこないと厳しい
    p.param<double>("ragistration_rate", RAGISTRATION_RATE, 0.5);
    p.param<double>("convert_rate", CONVERT_RATE, 1.0);

    double POSE_LOG_RATE;
    p.param<double>("pose_log_rate",POSE_LOG_RATE,10.0);
    POSE_LOG_INTERVAL = 1/POSE_LOG_RATE;

    p.param<std::string>("indivisual_pose_log_topic",INDIVISUAL_POSE_LOG_TOPIC,"pose_log/global");
}

void RobotManager::robotRegistration(void){
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);

    for(const auto& topic : topicList){
        // ROS_DEBUG_STREAM("topic name : " << topic.name);
        if(!isRobotTopic(topic)) continue;
        std::string robotName = ros::names::parentNamespace(ros::names::parentNamespace(topic.name));
        //すでに登録されていないかロボットの名前を確認
        {
            bool isRegisterd = false;
            {
                boost::shared_lock<boost::shared_mutex> bLock(robotListMutex);
                for(auto&& robot : robotList){
                    std::lock_guard<std::mutex> lock(robot.mutex);
                    if(robotName == robot.name){
                        isRegisterd = true;
                        break;
                    }
                }
            }
            if(isRegisterd) continue;
        }

        ROS_DEBUG_STREAM("registrationThread << add to system : " << robotName);
        {
            std::lock_guard<boost::shared_mutex> bLock(robotListMutex);
            robotList.emplace_front();
            RobotManager::robotStruct& robot = robotList.front();
            {
                ROS_DEBUG_STREAM("registrationThread << edit list");
                std::lock_guard<std::mutex> lock(robot.mutex);
                robot.name = robotName;
                robot.sub = nhs.subscribe<geometry_msgs::PoseStamped>(topic.name, 1, [this, &robot](const geometry_msgs::PoseStamped::ConstPtr& msg) {update(msg, robot);});
                robot.pub = nhs.advertise<exploration_msgs::PoseStampedArray>(ros::names::append(INDIVISUAL_POSE_LOG_TOPIC,robotName),1);
            }
        }
    }
}

bool RobotManager::isRobotTopic(const ros::master::TopicInfo& topic){
    bool isRobot = topic.name == ros::names::append(ros::names::parentNamespace(ros::names::parentNamespace(topic.name)),ROBOT_TOPIC);
    bool isPose = topic.datatype == "geometry_msgs/PoseStamped";
    // ROS_DEBUG_STREAM("isRobot : " << isRobot << " : " << topic.name);
    // ROS_DEBUG_STREAM("isPose : " << isPose << " : " << topic.datatype);
    return isRobot && isPose;
}

void RobotManager::update(const geometry_msgs::PoseStamped::ConstPtr& msg, RobotManager::robotStruct& robot){
    std::lock_guard<std::mutex> lock(robot.mutex);
    if(msg -> header.stamp < robot.pose.header.stamp) return;
    robot.pose = *msg;

    if(ros::Duration(robot.pose.header.stamp - robot.poseLog.header.stamp).toSec() >= POSE_LOG_INTERVAL){
        robot.poseLog.poses.emplace_back(*msg);
        allPoseLog.poses.emplace_back(*msg);
        robot.poseLog.header = msg -> header;
        robot.poseLog.header.stamp = allPoseLog.header.stamp = ros::Time::now();
    }
    robot.pub.publish(robot.poseLog);
    // poseArray_.pub.publish(robot.poseLog);
    poseArray_.pub.publish(allPoseLog);
}

void RobotManager::convertPoseToRobotInfo(void){
    exploration_msgs::RobotInfoArray ria;
    ria.list.reserve(std::distance(robotList.begin(),robotList.end()));
    {
        std::lock_guard<boost::shared_mutex> bLock(robotListMutex);
        for(auto&& robot : robotList){
            std::lock_guard<std::mutex> lock(robot.mutex);
            double yaw = ExpLib::qToYaw(robot.pose.pose.orientation);
            ria.list.emplace_back(ExpLib::msgRobotInfo(robot.name,robot.pose,robot.pose.pose.position,ExpLib::msgVector(cos(yaw),sin(yaw))));
        }
    }
    ria.header.stamp = ros::Time::now();
    robotArray_.pub.publish(ria);
}

void RobotManager::registrationLoop(void){
    ros::Rate rate(RAGISTRATION_RATE);
    while(ros::ok()){
        robotRegistration();
        rate.sleep();
    }
}

void RobotManager::convertLoop(void){
    ros::Rate rate(CONVERT_RATE);
    while(ros::ok()){
        convertPoseToRobotInfo();
        rate.sleep();
    }
}

void RobotManager::multiThreadMain(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread registrationThread([this]{registrationLoop();});
    std::thread convertThread([this]{convertLoop();});
    ros::spin();
    registrationThread.join();
    convertThread.join();//スレッドの終了を待つ??
    ROS_INFO_STREAM("end main loop\n");
}

#endif // ROBOT_MANAGER