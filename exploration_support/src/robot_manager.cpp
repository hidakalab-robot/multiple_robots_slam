#include <exploration_support/robot_manager.h>
#include <exploration_libraly/construct.h>
#include <exploration_libraly/convert.h>
#include <exploration_libraly/struct.h>
// #include <exploration_msgs/PoseStampedArray.h>
#include <nav_msgs/Path.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <iterator>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

//複数台のロボットの情報を管理する
namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;
namespace ExCov = ExpLib::Convert;

struct RobotManager::robotStruct{
    std::mutex mutex;
    std::string name;
    geometry_msgs::PoseStamped pose;
    // exploration_msgs::PoseStampedArray poseLog;
    nav_msgs::Path poseLog;
    ros::Subscriber sub;
    ros::Publisher pub;
};

RobotManager::RobotManager()
    :robotArray_(new ExStc::pubStruct<exploration_msgs::RobotInfoArray>("robot_array",1,true))
    // ,poseArray_(new ExStc::pubStruct<exploration_msgs::PoseStampedArray>("pose_log/merge",1,true))
    ,poseArray_(new ExStc::pubStruct<nav_msgs::Path>("pose_log/merge",1,true))
    ,robotListMutex_(new boost::shared_mutex())
    ,robotList_(new std::forward_list<robotStruct>())
    // ,allPoseLog_(new exploration_msgs::PoseStampedArray()){
    ,allPoseLog_(new nav_msgs::Path()){
    loadParams();
}
RobotManager::~RobotManager(){}

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

void RobotManager::robotRegistration(void){
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);

    for(const auto& topic : topicList){
        // ROS_DEBUG_STREAM("topic name : " << topic.name);
        if(!isRobotTopic(topic)) continue;
        std::string robotName = ros::names::parentNamespace(topic.name);
        //すでに登録されていないかロボットの名前を確認
        {
            bool isRegisterd = false;
            {
                boost::shared_lock<boost::shared_mutex> bLock(*robotListMutex_);
                for(auto&& robot : *robotList_){
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
            std::lock_guard<boost::shared_mutex> bLock(*robotListMutex_);
            robotList_->emplace_front();
            RobotManager::robotStruct& robot = robotList_->front();
            {
                ROS_DEBUG_STREAM("registrationThread << edit list");
                std::lock_guard<std::mutex> lock(robot.mutex);
                robot.name = robotName;
                robot.sub = ros::NodeHandle().subscribe<geometry_msgs::PoseStamped>(topic.name, 1, [this, &robot](const geometry_msgs::PoseStamped::ConstPtr& msg) {update(msg, robot);});
                // robot.pub = ros::NodeHandle().advertise<exploration_msgs::PoseStampedArray>(ros::names::append(INDIVISUAL_POSE_LOG_TOPIC,robotName),1);
                robot.pub = ros::NodeHandle().advertise<nav_msgs::Path>(ros::names::append(INDIVISUAL_POSE_LOG_TOPIC,robotName),1);
            }
        }
    }
}

bool RobotManager::isRobotTopic(const ros::master::TopicInfo& topic){
    bool isRobot = topic.name == ros::names::append(ros::names::parentNamespace(topic.name),ROBOT_TOPIC);
    bool isPose = topic.datatype == "geometry_msgs/PoseStamped";
    // ROS_DEBUG_STREAM("isRobot : " << isRobot << " : " << topic.name);
    // ROS_DEBUG_STREAM("isPose : " << isPose << " : " << topic.datatype);
    return isRobot && isPose;
}

void RobotManager::update(const geometry_msgs::PoseStampedConstPtr& msg, RobotManager::robotStruct& robot){
    std::lock_guard<std::mutex> lock(robot.mutex);
    // if(msg -> header.stamp < robot.pose.header.stamp) return;
    robot.pose = *msg;

    if(ros::Duration(robot.pose.header.stamp - robot.poseLog.header.stamp).toSec() >= POSE_LOG_INTERVAL){
        robot.poseLog.poses.emplace_back(*msg);
        allPoseLog_->poses.emplace_back(*msg);
        robot.poseLog.header = msg -> header;
        robot.poseLog.header.stamp = allPoseLog_->header.stamp = ros::Time::now();
    }
    robot.pub.publish(robot.poseLog);
    // poseArray_->pub.publish(robot.poseLog);
    poseArray_->pub.publish(*allPoseLog_);
}

void RobotManager::convertPoseToRobotInfo(void){
    exploration_msgs::RobotInfoArray ria;
    ria.info.reserve(std::distance(robotList_->begin(),robotList_->end()));
    {
        std::lock_guard<boost::shared_mutex> bLock(*robotListMutex_);
        for(auto&& robot : *robotList_){
            std::lock_guard<std::mutex> lock(robot.mutex);
            double yaw = ExCov::qToYaw(robot.pose.pose.orientation);
            ria.info.emplace_back(ExCos::msgRobotInfo(robot.name,robot.pose.pose));
        }
    }
    ria.header.stamp = ros::Time::now();
    robotArray_->pub.publish(ria);
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

void RobotManager::loadParams(void){
    ros::NodeHandle nh("~");
    // static parameters
    nh.param<std::string>("robot_topic",ROBOT_TOPIC,"pose");//globalのポーズを撮ってこないと厳しい
    nh.param<double>("ragistration_rate", RAGISTRATION_RATE, 0.5);
    nh.param<double>("convert_rate", CONVERT_RATE, 1.0);
    double POSE_LOG_RATE;
    nh.param<double>("pose_log_rate",POSE_LOG_RATE,10.0);
    POSE_LOG_INTERVAL = 1/POSE_LOG_RATE;
    nh.param<std::string>("indivisual_pose_log_topic",INDIVISUAL_POSE_LOG_TOPIC,"pose_log");
}