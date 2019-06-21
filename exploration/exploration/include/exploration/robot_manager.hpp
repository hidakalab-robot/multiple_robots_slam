#include <ros/ros.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <exploration/common_lib.hpp>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <forward_list>
#include <iterator>

//複数台のロボットの情報を管理する
//
class RobotManager
{
private:
    struct robotStruct{
        std::mutex mutex;
        std::string name;
        geometry_msgs::PoseStamped pose;
        ros::Subscriber sub;
        // bool update;
    };

    ros::NodeHandle nhs;
    boost::shared_mutex robotListMutex;
    std::forward_list<robotStruct> robotList;

    std::string ROBOT_TOPIC;
    CommonLib::pubStruct<exploration_msgs::RobotInfoArray> robotArray_;
    void robotRegistration(void);//ロボットの情報を登録
    void update(const geometry_msgs::PoseStamped::ConstPtr& msg, RobotManager::robotStruct& robot); // データの更新
    bool isRobotTopic(const ros::master::TopicInfo& topic);
    void convertPoseToRobotInfo(void);//publish用のデータに整形
public:
    RobotManager();
};

RobotManager::RobotManager():robotArray_("robotArray",1){
    ros::NodeHandle p("~");
    p.param<std::string>("robot_topic",ROBOT_TOPIC,"robot_pose");//globalのポーズを撮ってこないと厳しい
}

void RobotManager::robotRegistration(void){
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);

    for(const auto& topic : topicList){
        if(!isRobotTopic(topic)) continue;

        std::string robotName = ros::names::parentNamespace(topic.name);
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
                // robot.update = false;
            }
        }

    }
}

bool RobotManager::isRobotTopic(const ros::master::TopicInfo& topic){
    bool isRobot = topic.name == ros::names::append(ros::names::parentNamespace(topic.name),ROBOT_TOPIC);
    bool isPose = topic.datatype == "geometry_msgs::PoseStamped";
    return isRobot && isPose;
}

void RobotManager::update(const geometry_msgs::PoseStamped::ConstPtr& msg, RobotManager::robotStruct& robot){
    std::lock_guard<std::mutex> lock(robot.mutex);
    if(msg -> header.stamp < robot.pose.header.stamp) return;
    robot.pose = *msg;
    // robot.update = true;
}

void RobotManager::convertPoseToRobotInfo(void){
    exploration_msgs::RobotInfoArray ria;
    ria.list.reserve = std::distance(robotList.begin(),robotList.end());
    {
        std::lock_guard<boost::shared_mutex> bLock(robotListMutex);
        for(auto&& robot : robotList){
            std::lock_guard<std::mutex> lock(robot.mutex);
            double yaw = CommonLib::qToYaw(robot.pose.pose.orientation);
            ria.list.emplace_back(CommonLib::msgRobotInfo(robot.name,robot.pose.pose.position,CommonLib::msgVector(cos(yaw),sin(yaw))));
        }
    }
    ria.header.stamp = ros::Time::now();
    robotArray_.pub.publish(ria);
}