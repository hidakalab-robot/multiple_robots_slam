#ifndef ROBOT_MANAGER
#define ROBOT_MANAGER

#include <exploration_libraly/construct.h>
#include <exploration_libraly/convert.h>
#include <exploration_libraly/struct.h>
#include <exploration_msgs/PoseStampedArray.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <forward_list>
#include <geometry_msgs/PoseStamped.h>
#include <iterator>
#include <mutex>
#include <ros/ros.h>
#include <thread>

//複数台のロボットの情報を管理する
namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;
namespace ExCov = ExpLib::Convert;

class RobotManager
{
private:
    // static parameters
    std::string ROBOT_TOPIC;
    double RAGISTRATION_RATE;
    double CONVERT_RATE;
    double POSE_LOG_INTERVAL;
    std::string INDIVISUAL_POSE_LOG_TOPIC;

    // struct
    struct robotStruct{
        std::mutex mutex;
        std::string name;
        geometry_msgs::PoseStamped pose;
        exploration_msgs::PoseStampedArray poseLog;
        ros::Subscriber sub;
        ros::Publisher pub;
    };

    // variables
    ExStc::pubStruct<exploration_msgs::RobotInfoArray> robotArray_;
    ExStc::pubStruct<exploration_msgs::PoseStampedArray> poseArray_; 
    // ros::NodeHandle nhs_;
    boost::shared_mutex robotListMutex_;
    std::forward_list<robotStruct> robotList_;
    exploration_msgs::PoseStampedArray allPoseLog_;

    // functions
    void robotRegistration(void);//ロボットの情報を登録
    void update(const geometry_msgs::PoseStamped::ConstPtr& msg, RobotManager::robotStruct& robot); // データの更新
    bool isRobotTopic(const ros::master::TopicInfo& topic);
    void convertPoseToRobotInfo(void);//publish用のデータに整形
    void registrationLoop(void);
    void convertLoop(void);
    void loadParams(void);

public:
    RobotManager();
    void multiThreadMain(void);
};

#endif // ROBOT_MANAGER