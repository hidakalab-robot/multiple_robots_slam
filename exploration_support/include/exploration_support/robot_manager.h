#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <memory>
#include <forward_list>

// 前方宣言

namespace boost{
    class shared_mutex;
    template<class T> 
    class shared_ptr;
}
/// my packages
namespace ExpLib{
    namespace Struct{
        template<typename T>
        struct pubStruct;
    }
}
namespace exploration_msgs{
    template <class ContainerAllocator>
    struct PoseStampedArray_;
    typedef ::exploration_msgs::PoseStampedArray_<std::allocator<void>> PoseStampedArray;
    template <class ContainerAllocator>
    struct RobotInfoArray_;
    typedef ::exploration_msgs::RobotInfoArray_<std::allocator<void>> RobotInfoArray;
}
/// ros
namespace ros{
    namespace master{
        struct TopicInfo;
    }
}
/// rosmsgs
namespace geometry_msgs{
    template <class ContainerAllocator>
    struct PoseStamped_;
    typedef ::geometry_msgs::PoseStamped_<std::allocator<void>> PoseStamped;
    typedef boost::shared_ptr< ::geometry_msgs::PoseStamped const> PoseStampedConstPtr;
}
// 前方宣言ここまで

//複数台のロボットの情報を管理する
namespace ExStc = ExpLib::Struct;

class RobotManager{
    private:
        // static parameters
        std::string ROBOT_TOPIC;
        double RAGISTRATION_RATE;
        double CONVERT_RATE;
        double POSE_LOG_INTERVAL;
        std::string INDIVISUAL_POSE_LOG_TOPIC;

        // struct
        struct robotStruct;

        // variables
        std::unique_ptr<ExStc::pubStruct<exploration_msgs::RobotInfoArray>> robotArray_;
        std::unique_ptr<ExStc::pubStruct<exploration_msgs::PoseStampedArray>> poseArray_; 
        std::unique_ptr<boost::shared_mutex> robotListMutex_;
        std::unique_ptr<std::forward_list<robotStruct>> robotList_;
        std::unique_ptr<exploration_msgs::PoseStampedArray> allPoseLog_;

        // functions
        void robotRegistration(void);//ロボットの情報を登録
        void update(const geometry_msgs::PoseStampedConstPtr& msg, RobotManager::robotStruct& robot); // データの更新
        bool isRobotTopic(const ros::master::TopicInfo& topic);
        void convertPoseToRobotInfo(void);//publish用のデータに整形
        void registrationLoop(void);
        void convertLoop(void);
        void loadParams(void);

    public:
        RobotManager();
        ~RobotManager();
        void multiThreadMain(void);
};

#endif // ROBOT_MANAGER_H