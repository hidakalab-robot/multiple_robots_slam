#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <memory>

// 前方宣言

namespace boost{
    template<class T> 
    class shared_ptr;
}
/// my packages
namespace ExpLib{
    namespace Struct{
        template<typename T>
        struct pubStruct;
        struct subStructSimple;
    }
}
namespace exploration_msgs{
    template <class ContainerAllocator>
    struct AvoidanceStatus_;
    typedef ::exploration_msgs::AvoidanceStatus_<std::allocator<void>> AvoidanceStatus;
    typedef boost::shared_ptr< ::exploration_msgs::AvoidanceStatus const> AvoidanceStatusConstPtr;
    template <class ContainerAllocator>
    struct FrontierArray_;
    typedef ::exploration_msgs::FrontierArray_<std::allocator<void>> FrontierArray;
    typedef boost::shared_ptr< ::exploration_msgs::FrontierArray const> FrontierArrayConstPtr;
    template <class ContainerAllocator>
    struct PointArray_;
    typedef ::exploration_msgs::PointArray_<std::allocator<void>> PointArray;
    typedef boost::shared_ptr< ::exploration_msgs::PointArray const> PointArrayConstPtr;
}
/// rosmsgs
namespace actionlib_msgs{
    template <class ContainerAllocator>
    struct GoalStatusArray_;
    typedef ::actionlib_msgs::GoalStatusArray_<std::allocator<void>> GoalStatusArray;
    typedef boost::shared_ptr< ::actionlib_msgs::GoalStatusArray const> GoalStatusArrayConstPtr;
}
namespace geometry_msgs{
    template <class ContainerAllocator>
    struct PointStamped_;
    typedef ::geometry_msgs::PointStamped_<std::allocator<void>> PointStamped;
    typedef boost::shared_ptr< ::geometry_msgs::PointStamped const> PointStampedConstPtr;
    template <class ContainerAllocator>
    struct PoseStamped_;
    typedef ::geometry_msgs::PoseStamped_<std::allocator<void>> PoseStamped;
    typedef boost::shared_ptr< ::geometry_msgs::PoseStamped const> PoseStampedConstPtr;
}
namespace nav_msgs{
    template <class ContainerAllocator>
    struct Path_;
    typedef ::nav_msgs::Path_<std::allocator<void>> Path;
}
namespace visualization_msgs{
    template <class ContainerAllocator>
    struct Marker_;
    typedef ::visualization_msgs::Marker_<std::allocator<void>> Marker;
    template <class ContainerAllocator>
    struct MarkerArray_;
    typedef ::visualization_msgs::MarkerArray_<std::allocator<void>> MarkerArray;
}
//  others
namespace std{
    class mutex;
}
// 前方宣言ここまで


namespace ExStc = ExpLib::Struct;

class Visualization{
    private:
        // static parameters
        std::string INIT_FRAME_ID;
        double POSE_PUBLISH_RATE;
        double GOAL_PUBLISH_RATE;
        double BRANCH_PUBLISH_RATE;
        double DUPLICATE_BRANCH_PUBLISH_RATE;
        double ON_MAP_BRANCH_PUBLISH_RATE;
        double FRONTIER_PUBLISH_RATE;
        double USEFUL_FRONTIER_PUBLISH_RATE;
        double ROAD_PUBLISH_RATE;
        double AVOIDANCE_STATUS_PUBLISH_RATE;
        double CANCELED_GOALS_PUBLISH_RATE;
        
        // variables
        // pose
        std::unique_ptr<ExStc::subStructSimple> pose_;
        std::unique_ptr<ExStc::pubStruct<nav_msgs::Path>> posePath_;   
        std::unique_ptr<nav_msgs::Path> pp_;

        // goal
        std::unique_ptr<ExStc::subStructSimple> goal_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> goalMarker_;
        std::unique_ptr<visualization_msgs::Marker> gm_;
        std::unique_ptr<ExStc::subStructSimple> goSt_;
        std::unique_ptr<std::mutex> gmMutex_;

        // branch
        std::unique_ptr<ExStc::subStructSimple> branch_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> branchMarker_;
        std::unique_ptr<visualization_msgs::Marker> bm_;

        // duplicated branch
        std::unique_ptr<ExStc::subStructSimple> dupBranch_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> dupBranchMarker_;
        std::unique_ptr<visualization_msgs::Marker> dbm_;

        // on map branch
        std::unique_ptr<ExStc::subStructSimple> omBranch_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> omBranchMarker_;
        std::unique_ptr<visualization_msgs::Marker> obm_;

        // frontier
        std::unique_ptr<ExStc::subStructSimple> frontier_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> frontierMarker_;
        std::unique_ptr<visualization_msgs::Marker> fm_;

        // useful frontier
        std::unique_ptr<ExStc::subStructSimple> useFro_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> useFroMarker_;
        std::unique_ptr<visualization_msgs::Marker> ufm_;

        // road
        std::unique_ptr<ExStc::subStructSimple> road_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> roadMarker_;
        std::unique_ptr<visualization_msgs::Marker> rm_;
        std::unique_ptr<std::mutex> rmMutex_;

        // avoidance status
        std::unique_ptr<ExStc::subStructSimple> avoSta_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::MarkerArray>> avoStaMarker_;
        std::unique_ptr<visualization_msgs::Marker> asmt_;
        std::unique_ptr<visualization_msgs::MarkerArray> asm_;

        // canceled goals
        std::unique_ptr<ExStc::subStructSimple> caGoals_;
        std::unique_ptr<ExStc::pubStruct<visualization_msgs::Marker>> caGoalsMarker_;
        std::unique_ptr<visualization_msgs::Marker> cgm_;
        
        // functions
        void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
        void goalCB(const geometry_msgs::PointStampedConstPtr& msg);
        void goalStatusCB(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
        void branchCB(const exploration_msgs::PointArrayConstPtr& msg);
        void dupBranchCB(const exploration_msgs::PointArrayConstPtr& msg);
        void omBranchCB(const exploration_msgs::PointArrayConstPtr& msg);
        void frontierCB(const exploration_msgs::FrontierArrayConstPtr& msg);
        void useFroCB(const exploration_msgs::FrontierArrayConstPtr& msg);
        void roadCB(const geometry_msgs::PointStampedConstPtr& msg);
        void avoStaCB(const exploration_msgs::AvoidanceStatusConstPtr& msg);
        void caGoalsCB(const exploration_msgs::PointArrayConstPtr& msg);
        void posePathPublisher(void);
        void goalMarkerPublisher(void);
        void branchMarkerPublisher(void);
        void dupBranchMarkerPublisher(void);
        void omBranchMarkerPublisher(void);
        void frontierMarkerPublisher(void);
        void useFroMarkerPublisher(void);
        void roadMarkerPublisher(void);
        void avoStaMarkerPublisher(void);
        void caGoalsMarkerPublisher(void);
        void loadParams(void);

    public:
        Visualization();
        ~Visualization();
        void multiThreadMain(void);
};

#endif //VISUALIZATION_H