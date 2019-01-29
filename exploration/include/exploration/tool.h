//オドメトリのログなど何かで使いそうなログを保存するようのクラス
//後色々便利そうな物を追加予定
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

class Tool
{
private:
    ros::NodeHandle p;
    std::string poseTopic;
    std::string poseLogTopic;


    ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    geometry_msgs::PoseStamped poseData;

    ros::NodeHandle ppl;
	ros::Publisher pubPoseLog;
    geometry_msgs::PoseArray poseLog;

    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    Tool();
    ~Tool(){};

    void poseLogPublish(void);
};

Tool::Tool(){
    p.param<std::string>("pose_topic", poseTopic, "pose");
    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe(poseTopic,1,&Tool::poseCB,this);

	p.param<std::string>("pose_log_topic", poseLogTopic, "pose_log");
	pubPoseLog = ppl.advertise<geometry_msgs::PoseArray>(poseLogTopic, 1);

}

void Tool::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

void Tool::poseLogPublish(void){
    qPose.callOne(ros::WallDuration(1));

    poseLog.poses.push_back(poseData.pose);
    poseLog.header = poseData.header;

    pubPoseLog.publish(poseLog);
}