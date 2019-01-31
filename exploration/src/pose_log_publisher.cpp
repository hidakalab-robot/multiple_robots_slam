#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

class PoseLogPublisher
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
    PoseLogPublisher();
    ~PoseLogPublisher(){};

    void poseLogPublish(void);
};

PoseLogPublisher::PoseLogPublisher(){
    p.param<std::string>("pose_topic", poseTopic, "pose");
    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe(poseTopic,1,&PoseLogPublisher::poseCB,this);

	p.param<std::string>("pose_log_topic", poseLogTopic, "pose_log");
	pubPoseLog = ppl.advertise<geometry_msgs::PoseArray>(poseLogTopic, 1);

}

void PoseLogPublisher::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    poseData = *msg;
}

void PoseLogPublisher::poseLogPublish(void){
    qPose.callOne(ros::WallDuration(1));

    poseLog.poses.push_back(poseData.pose);
    poseLog.header = poseData.header;

    pubPoseLog.publish(poseLog);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_log_publisher");

    PoseLogPublisher plp;

    while(ros::ok()){
        plp.poseLogPublish();
    }
    
    return 0;
}