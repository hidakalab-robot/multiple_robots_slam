#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class TfChanger
{
private:
    ros::NodeHandle tc;
    ros::NodeHandle param;

    ros::Publisher pub;
    ros::Subscriber sub;

    std::string pubTopic;
    std::string subTopic;

    std::string newFrameId;

    void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

public:
    TfChanger();
    ~TfChanger(){};
};

TfChanger::TfChanger()
:param("~")
{
    param.getParam("sub_topic", subTopic);
    param.getParam("pub_topic", pubTopic);
    param.getParam("new_frame_id", newFrameId);

    pub = tc.advertise<nav_msgs::OccupancyGrid>(pubTopic, 1);
    sub = tc.subscribe(subTopic, 1, &TfChanger::callback,this);

}

void TfChanger::callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid msgFix;
    msgFix = *msg;
    msgFix.header.stamp = ros::Time::now();
    msgFix.header.frame_id = newFrameId;
    pub.publish(msgFix);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_tf_changer");
    TfChanger tc;
    ros::spin();
}