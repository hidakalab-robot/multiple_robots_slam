#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

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

    bool ceiling;

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    sensor_msgs::PointCloud2 deleteCeiling(sensor_msgs::PointCloud2 msg);

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
    param.getParam("ceiling", ceiling);

    pub = tc.advertise<sensor_msgs::PointCloud2>(pubTopic, 1);
    sub = tc.subscribe(subTopic, 1, &TfChanger::callback,this);

}

void TfChanger::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 msgFix;

    if(ceiling)
    {
        msgFix = deleteCeiling(*msg);
    }
    else{
        msgFix = *msg;
    }

    msgFix.header.stamp = ros::Time::now();
    msgFix.header.frame_id = newFrameId;
    pub.publish(msgFix);
}

sensor_msgs::PointCloud2 TfChanger::deleteCeiling(sensor_msgs::PointCloud2 msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (msg, *cloud);

    const double ceilingHeight = 2.4;

  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  	for(int i=0;i<cloud->points.size();i++)
  	{
    	if((cloud->points[i].z < ceilingHeight) && (cloud->points[i].z > -0.05))
    	{
    	  tempCloud->points.push_back(cloud->points[i]);
   	 }
  	}

  	cloud -> points = tempCloud -> points;
  	cloud -> width = tempCloud -> points.size();
  	cloud -> height = 1;
  	cloud -> is_dense = false;

    sensor_msgs::PointCloud2 fix;
    pcl::toROSMsg(*cloud,fix);
    return fix;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_tf_changer");
    TfChanger tc;
    ros::spin();
}