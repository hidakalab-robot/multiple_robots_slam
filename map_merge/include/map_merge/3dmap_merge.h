#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sonsor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

class 3DmapMerge
{
private:
  ros::NodeHandle 3d;

  ros::NodeHandle s1;
  ros::NodeHandle s2;

  ros::NodeHandle psd;

  ros::Subscriber sub1;
  ros::Subscriber sub2;

  geometry_msgs::Pose robot1Pose;
  geometry_msgs::Pose robot2Pose;

  sonsor_msgs::PointCloud2 robot1Map;
  sonsor_msgs::PointCloud2 robot2Map;

  bool input1;
  bool input2;

  void callback1(const sonsor_msgs::PointCloud2::ConstPtr& msg);
  void callback2(const sonsor_msgs::PointCloud2::ConstPtr& msg);

public:
  ros::CallbackQueue queue1;
  ros::CallbackQueue queue2;

  3DmapMerge();
  ~3DmapMerge(){};

  bool isInput1(void);
  bool isInput2(void);
  bool resetFlag(void);

  void mergeProcess(void);

};

3DmapMerge::3DmapMerge()
{
  s1.setCallbackQueue(&queue1);
  s2.setCallbackQueue(&queue2);

  sub1 = s1.subscribe("/robot1/cloud_obstacles",1,&3DmapMerge::callback1,this);
  sub2 = s2.subscribe("/robot2/cloud_obstacles",1,&3DmapMerge::callback2,this);

  input1 = false;
  input2 = false;
}

void 3DmapMerge::callback1(const sonsor_msgs::PointCloud2::ConstPtr& msg)
{
  robot1Map = *msg;
  input1 = true;
}

void 3DmapMerge::callback2(const sonsor_msgs::PointCloud2::ConstPtr& msg)
{
  robot2Map = *msg;
  input2 = true;
}

bool 3DmapMerge::isInput1(void)
{
  return input1;
}

bool 3DmapMerge::isInput2(void)
{
  return input2;
}

void 3DmapMerge::resetFlag(void)
{
  input1 = false;
  input2 = false;
}
