#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <cloud_map_merge/RobotPose.h>
#include <cloud_map_merge/AllRobotData.h>
#include <cloud_map_merge/OverlapArray.h>

class CloudMapSet
{
private:
  ros::NodeHandle cms;

  ros::NodeHandle s1;
  ros::NodeHandle s2;
  ros::NodeHandle s3;

  ros::NodeHandle p;

  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;

  ros::Publisher pub;

  cloud_map_merge::RobotPose robot1Pose;
  cloud_map_merge::RobotPose robot2Pose;

  double robot1X;
  double robot1Y;
  double robot1Yaw;

  double robot2X;
  double robot2Y;
  double robot2Yaw;

  int robotNum;

  sensor_msgs::PointCloud2 robot1Map;
  sensor_msgs::PointCloud2 robot2Map;

  cloud_map_merge::OverlapArray overlaps;

  bool input1;
  bool input2;
  bool input3;

  void callback1(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void callback2(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void callback3(const cloud_map_merge::OverlapArray::ConstPtr& msg);

public:
  ros::CallbackQueue queue1;
  ros::CallbackQueue queue2;
  ros::CallbackQueue queue3;

  CloudMapSet();
  ~CloudMapSet(){};

  bool isInput1(void);
  bool isInput2(void);
  bool isInput3(void);
  void resetFlag(void);

  void dataPublish(void);

};

CloudMapSet::CloudMapSet()
:cms("~")
{
  s1.setCallbackQueue(&queue1);
  s2.setCallbackQueue(&queue2);
  s3.setCallbackQueue(&queue3);

  sub1 = s1.subscribe("/robot1/rtabmap/cloud_obstacles",1,&CloudMapSet::callback1,this);
  sub2 = s2.subscribe("/robot2/rtabmap/cloud_obstacles",1,&CloudMapSet::callback2,this);
  sub3 = s3.subscribe("/server/grid_map_merge/overlap",1,&CloudMapSet::callback3,this);

  pub = p.advertise<cloud_map_merge::AllRobotData>("cloud_map_merge/all_robot_data", 1);

  cms.getParam("robot1_init_x", robot1X);
  cms.getParam("robot1_init_y", robot1Y);
  cms.getParam("robot1_init_yaw", robot1Yaw);

  cms.getParam("robot2_init_x", robot2X);
  cms.getParam("robot2_init_y", robot2Y);
  cms.getParam("robot2_init_yaw", robot2Yaw);

  cms.getParam("robot_num",robotNum);


  input1 = false;
  input2 = false;
  input3 = false;
}

void CloudMapSet::callback1(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  robot1Map = *msg;
  input1 = true;
}

void CloudMapSet::callback2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  robot2Map = *msg;
  input2 = true;
}

void CloudMapSet::callback3(const cloud_map_merge::OverlapArray::ConstPtr& msg)
{
  overlaps = *msg;
  input3 = true;
}

bool CloudMapSet::isInput1(void)
{
  return input1;
}

bool CloudMapSet::isInput2(void)
{
  return input2;
}

bool CloudMapSet::isInput3(void)
{
  return input3;
}

void CloudMapSet::resetFlag(void)
{
  //std::cout << "robot1 << " << robot1X << ", " << robot1Y << ", " << robot1Yaw << '\n';
  //std::cout << "robot2 << " << robot2X << ", " << robot2Y << ", " << robot2Yaw << '\n';

  input1 = false;
  input2 = false;
  input3 = false;
}

void CloudMapSet::dataPublish(void)
{
  cloud_map_merge::AllRobotData data;
  std::vector<cloud_map_merge::RobotPose> poses;
  std::vector<sensor_msgs::PointCloud2> maps;

  robot1Pose.x = robot1X;
  robot1Pose.y = robot1Y;
  robot1Pose.yaw = robot1Yaw;

  robot2Pose.x = robot2X;
  robot2Pose.y = robot2Y;
  robot2Pose.yaw = robot2Yaw;

  poses.push_back(robot1Pose);
  poses.push_back(robot2Pose);

  maps.push_back(robot1Map);
  maps.push_back(robot2Map);

  data.poses = poses;
  data.maps = maps;

  if(input3)
    data.overlaps = overlaps.overlapArray;

  data.header.stamp = ros::Time::now();

  pub.publish(data);
}
