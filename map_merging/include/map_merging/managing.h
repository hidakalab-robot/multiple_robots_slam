#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <map_merging/TowMap.h>
#include <sensor_msgs/PointCloud2.h>

class Managing
{
private:
  ros::NodeHandle sM;
  ros::NodeHandle pC;
  ros::NodeHandle pO;
  ros::NodeHandle pM;

  ros::Subscriber subM;

  ros::Publisher pubCombine;
  ros::Publisher pubObstacles;
  ros::Publisher pubMap;

  map_merging::TowMap cMap;

  bool input;

public:
  ros::CallbackQueue queueM;

  Managing();
	~Managing(){};

  void inputMerged(const map_merging::TowMap::ConstPtr& sMMsg);
  bool isInput(void);
  void resetFlag(void);
  void combinedMapPublisher(void);
  void MapPublisher(void);
};

Managing::Managing()
{
  sM.setCallbackQueue(&queueM);

  subM = sM.subscribe("/map_merging/merging/mergedMap",1,&Managing::inputMerged,this);

  pubCombine = pC.advertise<map_merging::TowMap>("/map_merging/combining/mCombining", 1);
  pubObstacles = pO.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/mergedCloudObstacles", 1);
  pubMap = pM.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/mergedCloudMap", 1);

  input = false;
}

void Managing::inputMerged(const map_merging::TowMap::ConstPtr& sMMsg)
{
  cMap = *sMMsg;

  input = true;

  std::cout << "input MergedMap" << '\n';
}

bool Managing::isInput(void)
{
  return input;
}

void Managing::resetFlag(void)
{
  input = false;
}

void Managing::combinedMapPublisher(void)
{
  cMap.header.stamp = ros::Time::now();
  pubCombine.publish(cMap);
  std::cout << "published" << '\n';
}

void Managing::MapPublisher(void)
{
  cMap.cloudMap.header.stamp = ros::Time::now();
  cMap.cloudObstacles.header.stamp = ros::Time::now();

  pubMap.publish(cMap.cloudMap);
  pubObstacles.publish(cMap.cloudObstacles);

  std::cout << "published" << '\n';
}
