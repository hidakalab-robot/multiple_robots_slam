#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <map_merging/TowMap.h>
#include <std_srvs/Empty.h>

class Combining
{
private:
  ros::NodeHandle sO;
  ros::NodeHandle sM;

  ros::NodeHandle pC;

  ros::NodeHandle v;

  ros::Subscriber subO;
	ros::Subscriber subM;

  ros::Publisher pubCombine;

  ros::ServiceClient vC;

  map_merging::TowMap cMap;

  std_srvs::Empty srv;

  bool inputO;
	bool inputM;

  int ALLOW_SIZE;

public:
  ros::CallbackQueue queueO;
	ros::CallbackQueue queueM;

  Combining();
	~Combining(){};

  void inputObstacles(const sensor_msgs::PointCloud2::ConstPtr& sOMsg);
  void inputMap(const sensor_msgs::PointCloud2::ConstPtr& sMMsg);
  void combinedMapPublisher(void);
  bool isInputO(void);
  bool isInputM(void);
  void resetFlag(void);
  void resetMap(void);

};

Combining::Combining()
{
  sO.setCallbackQueue(&queueO);
  sM.setCallbackQueue(&queueM);

  subO = sO.subscribe("/rtabmap/cloud_obstacles",1,&Combining::inputObstacles,this);
  subM = sM.subscribe("/rtabmap/cloud_map",1,&Combining::inputMap,this);

  pubCombine = pC.advertise<map_merging::TowMap>("/map_merging/sCombining", 1);

  vC = v.serviceClient<std_srvs::Empty>("/rtabmap/reset");

  inputO = false;
  inputM = false;

  ALLOW_SIZE = 10000;
}

void Combining::inputObstacles(const sensor_msgs::PointCloud2::ConstPtr& sOMsg)
{
  cMap.cloudObstacles = *sOMsg;

  inputO = true;

  std::cout << "input O_cloud " << std::endl;
}

void Combining::inputMap(const sensor_msgs::PointCloud2::ConstPtr& sMMsg)
{
  cMap.cloudMap = *sMMsg;

  inputM = true;

  std::cout << "input M_cloud " << std::endl;
}

void Combining::combinedMapPublisher(void)
{

  if(cMap.cloudObstacles.height * cMap.cloudObstacles.width > ALLOW_SIZE)
  {
    cMap.header.stamp = ros::Time::now();
    pubCombine.publish(cMap);
    std::cout << "published" << '\n';
    resetMap();
  }
  else
  {
    std::cout << "not_published" << '\n';
  }

}

bool Combining::isInputO(void)
{
  return inputO;
}

bool Combining::isInputM(void)
{
  return inputM;
}

void Combining::resetFlag(void)
{
  inputO = false;
  inputM = false;
}

void Combining::resetMap(void)
{
  if (vC.call(srv))
  {
    std::cout << "map-reset is success" << '\n';
  }
  else
  {
    std::cout << "map-reset is failed" << '\n';
  }
}
