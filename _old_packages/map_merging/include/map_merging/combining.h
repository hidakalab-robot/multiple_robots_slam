#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <map_merging/TowMap.h>
#include <std_srvs/Empty.h>
#include <map_merging/ProcessTime.h>

class Combining
{
private:
  ros::NodeHandle sO;
  ros::NodeHandle sM;

  ros::NodeHandle pC;
  ros::NodeHandle pF;

  ros::NodeHandle pT;

  ros::NodeHandle v;

  ros::Subscriber subO;
	ros::Subscriber subM;

  ros::Publisher pubCombine;
  ros::Publisher pubFirst;
  ros::Publisher pubTime;
  map_merging::ProcessTime processTime;

  ros::ServiceClient vC;

  map_merging::TowMap cMap;

  std_srvs::Empty srv;

  bool inputO;
	bool inputM;

  bool twoCloud;

  bool firstProccess;

  int ALLOW_SIZE;

  void inputObstacles(const sensor_msgs::PointCloud2::ConstPtr& sOMsg);
  void inputMap(const sensor_msgs::PointCloud2::ConstPtr& sMMsg);
  void resetMap(void);

public:
  ros::CallbackQueue queueO;
	ros::CallbackQueue queueM;

  Combining();
	~Combining(){};


  void combinedMapPublisher(void);
  bool isInputO(void);
  bool isInputM(void);
  void resetFlag(void);


};

Combining::Combining()
{
  sO.setCallbackQueue(&queueO);
  sM.setCallbackQueue(&queueM);

  subO = sO.subscribe("/rtabmap/cloud_obstacles",1,&Combining::inputObstacles,this);
  subM = sM.subscribe("/rtabmap/cloud_map",1,&Combining::inputMap,this);

  pubCombine = pC.advertise<map_merging::TowMap>("/map_merging/combining/sCombining", 1);
  pubFirst = pF.advertise<map_merging::TowMap>("/map_merging/merging/mergedMap", 1);
  pubTime = pT.advertise<map_merging::ProcessTime>("/map_merging/processTime/sCombining", 1);

  vC = v.serviceClient<std_srvs::Empty>("/rtabmap/reset");

  inputO = false;
  inputM = false;

  firstProccess = true;

  ALLOW_SIZE = 10000;

  twoCloud = true;

  processTime.processName = "SourceCombining";
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
    /*処理時間計算*/
    ros::Duration time;
    time = cMap.header.stamp - cMap.cloudObstacles.header.stamp;
    processTime.processTime = time.toSec();

    if(firstProccess)
    {
      pubFirst.publish(cMap);
      pubTime.publish(processTime);
      std::cout << "first published" << '\n';
      firstProccess = false;
    }
    else
    {
      if(twoCloud)
      {
        pubCombine.publish(cMap);
        pubTime.publish(processTime);
        std::cout << "published" << '\n';
      }
      else
      {
        while(ros::ok())
        {
          pubCombine.publish(cMap);
          pubTime.publish(processTime);
          std::cout << "published" << '\n';
        }
      }
    }
    if(!twoCloud)
    {
      resetMap();
    }
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
