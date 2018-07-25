#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <map_merging/Match.h>
#include <map_merging/TowMap.h>

class Merging
{
private:
  ros::NodeHandle sS;
  ros::NodeHandle pM;
  ros::Subscriber subS;
  ros::Publisher pubMerged;

  map_merging::Match shot;
  map_merging::TowMap merged;

  bool input;

public:
  ros::CallbackQueue queueS;

  Merging();
	~Merging(){};

  void inputShot(const map_merging::Match::ConstPtr& sSMsg);
  bool isInput(void);
  void resetFlag(void);
  void mergedPublisher(void);
  void mergingFunction(void);
};

Merging::Merging()
{
  sS.setCallbackQueue(&queueS);
  subS = sS.subscribe("/map_merging/matching/finalMatching",1,&Merging::inputShot,this);
  pubMerged = pM.advertise<map_merging::TowMap>("/map_merging/merging/mergedMap", 1);
}

void Merging::inputShot(const map_merging::Match::ConstPtr& sSMsg)
{
  shot = *sSMsg;
  input = true;
  std::cout << "input Shot" << '\n';
}

bool Merging::isInput(void)
{
  return input;
}

void Merging::resetFlag(void)
{
  input = false;
}

void Merging::mergedPublisher(void)
{
  merged.header.stamp = ros::Time::now();
  pubMerged.publish(merged);
  std::cout << "published" << '\n';
}

void Merging::mergingFunction(void)
{

}
