#include <map_merging/shot_matching.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "matching_shot_feature");

  ShotMatching shot;

  while(ros::ok())
  {
    shot.queueE.callOne(ros::WallDuration(1));

    if(shot.isInput())
    {
      shot.cluster2LinkCluster();
      if(shot.isMatch())
      {
        shot.shotPublisher();
      }
    }
    else
    {
      std::cout << "not input" << '\n';
    }
    shot.resetFlag();
  }

  return 0;
}
