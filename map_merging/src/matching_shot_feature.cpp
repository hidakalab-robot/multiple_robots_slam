#include <map_merging/shot_matching.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "matching_shot_feature");

  ShotMatching shot;

  while(ros::ok())
  {
    shot.queueS.callOne(ros::WallDuration(1));
    shot.queueM.callOne(ros::WallDuration(1));

    if(shot.isInputS() && shot.isInputM())
    {
      if(!shot.isSkip())
      {
        shot.includeCloud();
        shot.cluster2Scene();
        shot.shotPublisher();
        shot.setSkip(true);
        //usleep(500000);
      }
      else
      {
        shot.setSkip(false);
      }

    }
    else
    {
      if(!shot.isInputS())
      {
        std::cout << "not input source" << '\n';
      }
      if(!shot.isInputM())
      {
        std::cout << "not input merged" << '\n';
      }
    }
    shot.resetFlag();
  }

  return 0;
}
