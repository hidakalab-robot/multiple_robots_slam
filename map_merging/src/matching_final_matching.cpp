#include <map_merging/final_matching.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "matching_final_matching");

  FinalMatching fm;

  while(ros::ok())
  {
    fm.queueE.callOne(ros::WallDuration(1));
    fm.queueS.callOne(ros::WallDuration(1));

    if(fm.isInputE() && fm.isInputS())
    {
      if(fm.isSameCluster())//読み込んだマッチング結果が同じ時刻の点群についてやったものか確認
      {
        fm.receiveReport();
        fm.finalMatchProcess();
        //fm.finalMatchPublisher();
        fm.echoClouds();
        fm.echoMatch(0);
        fm.echoMatch(1);
        //fm.echoMatch(2);
      }
    }
    else
    {
      if(!fm.isInputE())
      {
        std::cout << "not input Eigen" << '\n';
      }
      if(!fm.isInputS())
      {
        std::cout << "not input Shot" << '\n';
      }
    }
    fm.resetFlag();
  }

  return 0;
}
