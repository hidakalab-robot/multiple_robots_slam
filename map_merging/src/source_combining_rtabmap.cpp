#include <map_merging/combining.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "combining_rtabmap");

  Combining com;

  while(ros::ok())
  {
    com.queueO.callOne(ros::WallDuration(1));
    com.queueM.callOne(ros::WallDuration(1));

    if(com.isInputO() && com.isInputM())
    {
      com.combinedMapPublisher();
    }
    else
    {
      if(!com.isInputO())
      {
        std::cout << "not inputO" << '\n';
      }
      if(!com.isInputM())
      {
        std::cout << "not inputM" << '\n';
      }
    }
    com.resetFlag();
  }

  return 0;
}
