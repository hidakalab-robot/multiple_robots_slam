#include <map_merging/merging.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "merging_map");

  Merging mer;

  while(ros::ok())
  {
    mer.queueS.callOne(ros::WallDuration(1));

    if(mer.isInput())
    {
      mer.mergingFunction();
      mer.mergedPublisher();
    }
    else
    {
      std::cout << "not input" << '\n';
    }
    mer.resetFlag();
  }

  return 0;
}
