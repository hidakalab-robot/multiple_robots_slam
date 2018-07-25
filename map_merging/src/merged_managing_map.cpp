#include <map_merging/managing.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "merged_managing_map");

  Managing man;

  while(ros::ok())
  {
    man.queueM.callOne(ros::WallDuration(1));

    if(man.isInput())
    {
      while(ros::ok())
      {
        man.combinedMapPublisher();
      }
      man.MapPublisher();
    }
  }

  return 0;
}
