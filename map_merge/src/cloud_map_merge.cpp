#include <map_merge/cloud_map_merge.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "cloud_map_merge");

  CloudMapMerge mm;

  while(ros::ok())
  {
    mm.queue.callOne(ros::WallDuration(1));

    if(mm.isInput())
    {
      mm.merge();
    }
    else
    {
        std::cout << "not input" << '\n';
    }
    mm.resetFlag();
  }
  return 0;
}
