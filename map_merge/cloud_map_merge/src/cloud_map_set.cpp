#include <cloud_map_merge/cloud_map_set.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "cloud_map_set");

  CloudMapSet ms;

  while(ros::ok())
  {
    ms.queue1.callOne(ros::WallDuration(1));
    ms.queue2.callOne(ros::WallDuration(1));
    ms.queue3.callOne(ros::WallDuration(1));

    if(ms.isInput1() && ms.isInput2() && ms.isInput3())
    {
      ms.dataPublish();
    }
    else
    {
      if(!ms.isInput1())
      {
        std::cout << "not input robot1" << '\n';
      }
      if(!ms.isInput2())
      {
        std::cout << "not input robot2" << '\n';
      }
      if(!ms.isInput3())
      {
        std::cout << "not input overlap" << '\n';
      }
    }
    ms.resetFlag();
  }
  return 0;
}
