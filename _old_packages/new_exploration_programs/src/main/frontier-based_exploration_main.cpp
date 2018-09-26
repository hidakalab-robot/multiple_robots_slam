#include <new_exploration_programs/basic_process.h>
#include <new_exploration_programs/frontier_map_process.h>
#include <new_exploration_programs/frontier_moving_process.h>

int main(int argc, char** argv){

  	ros::init(argc, argv, "frontier_based_exploration");
  	ros::NodeHandle nh("~");

    bool rotate;
/*
    <arg name="rotate" default="true"/>
    <node pkg="ros_pkg" name="node_name" type="file_name">
    <param name="rotate_param" value="($arg rotate)"/>
    </node>
    roslaunch ros_pkg ros_pkg.launch rotate:=false
*/
    nh.getParam("rotate_param", rotate);

    BasicProcess bp;
    FrontierMapProcess fmap;
    FrontierMovingProcess fmove;
    //AvoidanceProcess ap;

    bp.set_whichexp(1.0);//移動軌跡描写用

    int led_val = 3;
    bp.set_led1(led_val);
    bp.pub_led1();

    std::cout << "rotate:" << rotate << std::endl;

    if(rotate)
    {
      bp.one_rotation();
    }
    while(!fmap.stop_check() && ros::ok())
    {
      fmap.frontier_search();
      if(!fmap.stop_check())
      {
        fmap.choose_goal_frontier();
        if (!fmap.stop_check())
        {
          fmove.VFH_navigation();
        }
      }
    }
    return 0;
}
