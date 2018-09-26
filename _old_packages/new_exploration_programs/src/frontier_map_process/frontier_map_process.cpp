#include <new_exploration_programs/frontier_map_process.h>

float FrontierMapProcess::pre_goal_point_x;
float FrontierMapProcess::pre_goal_point_y;
float FrontierMapProcess::flo_goal_x;
float FrontierMapProcess::flo_goal_y;

FrontierMapProcess::FrontierMapProcess()
{
	//led2_pub = fmp.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);

	//first_cycle = true;
	//led2.value = 0;
	stop = false;
	pre_goal_point_x = 0;
	pre_goal_point_y = 0;
};
