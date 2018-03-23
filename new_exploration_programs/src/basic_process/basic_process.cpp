#include <new_exploration_programs/basic_process.h>


float BasicProcess::forward_vel;
float BasicProcess::omega;
float BasicProcess::pre_theta;
geometry_msgs::Twist BasicProcess::vel;
std::vector<float> BasicProcess::odom_log_x;//オドメトリxの履歴を保存
std::vector<float> BasicProcess::odom_log_y;//オドメトリyの履歴を保存
int BasicProcess::loop_count;
kobuki_msgs::Led BasicProcess::led1;
kobuki_msgs::Led BasicProcess::led2;

BasicProcess::BasicProcess()
{
	/*subscribeoptions*/
	bp1.setCallbackQueue(&bumper_queue);
	bp2.setCallbackQueue(&odom_queue);
	bp3.setCallbackQueue(&loop_queue);
	bp4.setCallbackQueue(&map_queue);
	bp5.setCallbackQueue(&scan_queue);
	/*subscriber*/
	bumper_sub = bp1.subscribe("/bumper_info",1,&BasicProcess::bumper_callback,this);
	odom_sub = bp2.subscribe("/odom_support",1,&BasicProcess::odom_callback,this);
	loop_sub = bp3.subscribe("/loop_counter",1,&BasicProcess::loop_callback,this);
	map_sub = bp4.subscribe("/map",1,&BasicProcess::map_callback,this);
	scan_sub = bp5.subscribe("/scan",1,&BasicProcess::scan_callback,this);
	/*publisher*/
	marker_pub = bp.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	which_pub = bp.advertise<geometry_msgs::Point>("/which_based", 1);
	vel_pub = bp.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	led1_pub = bp.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
	led2_pub = bp.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);

	which_bumper = 0;
	bumper_hit = false;
	omega = 0;
	loop_count = 0;
	pre_theta = 0;

	forward_vel = 0.2;
	rotate_vel = 0.5;
	scan_threshold = 0.8;
};
