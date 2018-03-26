#ifndef INCLUDE_GARDE_FMOP
#define INCLUDE_GARDE_FMOP

#include <ros/ros.h>
#include <new_exploration_programs/basic_process.h>
#include <new_exploration_programs/frontier_map_process.h>
#include <new_exploration_programs/avoidance_process.h>

//class BasicProcess;
//class FrontierMapProcess;
//class AvoidanceProcess;

class FrontierMovingProcess
{
private:
	float gra_angle;
	float gra_angle_r;
	float goal_angle;

	float odom_x;
	float odom_y;
	double yaw;
	float no_goal;//3.14

	//BasicProcess *bp;
	//FrontierMapProcess *fmap;
	//AvoidanceProcess *ap;

	BasicProcess bp;
	FrontierMapProcess fmap;
	AvoidanceProcess ap;
public:
	/*コンストラクタ*/
	FrontierMovingProcess();
	/*デストラクタ*/
	~FrontierMovingProcess(){};
	void VFH_navigation(void);
	void VFH_gravity(float goal_point_x, float goal_point_y, float odom_x, float odom_y);
	void VFH_goal_angle(void);
	void reverse_check(void);//なんか回転するやつ
	void VFH_moving(void);
};

#endif
