#include <ros/ros.h>


class BasicProcess;

class AvoidanceProcess
{
private:
	BasicProcess *bp;
	float avoidance_sign;//1.0
public:
	/*コンストラクタ*/
	AvoidanceProcess();
	/*デストラクタ*/
	~AvoidanceProcess(){};
	void obstacle_avoidance(void);
	void bumper_avoidance(void);
	void go_back(void);
	//void sen_vel_recovery()
	//void sen_vel_recovery_g()
	//bumper回避のやつここに入れる

};
