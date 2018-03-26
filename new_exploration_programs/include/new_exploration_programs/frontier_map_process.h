#ifndef INCLUDE_GARDE_FMAP
#define INCLUDE_GARDE_FMAP

#include <ros/ros.h>
#include <new_exploration_programs/basic_process.h>

//class BasicProcess;

class FrontierMapProcess
{
private:
//	ros::NodeHandle fmp;
	/*publisher*/
	//ros::Publisher led2_pub;
	/*変数*/
	int fro_num;
	std::vector<float> fro_x;//見つけたフロンティアのx座標
	std::vector<float> fro_y;//見つけたフロンティアのy座標

	//bool first_cycle;

	static float flo_goal_x;
	static float flo_goal_y;

	float pre_vector_x;
	float pre_vector_y;

	bool stop;

	//kobuki_msgs::Led led2;

	static float pre_goal_point_x;//移動用関数から持ってくるようにする
	static float pre_goal_point_y;

	//BasicProcess *bp;
	BasicProcess bp;

public:
	/*コンストラクタ*/
	FrontierMapProcess();
	/*デストラクタ*/
	~FrontierMapProcess(){};
	void frontier_search(void);
	void choose_goal_frontier(void);

	void get_flogoal(float *gx, float *gy);
	void set_pregoalpoint(float x, float y);
	bool stop_check(void);
};

#endif
