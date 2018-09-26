#include <class_test/duplicate_process.h>

bool DuplicateProcess::duplicate_detection(void)
{
	float global_x;//分岐領域の世界座標
	float global_y;//分岐領域の世界座標
	float x_margin_plus;
	float x_margin_minus;
	float y_margin_plus;
	float y_margin_minus;
	const float duplication_margin = 1.0;

	BasicProcess bp;

	float odom_x,odom_y;
	double yaw;

	bp.odom_get(&odom_x, &odom_y, &yaw);

	bool branch_find_flag;///これはあとで他のクラスから受け取るようにする set関数を使って書き換える

	float goal_x;//目標を決定するクラスからもらってくるようにする
	float goal_y;

	global_x = odom_x+(cos(yaw)*goal_x) - (sin(yaw)*goal_y);
	global_y = odom_y+(cos(yaw)*goal_y) + (sin(yaw)*goal_x);

	x_margin_plus = global_x + duplication_margin;
	x_margin_minus = global_x - duplication_margin;
	y_margin_plus = global_y + duplication_margin;
	y_margin_minus = global_y - duplication_margin;


	std::cout << "odom_x,odom_y (" << odom_x << "," << odom_y << ")" << std::endl;
	std::cout << "goal_x,goal_y (" << goal_x << "," << goal_y << ")" << std::endl;
	std::cout << "yaw (" << yaw << ")" << std::endl;
	std::cout << "global_x,global_y (" << global_x << "," << global_y << ")" << std::endl;
	

	std::vector<float> odom_log_x;
	std::vector<float> odom_log_y;

	bp.odom_log_get(&odom_log_x, &odom_log_y);

	int pre_odom_num = 600;
	int odom_num_init;

	if(odom_log_x.size()>pre_odom_num){
		odom_num_init = odom_log_x.size() - pre_odom_num;
	}
	else{
		odom_num_init = 0;
	}


	for(int i=odom_num_init;i<odom_log_x.size();i++){
		//過去のオドメトリが許容範囲の中に入っているか//
		if((x_margin_plus >= odom_log_x[i]) && (x_margin_minus <= odom_log_x[i])){
			if((y_margin_plus >= odom_log_y[i]) && (y_margin_minus <= odom_log_y[i])){
				//duplication_flag = true;
				branch_find_flag = false;
				std::cout << "すでに探査した領域でした・・・ぐすん;;" << std::endl;
				return true;
			}
		}
	}
	
	return false;
}
