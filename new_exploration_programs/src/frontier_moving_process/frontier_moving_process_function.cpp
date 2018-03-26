#include <new_exploration_programs/frontier_moving_process.h>
//#include <new_exploration_programs/frontier_map_process.h>
//#include <new_exploration_programs/basic_process.h>
//#include <new_exploration_programs/avoidance_process.h>

void FrontierMovingProcess::VFH_navigation(void){

	//FrontierMapProcess fmp;
	//BasicProcess bp;

	float goal_point_x;
	float goal_point_y;

	fmap.get_flogoal(&goal_point_x, &goal_point_y);
	bp.display_gravity(goal_point_x, goal_point_y);

	const float goal_margin = 1.0;//0.8;//0.5;
	float now2goal_dis = 100.0;
	float pre_now2goal_dis;

	float diff = 0;
	int cancel_count = 0;
	int end_count = 1;
	int keisu = -1;
	float diff_th = 0.1;

	//float odom_x;
	//float odom_y;

	bp.get_odom(&odom_x, &odom_y, &yaw);

	std::cout << "目標へ移動開始" << std::endl;
	std::cout << "goal(" << goal_point_x << "," << goal_point_y << ")" << std::endl;
	std::cout << "now(" << odom_x << "," << odom_y << ")\n" << std::endl;

	bool first_move = true;

	now2goal_dis = sqrt(pow(goal_point_x-odom_x,2)+pow(goal_point_y-odom_y,2));

	//目標をキャンセルする条件を近→離1→近2→離3に変更する

	while(now2goal_dis > goal_margin && ros::ok()){
		//scan_queue.callOne(ros::WallDuration(1));//重力の影響を受けた進行方向を決めて速度を送る

		VFH_gravity(goal_point_x, goal_point_y, odom_x, odom_y);//地図上での目標に対する角度を決定
		VFH_goal_angle();//ロボット座標系での目標角度を決定

		if(first_move){
			reverse_check();
			first_move = false;
		}

		//AvoidanceProcessにいれるbumper_che();
		ap.bumper_avoidance();
		VFH_moving();//ロボットを動かす

		//odom_queue.callOne(ros::WallDuration(1));//自分のオドメトリ取得
		bp.get_odom(&odom_x, &odom_y, &yaw);

		std::cout << "goal(" << goal_point_x << "," << goal_point_y << ")" << std::endl;
		std::cout << "now(" << odom_x << "," << odom_y << ")\n" << std::endl;
		pre_now2goal_dis = now2goal_dis;
		now2goal_dis = sqrt(pow(goal_point_x-odom_x,2)+pow(goal_point_y-odom_y,2));

		diff +=  now2goal_dis - pre_now2goal_dis;
		if(std::abs(diff) > diff_th){
			if(diff*keisu < 0){
				keisu *= -1;
				cancel_count++;
			}
			diff = 0;
		}
		if(cancel_count == end_count){
			std::cout << "距離が遠くなったためbreak" << std::endl;
			std::cout << "目標への移動不可" << std::endl;
			return;
		}
	}

	fmap.set_pregoalpoint(goal_point_x, goal_point_y);
	//pre_goal_point_x = goal_point_x;
	//pre_goal_point_y = goal_point_y;

	std::cout << "目標へ移動終了" << std::endl;
}

void FrontierMovingProcess::VFH_gravity(float goal_point_x, float goal_point_y, float odom_x, float odom_y)
{//引力の影響を受けた目標角度を決める
	gra_angle = atan2(goal_point_y - odom_y,goal_point_x - odom_x);//ロボットの現在座標から目標に対してのベクトル座標//-180~180
}

void FrontierMovingProcess::VFH_goal_angle(void){

	//BasicProcess bp;

	std::vector<float> ranges;
	float angle_min;
	float angle_max;
	float angle_increment;

	std::vector<float> angles;

	float scan_threshold;

	const float safe_space = 0.6;
	const float forward_dis = 0.75;

	bp.get_scanthreshold(&scan_threshold);

	bp.get_scan(&ranges, &angle_min, &angle_max, &angle_increment);

	bp.approx(ranges);

	for(int i=0;i<ranges.size();i++){
		angles.push_back(angle_min+(angle_increment*i));
	}

	float rad_min = no_goal;
	int i;
	float over_rad;
	int near_i;
	float gra_angle_abs = 3.14;
	float gra_angle_diff;
	int safe_num;
	int rad_counter = 0;
	int start_i;
	int j;
	int plus_rad_i = angles.size();
	int minus_rad_i = angles.size();
	float pd;
	float md;
	float x_g;
	float y_g;
	float ang_g;
	int l;

//gra_angleを中心にして±に角度を見ていって先にロボットの直径分の空間が見つかったらその時点でその中心を目標にする
//gra_angleをロボットの座標軸で表すgra_angle_r

	gra_angle_r = gra_angle - yaw;

	if(gra_angle_r < -M_PI){
		gra_angle_r = 2*M_PI + gra_angle_r;
	}

	if(gra_angle_r > M_PI){
		gra_angle_r = -2*M_PI + gra_angle_r;
	}


//gra_angle_rと角度が最も近くなるiの番号を調べる
	for(i=0;i<angles.size();i++){
		gra_angle_diff = std::abs(gra_angle_r - angles[i]);
		if(gra_angle_diff < gra_angle_abs){
			gra_angle_abs = gra_angle_diff;
			near_i = i;
		}
	}
//安全な角度を作るために必要な個数
	safe_num = (asin((safe_space)/(2*forward_dis))) / angle_increment ;
	std::cout << "angle_increment:" << angle_increment << ",scan_threshold:" << scan_threshold << std::endl;
//gra_angle_rに一番近いiから±に安全角度を見つける,プラス側に行って半径分領域があったらそのマイナス側に半径分の領域があるかを見る
	//先にプラス側を見る
	for(i=near_i;i<angles.size();i++){
		//std::cout << "plusのfor" << std::endl;
		if(ranges[i] > scan_threshold){
			start_i = i;
			l = i;
			rad_counter = 0;
			while(ranges[l] > scan_threshold && rad_counter < safe_num && l < angles.size()-1){//iの限界
				rad_counter++;
				l++;
			}
			if(rad_counter == safe_num && start_i >= safe_num){//プラス側はOKなのでマイナス側を見に行く
				rad_counter = 0;
				for(j=start_i;j>start_i-safe_num;j--){
					if(ranges[j] > scan_threshold && rad_counter < safe_num){
						rad_counter++;
					}
					//std::cout << "j:" << j << std::endl;
				}
				if(rad_counter == safe_num){
					plus_rad_i = start_i;
					//std::cout << "breakした" << std::endl;
					break;
				}
			}
		}
	}
	//std::cout << "plusのfor抜けた" << std::endl;

	for(i=near_i;i>=0;i--){
		//std::cout << "minusのfor" << std::endl;
		if(ranges[i] > scan_threshold){
			start_i = i;
			l = i;
			rad_counter = 0;
			while(ranges[l] > scan_threshold && rad_counter < safe_num && l > 1){
				rad_counter++;
				l--;
			}
			if(rad_counter == safe_num && start_i <= angles.size()-safe_num){//マイナス側はOKなのでプラス側を見に行く//ここやってない//やった
				rad_counter = 0;
				for(j=start_i;j<start_i+safe_num;j++){
					if(ranges[j] > scan_threshold && rad_counter < safe_num){
						rad_counter++;
					}
				}
				if(rad_counter == safe_num){
					minus_rad_i = start_i;
					//std::cout << "breakした" << std::endl;
					break;
				}
			}
		}
	}
	//std::cout << "minusのfor抜けた" << std::endl;



	//plusとminusでnear_iから近い方を選択してrad_minに入れる

	if(plus_rad_i != angles.size() || minus_rad_i != angles.size()){
		pd = std::abs(angles[near_i] - angles[plus_rad_i]);
		md = std::abs(angles[near_i] - angles[minus_rad_i]);

		if(plus_rad_i == angles.size()){
			pd = 100;
		}

		if(minus_rad_i == angles.size()){
			md = 100;
		}

		if(pd<=md){
			rad_min = angles[plus_rad_i];
		}
		else{
			rad_min = angles[minus_rad_i];
		}


		//目標角度をグローバルで
		ang_g = yaw + rad_min;
		if(ang_g > M_PI){
			over_rad = ang_g - M_PI;
			ang_g = -M_PI + over_rad;
		}
		if(ang_g < -M_PI){
			over_rad = ang_g + M_PI;
			ang_g = M_PI - over_rad;
		}

		//odom_queue.callOne(ros::WallDuration(1));

		x_g = (scan_threshold/2) * cos(ang_g) + odom_x;
		y_g = (scan_threshold/2) * sin(ang_g) + odom_y;

		bp.display_goal_angle(x_g, y_g);

	}

	goal_angle = rad_min;

	std::cout << "gra_angle:" << gra_angle << std::endl;
	std::cout << "yaw:" << yaw << std::endl;
	std::cout << "gra_angle_r:" << gra_angle_r << std::endl;
	std::cout << "near_i:" << near_i  << ", near_i_angle:" << angles[near_i] << std::endl;
	std::cout << "safe_num:" << safe_num << std::endl;
	std::cout << "plus_i: " << plus_rad_i << ", plus_i_rad:" << angles[plus_rad_i] << std::endl;
	std::cout << "minus_i: " << minus_rad_i << ", minus_i_rad:" << angles[minus_rad_i] << std::endl;
	std::cout << "rad_min:" << rad_min << std::endl;
}

void FrontierMovingProcess::reverse_check(void){

	//BasicProcess bp;

	float reverse_threshold = M_PI/4;
	float vel_x,vel_z;
	float rotate_vel;

	bp.get_rotatevel(&rotate_vel);

	//ここでgra_angle_rの確認をする
	if(std::abs(gra_angle_r) >= M_PI/2){
		vel_z = rotate_vel*(gra_angle-yaw)/std::abs(gra_angle-yaw);
		vel_x = 0;
		bp.set_vel(vel_x, vel_z);
		//yawとgra_angleの差が小さくなるまで回る
		while(ros::ok() && std::abs(gra_angle-yaw) > reverse_threshold){
			bp.pub_vel();
			bp.get_odom(&odom_x, &odom_y, &yaw);
		}
		VFH_goal_angle();
	}
}

void FrontierMovingProcess::VFH_moving(void)
{
	const float g_angle = goal_angle;	

	if(g_angle == no_goal)
	{
		//scan_rotateのやつ
		ap.obstacle_avoidance();
	}

	else//(ないとき)
	{
		bp.vel_curve_VFH(g_angle,0.7);
	}
}
