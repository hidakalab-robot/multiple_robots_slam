#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>//地図データ取得用
#include <geometry_msgs/Twist.h>//回転速度送信用
#include <fstream>//ファイル出力用
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Point.h>
#include <kobuki_msgs/Led.h>



//グローバル変数////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher vel_pub;
ros::Publisher marker_pub;
ros::Publisher which_pub;
ros::Publisher led1_pub;
ros::Publisher led2_pub;

ros::CallbackQueue bumper_queue;
ros::SubscribeOptions bumper_option;
ros::Subscriber bumper_sub;

ros::CallbackQueue map_queue;
ros::SubscribeOptions map_option;
ros::Subscriber map_sub;

ros::CallbackQueue scan_queue;
ros::SubscribeOptions scan_option;
ros::Subscriber scan_sub;

ros::CallbackQueue odom_queue;
ros::Subscriber odom_sub;
ros::SubscribeOptions odom_option;

ros::CallbackQueue scan_branch_queue;
ros::SubscribeOptions scan_branch_option;
ros::Subscriber scan_branch_sub;

ros::CallbackQueue wall_queue;
ros::SubscribeOptions wall_option;
ros::Subscriber wall_sub;

ros::CallbackQueue scan_rotate_queue;
ros::SubscribeOptions scan_rotate_option;
ros::Subscriber scan_rotate_sub;

geometry_msgs::Twist vel;
visualization_msgs::Marker marker3;
kobuki_msgs::Led led2;

float Emergency_avoidance = 0;
bool undecided_rotate = false;

float odom_x;//オドメトリx
float odom_y;//オドメトリy
double yaw;//ヨー角

bool stop = false;

float pre_vector_x = 0;
float pre_vector_y = 0;

float pre_goal_point_x = 0;
float pre_goal_point_y = 0;

float goal_point_x;
float goal_point_y;

float vfh_gra_x;
float vfh_gra_y;

float gra_angle;
float gra_angle_r;

bool first_cycle = false;//true;
float scan_angle;//この角度の範囲内に空間があれば回転を終了する
const float scan_branch_limit = 1.5;//分岐方向への回転をセンサデータから行うときにこの値以上だったら数値があっても良い
bool scan_rotation_ok = false;//スキャンデータからの分岐回転を終了していいか
bool need_back = true;//全部nanだったときに最初だけバックする
const float back_vel = -0.2;//VFHで全部nanだったときの後退速度[m/s]
const float back_time = 0.5;//VFHで全部nanだったときに後退する時間[s]
ros::Time set_time;//時間制限系while文の開始時間
bool need_rotate_calc = true;//全部nanだったときの回転方向を計算する必要があるか
float pre_theta = 0;
const float PI = 3.1415926;//円周率π
const float forward_vel = 0.2;//前進速度[m/s]
const float rotate_vel = 0.5;//回転速度[rad\s]
const float obst_recover_angle = 0.09;//リカバリー回転のときこの角度の±の範囲に障害物がなければ回転終了
const float forward_dis = 0.75;//一回のVFHで前方向に進む距離[m]
const float scan_threshold = 0.8;//VFHでの前方の安全確認距離(この距離以内に障害物がなければ安全と判断)[m]
const float safe_space = 0.6;//ロボットの直径(VFHでこの値以上に空間があれば安全と判断)[m]

bool bumper_hit = false;
int which_bumper = 0;
bool first_move = false;
bool reverse_flag = false;

//引力を考慮した回転方向が決まったらこの関数を使う//地図情報に基づいて回転方向を決定//引力方向に回ると反転しそうだったら//両側にあったら引力
void rotation_based_map(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
	const nav_msgs::MapMetaData info = map_msg->info;//地図の設定を取得
	const std::vector<int8_t> data = map_msg->data;//地図の値を取得
	const int x = info.width;//地図の横サイズ
	const int y = info.height;//地図の縦サイズ
	const float m_per_cell = info.resolution;
	const float low_left_x = info.origin.position.x;//地図の左下のx座標
	const float low_left_y = info.origin.position.y;//地図の左下のy座標
	//ここでもnewで確保したほうがいいかも
	int8_t map_array[x][y];//地図を行列に格納
	int i;
	int j;
	int k;
	float rotation;
	bool wall_hit = false;

	for(i=0;i<y;i++){
    		for(j=0;j<x;j++){
      			map_array[j][i] = data[k];
      			k++;
    		}
  	}

	//引力によって決定した回転方向
	rotation = vel.angular.z / std::abs(vel.angular.z);
	std::cout << "kaiten1:" << vel.angular.z << std::endl;

	//自分の座標が配列だとどこなのかを計算
	odom_queue.callOne(ros::WallDuration(1));
	int roa_x = (odom_x - low_left_x)/m_per_cell;
	int roa_y = (odom_y - low_left_y)/m_per_cell;

	//自分の周辺を定義
	float around_dis = scan_threshold;
	int adoa = around_dis/m_per_cell;

	std::cout << "around_dis:" << around_dis << std::endl;
	std::cout << "m_per_cell:" << m_per_cell << std::endl;
	std::cout << "adoa:" << adoa << std::endl;
	std::cout << "roa:(" << roa_x << "," << roa_y << ")" << std::endl;

	//90度方向に障害物がなければおｋ
	//ヨー角が0度の時のmax座標
	double yaw0x = adoa;
	double yaw0y = 0;
	std::cout << "yaw0:(" << yaw0x+roa_x << "," << yaw0y+roa_y << ")" << std::endl;
	//現在のヨー角で見た時のmax座標
	double yaw1x = cos(yaw)*yaw0x-sin(yaw)*yaw0y;
	double yaw1y = sin(yaw)*yaw0x+cos(yaw)*yaw0y;
	std::cout << "yaw1:(" << yaw1x+roa_x << "," << yaw1y+roa_y << ")" << std::endl;
	//回転予定の方向に90度回転した時のmax座標
	int yaw2x = cos(rotation*PI/2+yaw)*yaw1x-sin(rotation*PI/2+yaw)*yaw1y+roa_x;
	int yaw2y = sin(rotation*PI/2+yaw)*yaw1x+cos(rotation*PI/2+yaw)*yaw1y+roa_y;
	//現在座標と回転後のx座標の差の数だけfor文で代入
	int diff_x = yaw2x - roa_x;
	int diff_y = yaw2y - roa_y;

	std::cout << "diff_x:" << diff_x << std::endl;//diff_xが０になるとまずい
	std::cout << "diff_y:" << diff_y << std::endl;
	
	std::cout << "start:(" << roa_x << "," << roa_y << "), end:(" << yaw2x << "," << yaw2y << ")" << std::endl;


	if(std::abs(diff_x) >= std::abs(diff_y)){
		std::cout << "xだよ" << std::endl;
		for(i=roa_x;i<=roa_x+std::abs(diff_x);i++){
			j = roa_x + (i - roa_x)*(diff_x/std::abs(diff_x));
			std::cout << "x:" << j << ", y:" << int((double(diff_y)/double(diff_x))*(i-roa_x)+roa_y) << std::endl;
			if(map_array[j][int((double(diff_y)/double(diff_x))*(i-roa_x)+roa_y)] == 100){
				wall_hit = true;
				std::cout << "hit!!" << std::endl;
				break;
			}
		}
	}

	else{
		std::cout << "yだよ" << std::endl;
		for(i=roa_y;i<=roa_y+std::abs(diff_y);i++){
			j = roa_y + (i - roa_y)*(diff_y/std::abs(diff_y));
			std::cout << "x:" << int((double(diff_x)/double(diff_y))*(i-roa_y)+roa_x) << ", y:" << j << std::endl;
			if(map_array[int((double(diff_x)/double(diff_y))*(i-roa_y)+roa_x)][j] == 100){
				wall_hit = true;
				std::cout << "hit!!" << std::endl;
				break;
			}
		}
	}
	if(wall_hit){
		vel.angular.z *= -1;
	}
	std::cout << "kaiten2:" << vel.angular.z << std::endl;
}

void odom_marking(float x, float y){
	geometry_msgs::Point marking;
	marking.x = x;
	marking.y = y;
	marking.z = 1.0;
	which_pub.publish(marking);
}

void bumper(const kobuki_msgs::BumperEvent::ConstPtr& hit_msg){
	if(hit_msg -> state == 1){
		bumper_hit = true;
		which_bumper = hit_msg -> bumper;
		std::cout << "bumper_hit" << std::endl;
	}
	else{
		bumper_hit = false;
		std::cout << "no_bumper_hit" << std::endl;
	}
}

void display_goal_angle(float x, float y){
	uint32_t arrow = visualization_msgs::Marker::ARROW;
	visualization_msgs::Marker marker1;
	marker1.header.frame_id = "map";
	marker1.header.stamp = ros::Time::now();
	marker1.ns = "basic_shapes";
	marker1.id = 0;
	marker1.type = arrow;
	marker1.action = visualization_msgs::Marker::ADD;
	geometry_msgs::Point pgo0;
	geometry_msgs::Point pgo1;

	odom_queue.callOne(ros::WallDuration(1));
	
	pgo0.x = odom_x;
	pgo0.y = odom_y;
	pgo0.z = 0.0;

	pgo1.x = x;
	pgo1.y = y;
	pgo1.z = 0.0;

	marker1.points.push_back(pgo0);
	marker1.points.push_back(pgo1);

	marker1.scale.x = 0.1;
	marker1.scale.y = 0.5;
	marker1.scale.z = 0.0;

	marker1.color.r = 0.0f;
	marker1.color.g = 1.0f;
	marker1.color.b = 0.0f;
	marker1.color.a = 1.0;

	marker1.lifetime = ros::Duration(0);

	marker_pub.publish(marker1);
}
void display_gravity(float x, float y){
	uint32_t line = visualization_msgs::Marker::LINE_STRIP;
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "map";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "basic_shapes";
	marker2.id = 1;
	marker2.type = line;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.orientation.w = 1.0;
	geometry_msgs::Point pgr0;
	geometry_msgs::Point pgr1;

	odom_queue.callOne(ros::WallDuration(1));

	pgr0.x = odom_x;
	pgr0.y = odom_y;
	pgr0.z = 0.0;

	pgr1.x = x;
	pgr1.y = y;
	pgr1.z = 0.0;

	marker2.points.push_back(pgr0);
	marker2.points.push_back(pgr1);

	marker2.scale.x = 0.1;

	marker2.color.r = 1.0f;
	marker2.color.g = 0.0f;
	marker2.color.b = 0.0f;
	marker2.color.a = 1.0;

	marker2.lifetime = ros::Duration(0);

	marker_pub.publish(marker2);
}

void scan_branch_callback(const sensor_msgs::LaserScan::ConstPtr& scan_Branch_msg){
	std::vector<float> ranges = scan_Branch_msg->ranges;
	float angle_increment = scan_Branch_msg->angle_increment;

	int scan_width = scan_angle/angle_increment;

	int scan_i_min = (ranges.size()/2)-1 - scan_width;
	int scan_i_max = (ranges.size()/2) + scan_width + 1;

	for(int i = scan_i_min;i<scan_i_max;i++){
		if(!isnan(ranges[i]) && ranges[i] < scan_branch_limit){
			scan_rotation_ok = false;
			std::cout << "rotation_return(debag)" << std::endl;
			return;
		}
	}

	scan_rotation_ok = true;
	std::cout << "rotation_true(debag)" << std::endl;

}


void vel_recovery(){

	float gra_recov;
	float over;

	
	if(need_back){
		std::cout << "前に進めないよ;;" << std::endl;

		vel.angular.z = 0;
		vel.linear.x = back_vel;

		ros::Duration duration(back_time);
		
	
		std::cout << "バック♪バック♪" << std::endl;

		set_time = ros::Time::now();	
	
		while(ros::Time::now()-set_time < duration){
			vel_pub.publish(vel);
		}
		need_back = false;
	}
		
	std::cout << "かいて〜ん!!" << std::endl;

	odom_queue.callOne(ros::WallDuration(1));

	gra_recov = gra_angle - yaw;

	if(gra_recov > PI){
		over = gra_recov - PI;
		gra_recov = -PI + over;
	}

	if(gra_recov < -PI){
		over = gra_recov + PI;
		gra_recov = PI + over;
	}


	std::cout << "gra_angle_r: " << gra_angle_r << std::endl;
	std::cout << "yaw: " << yaw << std::endl;
	std::cout << "gra_recov: " << gra_recov << std::endl;

	if(need_rotate_calc){
		if(gra_recov >=0){
			vel.angular.z = rotate_vel;
		}
		else{
			vel.angular.z = -rotate_vel;
		}


		need_rotate_calc = false;
	}

	wall_queue.callOne(ros::WallDuration(1));

	vel.linear.x = 0;

	scan_angle = obst_recover_angle;

	if(bumper_hit){
		if(which_bumper == 0){
			vel.angular.z = -rotate_vel;
		}

		else if(which_bumper == 1){

			if(pre_theta > 0 ){
				vel.angular.z = -rotate_vel;
			}
			else{
				vel.angular.z = rotate_vel;
			}
		}
		
		else{
			vel.angular.z = rotate_vel;
		}	
	
		ros::Duration duration2(back_time+1.0);
		set_time = ros::Time::now();
		while(ros::Time::now()-set_time < duration2){
			vel_pub.publish(vel);
		}		

	}
	else{
		while(!scan_rotation_ok && ros::ok()){
			if(!scan_rotation_ok){
				std::cout << "rotation_return(debag)なので速度送信" << std::endl;
				vel_pub.publish(vel);
			}
		
			scan_branch_queue.callOne(ros::WallDuration(1));//scan_angleに設定した角度の範に空間ができるまで回転する
		}
		std::cout << "rotation_true(debag)だったので終わり" << std::endl;
		scan_rotation_ok = false;
	}
}

//速度などを引数で使えるようにした速度送信関数
void vel_curve_VFH2(float theta,float v,float t){
	float theta_rho;
	float omega;

	pre_theta = theta;

	theta_rho = 2*theta;
	omega = theta_rho/t;

	vel.linear.x = v;
	vel.angular.z = omega;

	vel_pub.publish(vel);
	std::cout << "障害物を回避しながら移動中♪" << std::endl;

	odom_queue.callOne(ros::WallDuration(1));
	odom_marking(odom_x,odom_y);
}

float VFH_move_angle(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan, float gra_angle, std::vector<float> &angles){
	float rad_min = all_nan;
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
	
	if(gra_angle_r < -PI){
		gra_angle_r = 2*PI + gra_angle_r;
	}

	if(gra_angle_r > PI){
		gra_angle_r = -2*PI + gra_angle_r;
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
		if(ang_g > PI){
			over_rad = ang_g - PI;
			ang_g = -PI + over_rad;
		}
		if(ang_g < -PI){
			over_rad = ang_g + PI;
			ang_g = PI - over_rad;
		}

		odom_queue.callOne(ros::WallDuration(1));

		x_g = (scan_threshold/2) * cos(ang_g) + odom_x;
		y_g = (scan_threshold/2) * sin(ang_g) + odom_y;

		display_goal_angle(x_g, y_g);	

	}

	std::cout << "gra_angle:" << gra_angle << std::endl;
	std::cout << "yaw:" << yaw << std::endl;
	std::cout << "gra_angle_r:" << gra_angle_r << std::endl;
	std::cout << "near_i:" << near_i  << ", near_i_angle:" << angles[near_i] << std::endl;
	std::cout << "safe_num:" << safe_num << std::endl;
	std::cout << "plus_i: " << plus_rad_i << ", plus_i_rad:" << angles[plus_rad_i] << std::endl;
	std::cout << "minus_i: " << minus_rad_i << ", minus_i_rad:" << angles[minus_rad_i] << std::endl;
	std::cout << "rad_min:" << rad_min << std::endl;


	return rad_min;
}



void approx(std::vector<float> &scan){
	float depth,depth1,depth2;
	depth=0;
	depth1=0;
	depth2=0;

	for(int j=0,count=0;j<scan.size()-1;j++){
		depth=scan[j];
		//|val|nan|のとき
		//ROS_INFO("(%d,%d):",i,j);

		if(!isnan(depth) && isnan(scan[j+1])){
			depth1=depth;
			count++;
			//ROS_INFO("!nan&nan(%d,%d)",j,j+1);
		}

		if(isnan(depth)){
	//|nan|nan|の区間
			if(isnan(scan[j+1])){
				count++;
				//ROS_INFO("nan&nan(%d,%d)",j,j+1);
			}
	//|nan|val|のとき
			else{
				//ROS_INFO("nan&!nan(%d,%d)",j,j+1);
				depth2=scan[j+1];
	//左端がnanのとき
				if(isnan(depth1)){
					for(int k=0;k<count+1;k++)
						scan[j-k]=0.01;//depth2;
				}
				else{
					for(int k=0;k<count;k++)
						scan[j-k]=depth2-(depth2-depth1)/(count+1)*(k+1);
				}
				//ROS_INFO("nan|val|:nancount=%d",count);
				count=0;
			}
		}
	//右端がnanのとき
		if(j==(scan.size()-1)-1 &&isnan(scan[j+1])){
			for(int k=0;k<count;k++)
				scan[j+1-k]=0.01;//depth1;
			//ROS_INFO("val|nan|nan|:nancount=%d",count);
			count=0;
		}
	}

	if(isnan(scan[0])){
		scan[0] = scan[1] - (scan[2] - scan[1]);
		if(scan[0] < 0){
			scan[0] = 0;
		}
	}
}

void scan_rotate_callback(const sensor_msgs::LaserScan::ConstPtr& src_msg){
	float plus_ave;
	float minus_ave;
	float sum = 0;
	float range_threshold = 1.0;
	const float angle_max = src_msg->angle_max;

	std::vector<float> ranges = src_msg->ranges;

	approx(ranges);

	//minus側の平均
	for(int i=0;i<ranges.size()/2;i++){
		if(!isnan(ranges[i])){	
			sum += ranges[i];
		}
	}
	minus_ave = sum/(ranges.size()/2);
	
	sum = 0;

	//plus側
	for(int i=ranges.size()/2;i<ranges.size();i++){
		if(!isnan(ranges[i])){
			sum += ranges[i];
		}
	}
	plus_ave = sum/(ranges.size()/2);

	//平均を比較

	std::cout << "plus_ave:" << plus_ave << std::endl;
	std::cout << "minus_ave:" << minus_ave << std::endl;

	if(plus_ave < range_threshold && minus_ave < range_threshold){
		vel_curve_VFH2(Emergency_avoidance*angle_max/6,0.0,0.3);
	}
	else{
		if(plus_ave > minus_ave){
			std::cout << "plusに回転\n" << std::endl;
			Emergency_avoidance = 1.0;
			vel_curve_VFH2(Emergency_avoidance*angle_max/6,forward_vel,0.3);
		}
		else if(plus_ave < minus_ave){
			std::cout << "minusに回転\n" << std::endl;
			Emergency_avoidance = -1.0;
			vel_curve_VFH2(Emergency_avoidance*angle_max/6,forward_vel,0.3);
		}
		else{
			std::cout << "無理です\n" << std::endl;	
			undecided_rotate = true;
		}
	}
}


void reverse(){
	float reverse_threshold = PI/4;
	//yawとgra_angleの差が小さくなるまで回る
	//vel.angular.z = 0.5;
	vel.angular.z = rotate_vel*(gra_angle-yaw)/std::abs(gra_angle-yaw);
	vel.linear.x = 0;
	
	while(ros::ok() && std::abs(gra_angle-yaw) > reverse_threshold){
		vel_pub.publish(vel);
		odom_queue.callOne(ros::WallDuration(1));
	}	
}

void VFH_gravity(const sensor_msgs::LaserScan::ConstPtr& scan_msg){//引力の影響を受けた目標角度を決める
	vfh_gra_x = goal_point_x - odom_x;
	vfh_gra_y = goal_point_y - odom_y;
	float goal_angle;

	display_gravity(goal_point_x, goal_point_y);

	gra_angle = atan2(vfh_gra_y,vfh_gra_x);//ロボットの現在座標から目標に対してのベクトル座標//-180~180

//ここからセンサベースと同じVFH/////////////////////////////////////////////////////

	const float angle_min = scan_msg->angle_min;
	const float angle_max = scan_msg->angle_max;
	const float angle_increment = scan_msg->angle_increment;
	const float all_nan = 3.14;

	std::vector<float> ranges = scan_msg->ranges;
	float rad;
	std::vector<float> angles;

	for(int i=0;i<ranges.size();i++){
		rad = angle_min+(angle_increment*i);
		angles.push_back(rad);
	}

	approx(ranges);

	goal_angle = VFH_move_angle(ranges,angle_min,angle_increment,all_nan,gra_angle,angles);

	//目標を設定した一回目だけ行う
	if(first_move){
		first_move = false;
	//ここでgra_angle_rが±150度くらいになったら反転する処理を入れる
		if(std::abs(gra_angle_r) >= PI/2){
			reverse();//反転する関数
			reverse_flag = true;
			return;
		}
	}
	bumper_queue.callOne(ros::WallDuration(1));

	if(bumper_hit){
		vel_recovery();
	}
	else if(goal_angle >= all_nan){
		scan_rotate_queue.callOne(ros::WallDuration(1));
		if(undecided_rotate){
			vel_recovery();
			undecided_rotate = false;
		}
		/*else{
			vel_curve_VFH_e(Emergency_avoidance*angle_max/6,angle_max);
		}*/
	}
	else{
		need_back = true;
		need_rotate_calc = true;
		//vel_curve_VFH(goal_angle, angle_max);
		vel_curve_VFH2(goal_angle,forward_vel,0.7);		
	}
}

void VFH_navigation(float goal_x, float goal_y){
	goal_point_x = goal_x;
	goal_point_y = goal_y;
	const float goal_margin = 1.0;//0.8;//0.5;
	float now2goal_dis = 100.0;
	float pre_now2goal_dis;

	//float sum_diff = 0;
	//const float cancel_diff = 0;

	float diff = 0;
	int cancel_count = 0;
	int end_count = 1;
	int keisu = -1;
	float diff_th = 0.1;
	

	std::cout << "目標へ移動開始" << std::endl;
	std::cout << "goal(" << goal_point_x << "," << goal_point_y << ")" << std::endl;
	odom_queue.callOne(ros::WallDuration(1));//自分のオドメトリ取得
	std::cout << "now(" << odom_x << "," << odom_y << ")\n" << std::endl;

	first_move = true;

	now2goal_dis = sqrt(pow(goal_point_x-odom_x,2)+pow(goal_point_y-odom_y,2));

	//目標をキャンセルする条件を近→離1→近2→離3に変更する
	
	while(now2goal_dis > goal_margin && ros::ok()){
		scan_queue.callOne(ros::WallDuration(1));//重力の影響を受けた進行方向を決めて速度を送る
		if(reverse_flag){
			odom_queue.callOne(ros::WallDuration(1));//自分のオドメトリ取得
			scan_queue.callOne(ros::WallDuration(1));
			reverse_flag = false;
		}
		odom_queue.callOne(ros::WallDuration(1));//自分のオドメトリ取得
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
/*
		if(pre_now2goal_dis - now2goal_dis < 0){
			sum_diff += pre_now2goal_dis - now2goal_dis;
			if(sum_diff < cancel_diff){
				std::cout << "距離が遠くなったためbreak" << std::endl;
				std::cout << "目標への移動不可" << std::endl;
				return;
			}
		}
		else{
			sum_diff = 0;
		}
*/
	}

	pre_goal_point_x = goal_point_x;
	pre_goal_point_y = goal_point_y;

	std::cout << "目標へ移動終了" << std::endl;
}

void odom_callback(const geometry_msgs::Point::ConstPtr& odom_msg){
	odom_x = odom_msg -> x;
	odom_y = odom_msg -> y;
	yaw = odom_msg -> z;
}

//現在位置に対して制限範囲の中で最も近い領域を探す関数////////////////////////////////////////////////////////////

void choose_goal_frontier(std::vector<float> fro_x, std::vector<float> fro_y, int fro_num){

//ロボットの現在座標を取得//////////////////////////////////////////////////////////////////////////////////////
	float ro_x_map;//ロボットの現在のx座標
	float ro_y_map;//ロボットの現在のy座標
	std::vector<float> fro_x_tmp = fro_x;
	std::vector<float> fro_y_tmp = fro_y;
	std::vector<float> length; //距離格納用の配列
	float dot_max;//計算した内積の最大値
	float dot_tmp;//内積を一時保存
	float length_max;//計算した距離の最大値
	std::vector<float> dot;//計算した内積を保存
	int fro_num_tmp = fro_num;
	int point_num;//制限範囲のフィルタをかけた後の個数
	std::vector<int> i_list;//制限範囲内にあったiの番号を保存
	float EVA;//評価式の計算結果
	float EVA_max;//評価式の最大値
	int goal_num;//目標地点の座標番号
	bool first_calc;
	float x_tmp;
	float y_tmp;
	float dis_tmp;
	int i;

	
	//選択の際にpre_goal_pointとの距離の差を計算して、近過ぎたら別の候補にいく
	//そもそも計算時にスキップする
	float dist_threshold = 1.0;//前回のゴールからこの距離以下の目標は取らないように
	float pregoal_to_point;
	//前回の目標座標  pre_goal_point_x,pre_goal_point_y


	std::cout << "start:far_frontier"  << std::endl;
	
	if(fro_num_tmp == 0){
		std::cout << "未探査座標が無いからskipしたよ"  << std::endl;
		stop = true;
		goto skip;
	}

	std::cout << "start:ロボットの現在座標を取得" << std::endl;

	odom_queue.callOne(ros::WallDuration(1));

	ro_x_map = odom_x;
	ro_y_map = odom_y;

	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;
	std::cout << "end  :ロボットの現在座標を取得" << std::endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//現在位置から各目標に対してのベクトルを計算し、前回の移動と近い目標を探す///////////////////////////////////////////////
//先に距離でフィルタをかけてから各計算をする
//現在位置//ro_x_map,ro_y_map//
//目標座標のxy//fro_x_tmp,fro_y_tmp//
//ベクトル格納用の配列xy//vector_x,vector_y//
//目標座標群の個数//fro_num_tmp//
//ベクトルの大きさの最大値//vector_max//
//距離格納用の配列//length//
//制限範囲のフィルタをかけた後の個数//point_num//
//過去のベクトル//pre_vector_x;pre_vector_y;
//過去のベクトルと目標へのベクトルの内積用配列//dot//
//制限範囲内にあったiを保存//i_list//fro_x_tmp[i]を保存している

	std::cout << "start:制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << std::endl;

	point_num = 0;
	first_calc = true;


	for(i=0;i<fro_num_tmp;i++){
		pregoal_to_point = sqrt(pow(fro_x_tmp[i]-pre_goal_point_x,2)+pow(fro_y_tmp[i]-pre_goal_point_y,2));
		if(pregoal_to_point > dist_threshold){
			x_tmp = fro_x_tmp[i] - ro_x_map;
			y_tmp = fro_y_tmp[i] - ro_y_map;
			dis_tmp = sqrt(pow(x_tmp, 2) + pow(y_tmp,2));
			dot_tmp = (x_tmp*pre_vector_x + y_tmp*pre_vector_y)/dis_tmp;
			dot.push_back(dot_tmp);
			length.push_back(dis_tmp);
			i_list.push_back(i);

			point_num++;

			if(first_calc){
				dot_max = std::abs(dot_tmp);
				length_max = dis_tmp;
				first_calc = false;
			}
			if(std::abs(dot_tmp)>dot_max){
				dot_max = std::abs(dot_tmp);
			}
			if(dis_tmp>length_max){
				length_max = dis_tmp;
			}
		}
	}

	if(point_num == 0){
		std::cout << "未探査座標が無いからskipしたよ"  << std::endl;
		stop = true;
		goto skip;
	}

	first_calc = true;

	for(i=0;i<point_num;i++){
		
		if(first_cycle){
			EVA = (-(length[i]/length_max));
		}
		else{
			EVA = ((dot[i]/dot_max)-2.0*(length[i]/length_max));
		}

		if(first_calc){
			EVA_max = EVA;
			goal_num = i_list[i];
			first_calc = false;
		}
		if(EVA>EVA_max){
			EVA_max = EVA;
			goal_num = i_list[i];
		}
	}

	first_cycle = false;

	
	std::cout << "目標座標 (" << fro_x_tmp[goal_num] << "," << fro_y_tmp[goal_num] <<  ")" <<std::endl;



	std::cout << "end  :制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << std::endl;


	//std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;
	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;

////////////////////////*******//ここで経路作成関数に目標を渡す///******////////////////////////////////
	//目標が決まるたびにLEDの色を変える
	if(led2.value < 3){
		led2.value++;
	}
	else{
		led2.value = 1;
	}
	led2_pub.publish(led2);


	VFH_navigation(fro_x_tmp[goal_num], fro_y_tmp[goal_num]);

	odom_queue.callOne(ros::WallDuration(1));
	pre_vector_x = cos(yaw);
	pre_vector_y = sin(yaw);

skip:

////////////////////////////////////////////////////////////////////////////////////////////////////
	
	std::cout << "end  :far_frontier" << std::endl;
}

//フロンティアを検索する関数//////////////////////////////////////////////////////////////////////////////////////////
void frontier_search(const nav_msgs::OccupancyGrid::ConstPtr& msg){

//地図データを配列に格納////////////////////////////////////////////////////////////////////////////////////////
	const nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	//const std::vector<int8_t> data = msg->data;//地図の値を取得
	const int x = info.width;//地図の横サイズ
	const int y = info.height;//地図の縦サイズ
	int fro_num;
	std::vector<float> fro_x;//見つけたフロンティアのx座標
	std::vector<float> fro_y;//見つけたフロンティアのy座標

	//newでメモリを確保する

	//配列new用ポインタの宣言
        int8_t **map_array;
        int **point;
        int **frontier_flag;

        //int8_t map_array[x][y];//地図を行列に格納
        map_array = new int8_t*[x];
        for(int p=0;p<x;p++){
                map_array[p] = new int8_t[y];
        }

        //int frontier_flag[x][y];//探査済みと未探査の境界を判定するフラグ
        frontier_flag = new int*[x];
        for(int p=0;p<x;p++){
                frontier_flag[p] = new int[y];
        }

        //int point[x][y];//未探査領域の近くに障害物があるか判定する用
        point = new int*[x];
        for(int p=0;p<x;p++){
                point[p] = new int[y];
        }


	int i,j;//for文
	int k = 0;//for文


	std::cout << "start:frontier_search" << std::endl;

	std::cout << "start:地図データを配列に格納" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<x;j++){
      			map_array[j][i] = msg->data[k];
			if(map_array[j][i]!=0 && map_array[j][i]!=100 && map_array[j][i]!=-1){
					std::cout << "exception:" << map_array[j][i] << std::endl;		
			}
			frontier_flag[j][i] = 0;
			point[j][i] = 0;
      			k++;
    		}
  	}

	std::cout << "end  :地図データを配列に格納" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//横方向で未探査と探査済の境界を探す//////////////////////////////////////////////////////////////////////////////

	std::cout << "start:横方向で境界を検索" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<(x-1);j++){
      			if(map_array[j][i] == 0 && map_array[j+1][i] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j+1][i] == 0){
				frontier_flag[j+1][i] = 1;	
			}
    		}
  	}
	std::cout << "end  :横方向で境界を検索" << " ここまで" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
//縦方向で未探査と探査済の境界を探す/////////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界を検索" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<(y-1);i++){
      			if(map_array[j][i] == 0 && map_array[j][i+1] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j][i+1] == 0){
				frontier_flag[j][i+1] = 1;	
			}
    		}
  	}

	std::cout << "end  :縦方向で境界を検索" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
//横方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////
	const float m_per_cell = info.resolution;//[m/cell]
	const float search_len = 0.4; //障害物を検索する正方形の一辺の長さ[m](都合上10倍して自然数の偶数になる値のみで計算時は+m_per_cell[m]される)
	const float robot_diameter = 0.4; //ロボットの直径は0.4[m]
	const int search_len_cell = search_len / m_per_cell;//セル換算した正方形の一辺の長さ
	const int robot_cellsize = robot_diameter / m_per_cell;//セル換算したロボットサイズ
	const float low_left_x = info.origin.position.x;//地図の左下のx座標
	const float low_left_y = info.origin.position.y;//地図の左下のy座標
	const int half_sq = search_len_cell / 2;//正方形の一辺の半分の長さ(セル)


	int frontier_sum;//フラグが続いているかの判定用
	float frontier_center;//フロンティア境界の中点
	int flo2int;
	std::vector<int> pre_frox;//未探査領域のx座標を一時保存
	std::vector<int> pre_froy;//未探査領域のy座標を一時保存
	int pre_fronum = 0;//未探査領域の個数を一時保存
	fro_num = 0;//フロンティアの個数を初期化


	std::cout << "start:横方向で境界が連続している場所を検索" << std::endl;
	int v;
	int continuity = 0;
	int start_k = 0;
	int end_k = 0;
	const int search_width = 3;//フラグの連続を検索するときの線の太さ(奇数)
	const int search_margin = search_width/2;

	for(i=search_margin;i<(y-search_margin);i=i+search_width){
		k = 0;
		while(k < x && ros::ok()){
			for(v=-search_margin;v<=search_margin;v++){
				frontier_sum += frontier_flag[k][i+v];
			}
			if(frontier_sum > 0){
				start_k = k;
				while(frontier_sum > 0 && ros::ok()){
					frontier_sum = 0;
					continuity++;
					if(k < x){
						k++;
						for(v=-search_margin;v<=search_margin;v++){
							frontier_sum += frontier_flag[k][i+v];
						}
					}
					else{	
						k++;
						break;
					}
					//std::cout << k << "," << x << std::endl;
				}
				end_k = k-1;
				if(continuity >= robot_cellsize){
					frontier_center = (start_k + end_k)/2;
					flo2int = frontier_center;
					point[flo2int][i] = 1;
					pre_frox.push_back(flo2int);
					pre_froy.push_back(i);
					pre_fronum++;
				}
			}
			else{			
				k++;
			}
		}
	}
	
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;


	for(j=search_margin;j<(x-search_margin);j=j+search_width){
		k = 0;
		while(k < y && ros::ok()){
			for(v=-search_margin;v<=search_margin;v++){
				frontier_sum += frontier_flag[j+v][k];
			}
			if(frontier_sum > 0){
				start_k = k;
				while(frontier_sum > 0 && ros::ok()){
					frontier_sum = 0;
					continuity++;
					if(k < y){
						k++;
						for(v=-search_margin;v<=search_margin;v++){
							frontier_sum += frontier_flag[j+v][k];
						}
					}
					else{	
						k++;
						break;
					}
				}
				end_k = k-1;
				if(continuity >= robot_cellsize){
					frontier_center = (start_k + end_k)/2;
					flo2int = frontier_center;
					point[j][flo2int] = 1;
					pre_frox.push_back(j);
					pre_froy.push_back(flo2int);
					pre_fronum++;
				}
			}
			else{			
				k++;
			}
		}
	}
	
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//未探査領域配列に障害物情報を追加

	std::cout << "start:障害物情報を追加" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<y;i++){
      			if(map_array[j][i] == 100){
	       			point[j][i] = 100;
			}
    		}
  	}
	std::cout << "end  :障害物情報を追加" << std::endl;
//////////////////////////////////////////////////////////////////////////////////////////

//未探査領域の周囲の障害物情報を検索し最終的な未探査領域の座標を決定
	
//search_len_cell;//セル換算した正方形の一辺の長さ
//half_sq = search_len_cell / 2;//正方形の一辺の半分の長さ(セル)
//x;//地図の横サイズ
//y;//地図の縦サイズ
//pre_frox[];//未探査領域のx座標を一時保存
//pre_froy[];//未探査領域のy座標を一時保存
//pre_fronum = 0;//未探査領域の個数を一時保存
//point[x][y];//未探査領域と障害物情報がある行列
//frontier_sum;//未探査領域の中身の和

	int half_leftx;//四角形の左半分の長さ
	int half_rightx;//四角形の右半分の長さ
	int half_topy;//四角形の上半分の長さ
	int half_bottomy;//四角形の下半分の長さ
	
	std::cout << "start:未探査領域周辺の障害物を検索" << std::endl;

	for(k=0;k<pre_fronum;k++){
		
		if(pre_frox[k]-half_sq < 0){
			half_leftx = pre_frox[k];
		}
		else{
			half_leftx = half_sq;
		}


		if(pre_frox[k]+half_sq > (x-1)){
			half_rightx = (x-1)-pre_frox[k];
		}
		else{
			half_rightx = half_sq;
		}


		if(pre_froy[k]-half_sq < 0){
			half_topy = pre_froy[k];
		}
		else{
			half_topy = half_sq;
		}


		if(pre_froy[k]+half_sq > (y-1)){
			half_bottomy = (y-1)-pre_froy[k];
		}
		else{
			half_bottomy = half_sq;
		}
		
		frontier_sum = 0;
		
		for(i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++){
			for(j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++){
				frontier_sum+=point[j][i];
			}
		}

		if(frontier_sum>100){
			point[pre_frox[k]][pre_froy[k]] = 0;
		}
	}


	//最終的な未探査領域を配列に格納
	for(j=0;j<x;j++){
    		for(i=0;i<y;i++){
      			if(point[j][i] == 1){
	       			fro_x.push_back(m_per_cell * j + low_left_x);
				fro_y.push_back(m_per_cell * i + low_left_y);
				fro_num++;
			}
    		}
  	}

	std::cout << "障害物判定前の未探査領域の個数 " << pre_fronum << std::endl;
	std::cout << "障害物判定後の未探査領域の個数 " << fro_num << std::endl;

	std::cout << "end  :未探査領域周辺の障害物を検索" << std::endl;

	if(fro_num == 0){
		stop = true;
		return;
	}

	choose_goal_frontier(fro_x, fro_y, fro_num);

	//newで確保したメモリを開放する
	for(int p=0;p<x;p++){
                delete[] map_array[p];
        }
        delete[] map_array;

        for(int p=0;p<x;p++){
                delete[] point[p];
        }
        delete[] point;

        for(int p=0;p<x;p++){
                delete[] frontier_flag[p];
        }
        delete[] frontier_flag;

	std::cout << "end  :frontier_search" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


//メイン関数////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){

  	ros::init(argc, argv, "new_vector_explore_program");
  	ros::NodeHandle f;

	scan_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,VFH_gravity,ros::VoidPtr(),&scan_queue);
	scan_sub = f.subscribe(scan_option);

	bumper_option = ros::SubscribeOptions::create<kobuki_msgs::BumperEvent>("/bumper_info",1,bumper,ros::VoidPtr(),&bumper_queue);
	bumper_sub = f.subscribe(bumper_option);

	map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,frontier_search,ros::VoidPtr(),&map_queue);
	map_sub = f.subscribe(map_option);

	wall_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,rotation_based_map,ros::VoidPtr(),&wall_queue);
	wall_sub = f.subscribe(wall_option);

	odom_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_support",1,odom_callback,ros::VoidPtr(),&odom_queue);
	odom_sub = f.subscribe(odom_option);

	scan_branch_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,scan_branch_callback,ros::VoidPtr(),&scan_branch_queue);	
	scan_branch_sub = f.subscribe(scan_branch_option);

	scan_rotate_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,scan_rotate_callback,ros::VoidPtr(),&scan_rotate_queue);
	scan_rotate_sub = f.subscribe(scan_rotate_option);

	vel_pub = f.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	marker_pub = f.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	which_pub = f.advertise<geometry_msgs::Point>("/which_based", 1);

	std::cout << "start:探査プログラム" << std::endl;

//pre_vecorの計算をyaw使ってここでやる
	odom_queue.callOne(ros::WallDuration(1));
	pre_vector_x = cos(yaw);
	pre_vector_y = sin(yaw);


	led1_pub = f.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
	led2_pub = f.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);
	kobuki_msgs::Led led1;
	led1.value = 3;
	led2.value = 0;
	
	sleep(1);
	
	led1_pub.publish(led1);

	while(!stop && ros::ok()){
		map_queue.callOne(ros::WallDuration(1));
	}
	
	led1.value = 0;
	led1_pub.publish(led1);

	std::cout << "end:探査プログラム" << std::endl;
	return 0;
}
