#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <map_merging/PartnerRobot.h>

/*ロボットを見つけたかの判断を受け取る用*/
ros::CallbackQueue meet_queue;
ros::Subscriber meet_sub;
ros::SubscribeOptions meet_option;

map_merging::PartnerRobot partner;

/*自分側のデータ*/
ros::CallbackQueue my_map_queue;
ros::Subscriber my_map_sub;
ros::SubscribeOptions my_map_option;

ros::CallbackQueue my_odom_queue;
ros::Subscriber my_odom_sub;
ros::SubscribeOptions my_odom_option;

nav_msgs::OccupancyGrid my_map;
geometry_msgs::Point my_odom;

/*相手側のデータ*/
ros::CallbackQueue pa_map_queue;
ros::Subscriber pa_map_sub;
ros::SubscribeOptions pa_map_option;

ros::CallbackQueue pa_odom_queue;
ros::Subscriber pa_odom_sub;
ros::SubscribeOptions pa_odom_option;

nav_msgs::OccupancyGrid pa_map;
geometry_msgs::Point pa_odom;

/*合成した地図用*/
nav_msgs::OccupancyGrid merged_map;
ros::Publisher merged_map_pub;





double calc_rotation_angle(){
	double angle;
	angle = partner.angle - pa_odom.z + my_odom.z;
	return angle;
}

/*地図を合成するメインの関数*/
void marge_map(){
	my_map_queue.callOne(ros::WallDuration(0.05));
	my_odom_queue.callOne(ros::WallDuration(0.05));
	pa_map_queue.callOne(ros::WallDuration(0.05));
	pa_odom_queue.callOne(ros::WallDuration(0.05));

	double rotation_angle;//地図を回転する角度を入れる
	
	/*pa_mapをルート2倍以上にした正方形の一辺の長さを計算し、奇数だったら１足す
	int rotated_width;//回転した行列の一辺の長さ
	
	int rotated_map[rotated_width][rotated_width];//回転した地図
	*/	
	
	rotation_angle = calc_rotation_angle();//地図を回転する角度を計算

	map_rotation(rotation_angle);//地図を回転する;

	//merging();合成する
	
	

	//最後にmerged_mapに代入する感じで
}

/*相手のodomを取得*/
void get_pa_odom(const geometry_msgs::Point::ConstPtr& pa_odom_msg){
	pa_odom.x = pa_odom_msg -> x;
	pa_odom.y = pa_odom_msg -> y;
	pa_odom.z = pa_odom_msg -> z;
}

/*相手のmapを取得*/
void get_pa_map(const nav_msgs::OccupancyGrid::ConstPtr& pa_map_msg){
	pa_map.header = pa_map_msg -> header;
	pa_map.info = pa_map_msg -> info;
	pa_map.data = pa_map_msg ->data;
}

/*自分のodomを取得*/
void get_my_odom(const geometry_msgs::Point::ConstPtr& my_odom_msg){
	my_odom.x = my_odom_msg -> x;
	my_odom.y = my_odom_msg -> y;
	my_odom.z = my_odom_msg -> z;
}

/*自分のmapを取得*/
void get_my_map(const nav_msgs::OccupancyGrid::ConstPtr& my_map_msg){
	my_map.header = my_map_msg -> header;
	my_map.info = my_map_msg -> info;
	my_map.data = my_map_msg ->data;
}

/*相手ロボットとの距離情報などを取得*/
void meet_robot(const map_merging::PartnerRobot::ConstPtr& meet_msg){
	partner.meet = meet_msg -> meet;//相手のロボットが見つかったかbool
	partner.x = meet_msg -> x;//ロボット座標系での相手のロボットのx座標double[m]
	partner.y = meet_msg -> y;//ロボット座標系での相手のロボットのy座標double[m]
	partner.angle = meet_msg -> angle;//相手のロボットの角度double[rad]
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_merging");
	ros::NodeHandle mm;

	/*データ送受信初期設定*/
	meet_option = ros::SubscribeOptions::create<map_merging::PartnerRobot>("/meet_robot",1,meet_robot,ros::VoidPtr(),&meet_queue);//ロボットを見つけたか、ロボットの距離、ロボットの角度を取得
	meet_sub = mm.subscribe(meet_option);

	my_map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,get_my_map,ros::VoidPtr(),&my_map_queue);
	my_map_sub = mm.subscribe(my_map_option);

	my_odom_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_supprt",1,get_my_odom,ros::VoidPtr(),&my_odom_queue);
	my_odom_sub = mm.subscribe(my_odom_option);

	pa_map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/rosrobot/shared_map",1,get_pa_map,ros::VoidPtr(),&pa_map_queue);//トピック名はロボットによって書き換える
	pa_map_sub = mm.subscribe(pa_map_option);

	pa_odom_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/rosrobot/shared_odom",1,get_pa_odom,ros::VoidPtr(),&pa_odom_queue);//トピック名はロボットによって書き換える
	pa_odom_sub = mm.subscribe(pa_odom_option);

	merged_map_pub = mm.advertise<nav_msgs::OccupancyGrid>("/merged_map", 1);//合成した地図を出力するパブリッシャー


	while(ros::ok()){
		meet_queue.callOne(ros::WallDuration(0.05));
		if(partner.meet){
			marge_map();
			merged_map_pub.publish(merged_map);
		}
	}

	return 0;
}
