#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

/*自分側のデータ*/
ros::CallbackQueue my_map_queue;
ros::Subscriber my_map_sub;
ros::SubscribeOptions my_map_option;

ros::CallbackQueue my_odom_queue;
ros::Subscriber my_odom_sub;
ros::SubscribeOptions my_odom_option;

nav_msgs::OccupancyGrid my_map;

/*相手側のデータ*/
ros::CallbackQueue pa_map_queue;
ros::Subscriber pa_map_sub;
ros::SubscribeOptions pa_map_option;

ros::CallbackQueue pa_odom_queue;
ros::Subscriber pa_odom_sub;
ros::SubscribeOptions pa_odom_option;

nav_msgs::OccupancyGrid pa_map;




/*合成した地図用*/
ros::Publisher merged_map_pub;

void rsn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& rsn_map_msg){
	rsn_map.header = rsn_map_msg -> header;
	rsn_map.info = rsn_map_msg -> info;
	rsn_map.data = rsn_map_msg ->data;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "map_merging");
	ros::NodeHandle mm;

	merged_map_pub = mm.advertise<nav_msgs::OccupancyGrid>("/merged_map", 1);//合成した地図を出力するパブリッシャー

	rsn_map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/rosrobot/map",1,rsn_map_callback,ros::VoidPtr(),&rsn_map_queue);//ロボットによって書き換え
	rsn_map_sub = rsn.subscribe(rsn_map_option);

	while(ros::ok()){
		rsn_map_queue.callOne(ros::WallDuration(0.05));
		rsn_map_pub.publish(rsn_map);
	}

	return 0;
}
