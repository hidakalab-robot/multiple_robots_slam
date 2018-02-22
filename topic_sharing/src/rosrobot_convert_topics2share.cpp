#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>

ros::CallbackQueue cts_map_queue;
ros::Subscriber cts_map_sub;
ros::SubscribeOptions cts_map_option;
nav_msgs::OccupancyGrid cts_map;
ros::Publisher cts_map_pub;

void cts_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& cts_map_msg){
	cts_map.header = cts_map_msg -> header;
	cts_map.info = cts_map_msg -> info;
	cts_map.data = cts_map_msg ->data;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "convert_topics2share");
	ros::NodeHandle cts;

	cts_map_pub = cts.advertise<nav_msgs::OccupancyGrid>("/rosrobot/map", 1);//ロボットによって書き換え

	cts_map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,cts_map_callback,ros::VoidPtr(),&cts_map_queue);
	cts_map_sub = cts.subscribe(cts_map_option);

	while(ros::ok()){
		cts_map_queue.callOne(ros::WallDuration(0.05));
		cts_map_pub.publish(cts_map);
	}

	return 0;
}
