#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>

ros::CallbackQueue rsn_map_queue;
ros::Subscriber rsn_map_sub;
ros::SubscribeOptions rsn_map_option;
nav_msgs::OccupancyGrid rsn_map;
ros::Publisher rsn_map_pub;

void rsn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& rsn_map_msg){
	rsn_map.header = rsn_map_msg -> header;
	rsn_map.info = rsn_map_msg -> info;
	rsn_map.data = rsn_map_msg ->data;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "share_topics");
	ros::NodeHandle rsn;

	rsn_map_pub = rsn.advertise<nav_msgs::OccupancyGrid>("/ros1robot/shared_map", 1);//ロボットによって書き換え

	rsn_map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/ros1robot/map",1,rsn_map_callback,ros::VoidPtr(),&rsn_map_queue);//ロボットによって書き換え
	rsn_map_sub = rsn.subscribe(rsn_map_option);

	while(ros::ok()){
		rsn_map_queue.callOne(ros::WallDuration(0.05));
		rsn_map_pub.publish(rsn_map);
	}

	return 0;
}
