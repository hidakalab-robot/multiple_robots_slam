#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>

ros::CallbackQueue move_queue;
ros::Subscriber move_sub;
ros::SubscribeOptions move_option;

float pre_odom_x = 0;
float pre_odom_y = 0;

float move_distance = 0;

void calc_callback(const geometry_msgs::Point::ConstPtr& odom_data){
	float odom_x = odom_data -> x;
	float odom_y = odom_data -> y;

	std::cout << "x:" << odom_x << "," << "y:" << odom_y << std::endl;
	move_distance += sqrt(pow(odom_x-pre_odom_x,2)+pow(odom_y-pre_odom_y,2));
	
	pre_odom_x = odom_x;
	pre_odom_y = odom_y;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "calc_move_distance");
	ros::NodeHandle cmd;

	move_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_support",1,calc_callback,ros::VoidPtr(),&move_queue);
	move_sub = cmd.subscribe(move_option);

	while(ros::ok()){
		move_queue.callOne(ros::WallDuration(1));
		std::cout << "現在の総移動距離 : " << move_distance << "[m]" << std::endl;
	}

	return 0;
}
