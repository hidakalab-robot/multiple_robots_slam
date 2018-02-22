#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>

ros::CallbackQueue tf_queue;
ros::Subscriber tf_sub;
ros::SubscribeOptions tf_option;

float pre_loop_x = 0;
float pre_loop_y = 0;
int loop_count = 0;
float sum_trans = 0;

void tf_callback(const geometry_msgs::Point::ConstPtr& tf_data){
	float trans_x = tf_data -> x;
	float trans_y = tf_data -> y;
	float trans_threshold = 1.0;

	if(trans_x != pre_loop_x || trans_y != pre_loop_y){
		std::cout << "x:" << trans_x << "," << "y:" << trans_y << std::endl;
		sum_trans += sqrt(pow(trans_x-pre_loop_x,2)+pow(trans_y-pre_loop_y,2));
		if(sum_trans > trans_threshold){
			loop_count++;
			std::cout << "ループクロージング" << loop_count << "回目" << std::endl;
			sum_trans = 0;
		}
		pre_loop_x = trans_x;
		pre_loop_y = trans_y;
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "loop_test");
	ros::NodeHandle tos;

	tf_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom2map_support",1,tf_callback,ros::VoidPtr(),&tf_queue);
	tf_sub = tos.subscribe(tf_option);

	while(ros::ok()){
		tf_queue.callOne(ros::WallDuration(1));
	}

	return 0;
}
