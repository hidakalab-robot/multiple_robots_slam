#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>

ros::CallbackQueue move_queue;
ros::Subscriber move_sub;
ros::SubscribeOptions move_option;

ros::CallbackQueue loop_queue;
ros::Subscriber loop_sub;
ros::SubscribeOptions loop_option;

float pre_trans_x = 0;
float pre_trans_y = 0;

float pre_odom_x = 0;
float pre_odom_y = 0;

float move_distance = 0;


float correction = 0;

void loop_callback(const geometry_msgs::Point::ConstPtr& loop_data){
	float trans_x = loop_data -> x;
	float trans_y = loop_data -> y;

	if(trans_x != pre_trans_x || trans_y != pre_trans_y){
		correction = sqrt(pow(trans_x-pre_trans_x,2)+pow(trans_y-pre_trans_y,2));
	}
	else{
		correction = 0;
	}
	pre_trans_x = trans_x;
	pre_trans_y = trans_y;
}


void calc_callback(const geometry_msgs::Point::ConstPtr& odom_data){
	float odom_x = odom_data -> x;
	float odom_y = odom_data -> y;

	std::cout << "x:" << odom_x << "," << "y:" << odom_y << std::endl;
	move_distance += sqrt(pow(odom_x-pre_odom_x,2)+pow(odom_y-pre_odom_y,2));

	pre_odom_x = odom_x;
	pre_odom_y = odom_y;	

	/*loopclosureの補正を引く*/

	//loop_queue.callOne(ros::WallDuration(1));

	move_distance -= correction;

}

int main(int argc, char** argv){

	ros::init(argc, argv, "calc_move_distance");
	ros::NodeHandle cmd;

	move_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_support",1,calc_callback,ros::VoidPtr(),&move_queue);
	move_sub = cmd.subscribe(move_option);

	loop_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom2map_support",1,loop_callback,ros::VoidPtr(),&loop_queue);
	loop_sub = cmd.subscribe(loop_option);

	while(ros::ok()){
		move_queue.callOne(ros::WallDuration(1));
		std::cout << "現在の総移動距離 : " << move_distance << "[m]" << std::endl;
	}

	return 0;
}
