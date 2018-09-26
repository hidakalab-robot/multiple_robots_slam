#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <kobuki_msgs/BumperEvent.h>
//#include <std_msgs/Bool.h>

//グローバル変数////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher bumper_pub;
ros::Subscriber bumper_sub;
ros::CallbackQueue bumper_queue;

ros::SubscribeOptions bumper_option;

//std_msgs::Bool hit;
kobuki_msgs::BumperEvent bumper_info;


void Bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& Bumper_msg){
	int state = Bumper_msg -> state;
	if(state == 1){
		std::cout << "hit" << std::endl;
		bumper_info.bumper = Bumper_msg -> bumper;
		bumper_info.state = Bumper_msg -> state;
	}
	else{
		bumper_info.state = Bumper_msg -> state;
		std::cout << "not_hit" << std::endl;
	}
}

//メイン関数////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){

  	ros::init(argc, argv, "bumper_hit");
  	ros::NodeHandle b;

	bumper_option = ros::SubscribeOptions::create<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper",1,Bumper_callback,ros::VoidPtr(),&bumper_queue);
	bumper_sub = b.subscribe(bumper_option);

	bumper_pub = b.advertise<kobuki_msgs::BumperEvent>("/bumper_info", 1);

	bumper_info.bumper = 0;
	bumper_info.state = 0;

	while(ros::ok()){
		bumper_queue.callOne(ros::WallDuration(0.1));
		bumper_pub.publish(bumper_info);
	}
	return 0;
}
