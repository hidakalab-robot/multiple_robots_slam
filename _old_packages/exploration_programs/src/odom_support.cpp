#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>

ros::CallbackQueue odom_queue;
ros::Subscriber odom_sub;
ros::SubscribeOptions odom_option;
geometry_msgs::Point odom_s;
ros::Publisher os_pub;

void odom_callback(const geometry_msgs::Point::ConstPtr& odom_msg){
	odom_s.x = odom_msg -> x;
	odom_s.y = odom_msg -> y;
	odom_s.z = odom_msg -> z;
	std::cout << "odom_s(" << odom_s.x << "," << odom_s.y << "," << odom_s.z << ")" << std::endl;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "odom_support");
	ros::NodeHandle os;

	os_pub = os.advertise<geometry_msgs::Point>("/odom_support", 1);


	odom_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_on_map",1,odom_callback,ros::VoidPtr(),&odom_queue);
	odom_sub = os.subscribe(odom_option);

	odom_s.x = 0;
	odom_s.y = 0;
	odom_s.z = 0;

	while(ros::ok()){
		odom_queue.callOne(ros::WallDuration(0.05));
		os_pub.publish(odom_s);
	}

	return 0;
}
