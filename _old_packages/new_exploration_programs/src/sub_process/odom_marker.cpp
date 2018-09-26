#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

ros::CallbackQueue which_queue;
ros::Subscriber which_sub;
ros::SubscribeOptions which_option;
ros::Publisher marker_pub;

geometry_msgs::Point odom_s;
visualization_msgs::Marker marker3;

void odom_marking(const geometry_msgs::Point::ConstPtr& which_msg){
	odom_s.x = which_msg -> x;
	odom_s.y = which_msg -> y;
	odom_s.z = which_msg -> z;

	uint32_t list = visualization_msgs::Marker::LINE_STRIP;
	geometry_msgs::Point marking_point;
	std_msgs::ColorRGBA marking_color;
	marker3.header.frame_id = "map";
	marker3.header.stamp = ros::Time::now();
	marker3.ns = "basic_shapes";
	marker3.id = 2;
	marker3.type = list;
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.lifetime = ros::Duration(0);
	marker3.pose.orientation.w = 1.0;
	marker3.scale.x = 0.1;
	
	marking_point.x = odom_s.x;
	marking_point.y = odom_s.y;
	marking_point.z = 0.0;

	if(odom_s.z < 1){//センサベースです
		marking_color.b = 0.0f;
		marking_color.a = 1.0;
		marking_color.r = 1.0f;
		marking_color.g = 1.0f;
	}
	else{//フロンティアベースです
		marking_color.b = 1.0f;
		marking_color.a = 1.0;
		marking_color.r = 0.0f;
		marking_color.g = 0.0f;
	}
	marker3.points.push_back(marking_point);
	marker3.colors.push_back(marking_color);
	
	std::cout << "odom_s(" << odom_s.x << "," << odom_s.y << "," << odom_s.z << ")" << std::endl;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "odom_marker");
	ros::NodeHandle oma;

	marker_pub = oma.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	which_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/which_based",1,odom_marking,ros::VoidPtr(),&which_queue);
	which_sub = oma.subscribe(which_option);

	odom_s.x = 0;
	odom_s.y = 0;
	odom_s.z = 0;

	while(ros::ok()){
		which_queue.callOne(ros::WallDuration(10));
		marker_pub.publish(marker3);
	}

	return 0;
}
