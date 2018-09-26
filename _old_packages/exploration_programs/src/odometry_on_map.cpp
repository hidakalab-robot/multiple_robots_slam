#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "odometry_on_map");
	ros::NodeHandle om;

	ros::Publisher oom_pub;
	oom_pub = om.advertise<geometry_msgs::Point>("/odom_on_map", 100);

	tf::StampedTransform transform;
	tf::TransformListener listener;
	geometry_msgs::Point odom_m;

	ros::Time now = ros::Time::now();

	while(ros::ok()){

		listener.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	
		odom_m.x = transform.getOrigin().x();
		odom_m.y = transform.getOrigin().y();
		odom_m.z = tf::getYaw(transform.getRotation());

		std::cout << "odom_m(" << odom_m.x << "," << odom_m.y << "," << odom_m.z << ")" << std::endl;

		oom_pub.publish(odom_m);
	}

	return 0;
}
