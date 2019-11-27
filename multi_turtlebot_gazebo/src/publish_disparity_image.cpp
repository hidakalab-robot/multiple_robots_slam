#include<ros/ros.h>
#include<stereo_msgs/DisparityImage.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "publish_disparity_image");

    ros::Publisher pub = ros::NodeHandle().advertise<sensor_msgs::Image>("disparity_image",1,true);
    ros::Subscriber sub = ros::NodeHandle().subscribe<stereo_msgs::DisparityImage>("disparity_data", 1, [&](const stereo_msgs::DisparityImage::ConstPtr& msg){pub.publish(msg->image);});

    ros::spin();
    return 0;
}