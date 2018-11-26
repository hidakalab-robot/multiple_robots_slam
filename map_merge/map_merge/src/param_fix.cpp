#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_msgs/TFMessage.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "get_other_pose");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  ros::Subscriber p_sub;

  std::string sub_topic;
  std::string pub_topic;

  nh_priv.getParam("sub_topic", sub_topic);
  nh_priv.getParam("pub_topic", pub_topic);
  //nh_priv.getParam("my_mergemap_frame", my_mergemap_frame);

  //p_sub = nh.subscribe(sub_topic,1,pose_frame_editer);
  //p_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);

  ros::spin();

  return 0;

}