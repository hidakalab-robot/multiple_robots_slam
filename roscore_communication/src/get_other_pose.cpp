#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher p_pub;
std::string my_mergemap_frame;

void pose_frame_editer(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped pub_pose;
  pub_pose.pose = msg -> pose;

  pub_pose.header.stamp = ros::Time::now();
  pub_pose.header.frame_id = my_mergemap_frame;

  p_pub.publish(pub_pose);
}

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
  nh_priv.getParam("my_mergemap_frame", my_mergemap_frame);

  p_sub = nh.subscribe(sub_topic,1,pose_frame_editer);
  p_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);

  ros::spin();

  return 0;

}
