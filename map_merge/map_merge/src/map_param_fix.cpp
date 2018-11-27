#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Pose.h>

class MapParamFix
{
private:
  std::string mapTopic;
  std::string tfTopic;

  std::string odomFrame;

  nav_msgs::OccupancyGrid newMap;
  nav_msgs::OccupancyGrid oldMap;
  tf2_msgs::TFMessage newTf;
  tf2_msgs::TFMessage oldTf;

  geometry_msgs::Pose initMapOrigin;
  geometry_msgs::Pose initMapParam;
  double resolution;

  ros::NodeHandle init_param;
  ros::NodeHandle edit_param;

  ros::NodeHandle s1;
  ros::NodeHandle s2;

  ros::Subscriber sub1;
  ros::Subscriber sub2;

  bool inputM;
  bool inputT;

  
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);

public:
  ros::CallbackQueue queueM;
  ros::CallbackQueue queueT;


  MapParamFix();
  ~MapParamFix(){};

  void param_fix(void);

  void initialize(void);

  bool isInputM(void);
  bool isInputT(void);
  void resetFlag(void);

};

MapParamFix::MapParamFix()
:init_param("~"), edit_param("~")
{
  init_param.getParam("map_topic", mapTopic);
  init_param.getParam("tf_topic", tfTopic);
  init_param.getParam("odom_frame", odomFrame);

  init_param.getParam("resolution", resolution);

  init_param.getParam("init_origin_x", initMapOrigin.position.x);
  init_param.getParam("init_origin_y", initMapOrigin.position.y);

  init_param.getParam("init_param_x", initMapParam.position.x);
  init_param.getParam("init_param_y", initMapParam.position.y);

  s1.setCallbackQueue(&queueM);
  s2.setCallbackQueue(&queueT);

  sub1 = s1.subscribe(mapTopic,1,&MapParamFix::map_callback,this);
  sub2 = s2.subscribe(tfTopic,1,&MapParamFix::tf_callback,this);

  inputM = false;
  inputT = false;
}

void MapParamFix::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  oldMap = newMap;
  newMap = *msg;
  //std::cout << "input map" << "\n";
  inputM = true;
}

void MapParamFix::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  if(msg -> transforms[0].child_frame_id == odomFrame)
  {
    oldTf = newTf;
    newTf = *msg;
    inputT = true;
  }
}

void MapParamFix::param_fix(void)
{
  //paramを編集

  //originの差分を計算
  double diffX,diffY;

  diffX = newMap.info.origin.position.x - initMapOrigin.position.x;
  diffY = newMap.info.origin.position.y - initMapOrigin.position.y;

  double newParamX,newParamY;

  geometry_msgs::Pose newParam;

  newParam.position.x = initMapParam.position.x + (diffX / resolution * -1);
  newParam.position.y = initMapParam.position.y + (diffY / resolution * -1);

  edit_param.setParam("/robot2/grid_map_merge/init_pose_x",newParam.position.x);
  edit_param.setParam("/robot2/grid_map_merge/init_pose_y",newParam.position.y);

  if(diffX != 0 || diffY != 0)
  {
    std::cout << "newParam :\n" << newParam << "\n";
  }
}

void MapParamFix::initialize(void)
{
  initMapOrigin = newMap.info.origin;
  // /robot1/grid_map_merge/init_pose_x
  // /robot1/grid_map_merge/init_pose_y
  // /robot1/grid_map_merge/init_pose_yaw

  edit_param.getParam("/robot2/grid_map_merge/init_pose_x",initMapParam.position.x);
  edit_param.getParam("/robot2/grid_map_merge/init_pose_y",initMapParam.position.y);

  std::cout << "initMapOrigin :\n" << initMapOrigin << "\n";
  std::cout << "initMapParam :\n" << initMapParam << "\n";

  std::cout << "initialize complete" << "\n";
}

bool MapParamFix::isInputM(void)
{
  return inputM;
}

bool MapParamFix::isInputT(void)
{
  return inputT;
}

void MapParamFix::resetFlag(void)
{
  inputM = false;
  inputT = false;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "map_param_fix");

  MapParamFix mpf;

  // while(ros::ok())
  // {
  //   mpf.queueM.callOne(ros::WallDuration(1));
  //   //mpf.queueT.callOne(ros::WallDuration(1));
  //   if(mpf.isInputM())
  //   {
  //     mpf.initialize();
  //     mpf.resetFlag();
  //     break;
  //   }
  //   std::cout << "not initial map" << "\n";
  //   mpf.resetFlag();
  // }

  while(ros::ok())
  {
    mpf.queueM.callOne(ros::WallDuration(1));
    //mpf.queueT.callOne(ros::WallDuration(1));
    if(mpf.isInputM())
    {
      mpf.param_fix();
    }
    //std::cout << "not map" << "\n";
    mpf.resetFlag();
  }
  //ros::NodeHandle nh;
  //ros::NodeHandle nh_priv("~");

  //ros::Subscriber p_sub;

  //std::string sub_topic;
  //std::string pub_topic;

  //nh_priv.getParam("sub_topic", sub_topic);
  //nh_priv.getParam("pub_topic", pub_topic);
  //nh_priv.getParam("my_mergemap_frame", my_mergemap_frame);

  //p_sub = nh.subscribe(sub_topic,1,pose_frame_editer);
  //p_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);

  //ros::spin();

  return 0;

}