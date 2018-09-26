#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class CheckYaw
{
private:
/*サブスクライバ用*/
  ros::NodeHandle sO;
  ros::Subscriber subO;

/*パブリッシャ用*/
  ros::NodeHandle pV;
  ros::Publisher pubV;

  //計算したyaw角を格納する変数
  double yaw;

  //速度命令のメッセージ定義
  geometry_msgs::Twist vw;

public:
  //サブスクライバに必要なやつ
  ros::CallbackQueue queueO;

  /*コンストラクタとデストラクタ*/
  CheckYaw();
	~CheckYaw(){};

  //コールバック用の関数
  void OdomToYaw(const nav_msgs::Odometry::ConstPtr& sOMsg);
  //速度を送る関数
  void pubVelocity(void);

};

CheckYaw::CheckYaw()
{
/*サブスクライバの設定*/
  sO.setCallbackQueue(&queueO);
  subO = sO.subscribe("/odom",1,&CheckYaw::OdomToYaw,this);

/*速度を送るトピックを設定*/
  pubV = pV.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

/*速度初期化*/
  vw.linear.x = 0;
  vw.linear.y = 0;
  vw.linear.z = 0;
  vw.angular.x = 0;
  vw.angular.y = 0;
  vw.angular.z = 0.5;

}

void CheckYaw::OdomToYaw(const nav_msgs::Odometry::ConstPtr& sOMsg)
{
  //クオータニオンのyaw角の部分を取り出す
  double yawMsg = sOMsg -> pose.pose.orientation.z;

  //クオータニオン→オイラー角への変換
  yaw = 2*asin(yawMsg);

  std::cout << "yaw << " << yaw << " [rad] << " << yaw*180/M_PI << "[deg]" << '\n';
  std::cout << "90[deg]との差 << " << yaw - M_PI/2 << "[rad] << " << (yaw - M_PI/2)*180/M_PI << "[deg]" << '\n' << '\n';
}

void CheckYaw::pubVelocity(void)
{
  //オドメトリが90度より小さければ速度を送る
  if(yaw < M_PI/2)
  {
    pubV.publish(vw);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "check_yaw");

  CheckYaw cy;

  while(ros::ok())
  {
    //オドメトリをサブスクライブしてコールバック関数を呼び出す
    cy.queueO.callOne(ros::WallDuration(1));//数値はトピックの更新を待つ最大時間[s]
    //速度を送る関数
    cy.pubVelocity();
  }

  return 0;
}
