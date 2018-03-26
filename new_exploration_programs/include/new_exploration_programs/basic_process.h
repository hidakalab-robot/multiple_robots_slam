#ifndef INCLUDE_GARDE_BP
#define INCLUDE_GARDE_BP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/Led.h>
#include <std_msgs/Int8.h>

class BasicProcess
{
private:
	ros::NodeHandle bp;
	ros::NodeHandle bp1;
	ros::NodeHandle bp2;
	ros::NodeHandle bp3;
	ros::NodeHandle bp4;
	ros::NodeHandle bp5;
	/*callbackqueue*/
	ros::CallbackQueue bumper_queue;
	ros::CallbackQueue odom_queue;
	ros::CallbackQueue loop_queue;
	ros::CallbackQueue map_queue;
	ros::CallbackQueue scan_queue;
	/*subscribeoptions*/
	ros::SubscribeOptions bumper_option;
	ros::SubscribeOptions odom_option;
	ros::SubscribeOptions loop_option;
	ros::SubscribeOptions map_option;
	ros::SubscribeOptions scan_option;
	/*subscriber*/
	ros::Subscriber bumper_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber loop_sub;
	ros::Subscriber map_sub;
	ros::Subscriber scan_sub;
	/*publisher*/
	ros::Publisher which_pub;//自己位置とどちらの探査方法かを出力
	ros::Publisher marker_pub;//rviz上に移動目標地点などを示す
	ros::Publisher vel_pub;//速度を入力する
	ros::Publisher led1_pub;
	ros::Publisher led2_pub;

	/*通常変数*/
	int which_bumper;
	bool bumper_hit;
	float odom_x;//オドメトリx
	float odom_y;//オドメトリy
	double yaw;//ヨー角
	float angle_min;
	float angle_max;
	float angle_increment;
	std::vector<float> ranges;
	nav_msgs::MapMetaData map_info;
	std::vector<int8_t> map_data;
	float rotate_vel;//数値0.5
	float scan_threshold;//数値0.8
	float which_exp;

	/*static変数*/

	static float omega;
	static geometry_msgs::Twist vel;
	static std::vector<float> odom_log_x;//オドメトリxの履歴を保存
	static std::vector<float> odom_log_y;//オドメトリyの履歴を保存
	static int loop_count;
	static float pre_theta;

	static float forward_vel;//数値0.2


	static kobuki_msgs::Led led1;
	static kobuki_msgs::Led led2;
public:
	/*コンストラクタ*/
	BasicProcess();
	/*デストラクタ*/
	~BasicProcess(){};
	/*callback_function*/
	void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg);
	void odom_callback(const geometry_msgs::Point::ConstPtr& odom_msg);
	//void loop_callback(const geometry_msgs::Point::ConstPtr& loop_msg);//loop-closureを数える
	//loop_Testの方もpoint-intに変更しておく
	void loop_callback(const std_msgs::Int8::ConstPtr& loop_msg);
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

	/*get_function*/
	void get_bumper(bool *hit, int *which);
	void get_loop(int *loop);
	void get_odom(float *x, float *y, double *z);//odomの中身を受け渡す用
	void get_map(nav_msgs::MapMetaData *info, std::vector<int8_t> *data);
	void get_scan(std::vector<float> *r, float *min, float *max, float *inc);
	void get_odomlog(std::vector<float> *logx, std::vector<float> *logy);
	void get_vel(float *x, float *z);
	void get_led1(int *val);
	void get_led2(int *val);

	void get_rotatevel(float *r);
	void get_scanthreshold(float *s);
	void get_pretheta(float *p);
	void get_omega(float *o);

	/*set_function*/
	void set_vel(float x, float z);
	void set_led1(int val);
	void set_led2(int val);
	void set_whichexp(float w);

	/*pub_function*/
	void pub_vel(void);
	void pub_led1(void);
	void pub_led2(void);

	/*sonota_function*/
	void odom_marking(float x, float y);
	void display_goal_angle(float x, float y);
	void display_gravity(float x, float y);
	void vel_curve_VFH(float theta,float t,float v=forward_vel);////frontierでもodomlogを取るようにする
	void approx(std::vector<float> &scan);
	void one_rotation(void);
};

#endif
