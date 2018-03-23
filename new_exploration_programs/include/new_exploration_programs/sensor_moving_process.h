#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Point.h>
#include <kobuki_msgs/Led.h>
#include <nav_msgs/OccupancyGrid.h>

class SensorMovingProcess
{
private:


public:
	/*コンストラクタ*/
	SensorMovingProcess(/*引数*/);
	/*デストラクタ*/
	~SensorMovingProcess();
	float sen_VFH_move_angle(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan, std::vector<float> &angles)
	float sen_VFH_move_angle_g(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan, float gra_angle, std::vector<float> &angles)
	void sen_VFH_gravity(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	void sen_VFH4vel_publish_Branch()
	void sen_VFH_scan_callback(const sensor_msgs::LaserScan::ConstPtr& VFH_msg)

};
