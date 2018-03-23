#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Point.h>
#include <kobuki_msgs/Led.h>
#include <nav_msgs/OccupancyGrid.h>

class SubProcess
{
private:


public:
	/*コンストラクタ*/
	SubProcess(/*引数*/);
	/*デストラクタ*/
	~SubProcess();
	float road_center_search(std::vector<float> &fixed_ranges,std::vector<float> &fixed_angle)
	void road_center_callback(const sensor_msgs::LaserScan::ConstPtr& road_msg)
};
