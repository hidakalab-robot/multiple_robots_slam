#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Point.h>
#include <kobuki_msgs/Led.h>
#include <nav_msgs/OccupancyGrid.h>

class BranchProcess
{
private:


public:
	/*コンストラクタ*/
	BranchProcess(/*引数*/);
	/*デストラクタ*/
	~BranchProcess();
	void Branch_search(std::vector<float> &fixed_ranges,std::vector<float> &fixed_angle,float max_angle)
	void Branch_area_callback(const sensor_msgs::LaserScan::ConstPtr& Branch_msg)
};
