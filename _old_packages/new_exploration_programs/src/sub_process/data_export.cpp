#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>

ros::CallbackQueue map_data_queue;
ros::CallbackQueue odom_data_queue;

ros::SubscribeOptions map_data_option;
ros::SubscribeOptions odom_data_option;

ros::Subscriber map_data_sub;
ros::Subscriber odom_data_sub;

double odom_x;
double odom_y;
double explored_cell;

ros::Time start;
ros::Time process;


void export_data(){
	std::ofstream ofs("export_data.csv",std::ios::app);
	//ofs << odom_x << "," << odom_y << "," << explored_cell << std::endl;
	ros::Duration sec = process - start;
	double time = sec.toSec();;
	ofs << time << "," << explored_cell << std::endl;
}

void map_data(const nav_msgs::OccupancyGrid::ConstPtr& map_data){
	nav_msgs::MapMetaData info = map_data->info;
	//std::vector<int8_t> data = map_data->data;
	int x = info.width;
	int y = info.height;
	double m_per_cell = info.resolution;

	int explored_cell_num = 0;

	

	for(int i=0;i<map_data->data.size();i++){
	//for(int i=0;i<data.size();i++){
		//if(data[i] == 0){
		if(map_data->data[i] == 0){
			explored_cell_num++;
		}
	}	

	explored_cell = m_per_cell*m_per_cell*explored_cell_num;
}

void odom_data(const geometry_msgs::Point::ConstPtr& odom_data){
	odom_x = odom_data-> x;
	odom_y = odom_data-> y;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "data_export");
	ros::NodeHandle d;

	odom_data_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_support",1,odom_data,ros::VoidPtr(),&odom_data_queue);
	map_data_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,map_data,ros::VoidPtr(),&map_data_queue);

	odom_data_sub = d.subscribe(odom_data_option);
	map_data_sub = d.subscribe(map_data_option);


	time_t timer;
   	struct tm *local;
   	timer = time(NULL);
   	local = localtime(&timer);

	std::ofstream ofs("export_data.csv",std::ios::app);
	
	ofs << local->tm_year+1900 << "/" << local->tm_mon+1 << "/" << local->tm_mday << " " << local->tm_hour << ":" << local->tm_min << ":" << local->tm_sec << std::endl;
	//ofs << "odom_x,odom_y,探査済み面積[m*m]" << std::endl;
	ofs << "time,探査済み面積[m*m]" << std::endl;


	//ros::Rate rate(0.5);

	start = ros::Time::now();

	while(ros::ok()){
		//odom_data_queue.callOne(ros::WallDuration(1));
		process = ros::Time::now();
		map_data_queue.callOne(ros::WallDuration(1));
		export_data();
		//rate.sleep();
	}

	ofs << "" << std::endl;

	return 0;
}
