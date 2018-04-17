#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class AnalysisPointCloud
{
private:
	ros::NodeHandle apc;	
	ros::NodeHandle apc2;
	ros::NodeHandle apcp;
	ros::Subscriber pc_sub;
	ros::Subscriber pc_sub2;

	ros::Publisher pc_pub;

	uint32_t height;
	uint32_t width;
	std::vector<sensor_msgs::PointField> fields;
	bool is_bigendian;
	uint32_t  point_step;
	uint32_t  row_step;
	std::vector<uint8_t> data;
	bool is_dense;
	std::string name;
	uint32_t offset;
	uint8_t  datatype;
	uint32_t count;
public:
	ros::CallbackQueue pc_queue;
	ros::CallbackQueue pc_queue2;
	AnalysisPointCloud()
	{
		apc.setCallbackQueue(&pc_queue);
		pc_sub = apc.subscribe("/camera/depth_registered/points",1,&AnalysisPointCloud::pointcloud_callback,this);
		apc2.setCallbackQueue(&pc_queue2);
		pc_sub2 = apc2.subscribe("/camera/depth_registered/points",1,&AnalysisPointCloud::processing_pc,this);
		pc_pub = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud", 1);
	};
	~AnalysisPointCloud(){};
	void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
	void processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg);
};


void AnalysisPointCloud::processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg)
{
	std::cout << "1" << std::endl;
	pcl::PointCloud<pcl::PointXYZ> test_cloud;
	std::cout << "2" << std::endl;
	//pcl_conversions::toPCL(*ppc_msg,pcl_pc);
	pcl::fromROSMsg (*ppc_msg, test_cloud);
	std::cout << "get_point_cloud" << std::endl;

	


	for(int i=0;i<test_cloud.points.size();i++)
	{
		test_cloud.points[i].x+=1.0;
		//std::cout << test_cloud.points[i].x << ", " << test_cloud.points[i].y << ", " << test_cloud.points[i].z << std::endl;
	//", " << +test_cloud.points[i].r << ", " << +test_cloud.points[i].g << ", " << +test_cloud.points[i].b << std::endl;
		if(!ros::ok())
			break;
	}
	std::cout << "edit_cloud" << std::endl;
	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (test_cloud, edit_cloud);
	pc_pub.publish(edit_cloud);
	std::cout << "publish_cloud" << std::endl;
}



void AnalysisPointCloud::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
	height = pc_msg -> height;
	width = pc_msg -> width;
	fields = pc_msg -> fields;
	is_bigendian = pc_msg -> is_bigendian;
	point_step = pc_msg -> point_step;
	row_step = pc_msg -> row_step;
	data = pc_msg -> data;
	is_dense = pc_msg -> is_dense;

	std::cout << "height: " << height << "\n" << "width: " << width << "\n" << "is_bigendian: " << is_bigendian <<  "\n" << "point_step: " << point_step << "\n" << "row_step: " << row_step << "\n" << "data_size: " << data.size() << "\n" << "is_dense: " << is_dense << std::endl;


	for(int i=0;i<fields.size();i++)
	{
		name = fields[i].name;
		offset = fields[i].offset;
		datatype = fields[i].datatype;
		count = fields[i].count;
		//std::cout << "name: " << name << ", " << "offset: " << offset << ", " << "datatype: " << +datatype <<  ", " << "count: " << count << std::endl;
		//std::cout << "name: " << name << std::endl;
		//std::cout << "offset: " << offset << std::endl; 
		//std::cout << "datatype: " << datatype << std::endl; 
		//std::cout << "count: " << count << std::endl; 
	}
	
	for(int i=0;i<100/*data.size()*/;i++)
	{
		std::cout << "data[" << i << "]: " << +data[i] << std::endl;
		if(!ros::ok())
			break;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "analysis_point_cloud");
	AnalysisPointCloud apc;
	while(ros::ok()){
		//apc.pc_queue.callOne(ros::WallDuration(1));
		std::cout << "0" << std::endl;
		apc.pc_queue2.callOne(ros::WallDuration(10));
	}
	return 0;
}
