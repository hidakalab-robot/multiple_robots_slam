#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

namespace FeatureMatching
{
	class Eigenvalue
	{
	private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;


	public:
		Eigenvalue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud2);
		~Eigenvalue(){};

		void test(void);
	};

}

FeatureMatching::Eigenvalue::Eigenvalue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud2)
{
	/*引数として受け取ったpointcloudを使って初期化*/
	cloud1 = argCloud1;
	cloud2 = argCloud2;
}

void FeatureMatching::Eigenvalue::test(void)
{

}
