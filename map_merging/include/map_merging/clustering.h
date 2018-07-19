#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/Point.h>
#include <map_merging/TowMap.h>
#include <map_merging/Cluster.h>

class Clustering
{
private:

  ros::NodeHandle sC1;
  ros::NodeHandle pC1;
  ros::Subscriber subC1;
  ros::Publisher pubC1;

  ros::NodeHandle sC2;
  ros::NodeHandle pC2;
  ros::Subscriber subC2;
  ros::Publisher pubC2;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;

  std::vector<pcl::PointIndices> clusterIndicesM;

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  map_merging::Cluster clu;

  bool input;

public:

  ros::CallbackQueue queueC1;
  ros::CallbackQueue queueC2;

  Clustering();
	~Clustering(){};

  void inputCombine(const map_merging::TowMap::ConstPtr& sCMsg);
  bool isInput(void);
  void resetFlag(void);
  void euclideanClustering(void);
  void coloring(void);
  void ListAndCentroid(void);
  void clusterPublisher1(void);
  void clusterPublisher2(void);
};

Clustering::Clustering()
:inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
tree(new pcl::search::KdTree<pcl::PointXYZRGB>)
{
  sC1.setCallbackQueue(&queueC1);
  subC1 = sC1.subscribe("map_merging/sCombining",1,&Clustering::inputCombine,this);
  pubC1 = pC1.advertise<map_merging::Cluster>("map_merging/sClustering", 1);

  sC2.setCallbackQueue(&queueC2);
  subC2 = sC2.subscribe("map_merging/mCombining",1,&Clustering::inputCombine,this);
  pubC2 = pC2.advertise<map_merging::Cluster>("map_merging/mClustering", 1);

  input = false;

  ec.setClusterTolerance (0.1);//同じクラスタとみなす距離
  ec.setMinClusterSize (100);//クラスタを構成する最小の点数
  ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
}

void Clustering::inputCombine(const map_merging::TowMap::ConstPtr& sCMsg)
{
  pcl::fromROSMsg (sCMsg -> cloudObstacles, *inputCloud);

  clu.cloudObstacles = sCMsg -> cloudObstacles;
  clu.cloudMap = sCMsg -> cloudMap;


  input = true;
  std::cout << "input combined map" << '\n';
}

bool Clustering::isInput(void)
{
  return input;
}

void Clustering::resetFlag(void)
{
  input = false;
}

void Clustering::euclideanClustering(void)
{
	tree->setInputCloud (inputCloud);
	std::vector<pcl::PointIndices> clusterIndices;//<-何故かここで宣言しないとだめ???????
	ec.setSearchMethod (tree);

	ec.setInputCloud (inputCloud);

	ec.extract (clusterIndices);

	clusterIndicesM = clusterIndices;//インデックスをメンバ変数に保存するやつ

	std::cout << "map_is_clustered" << std::endl;

}

void Clustering::coloring(void)
{
  int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*inputCloud, *clusteredCloud);

	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndicesM.begin (); it != clusterIndicesM.end (); ++it)
  {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
					clusteredCloud->points[*pit].r = colors[j%12][0];
					clusteredCloud->points[*pit].g = colors[j%12][1];
					clusteredCloud->points[*pit].b = colors[j%12][2];
      }
      j++;
  }

  std::cout << "coloring is finished" << std::endl;

  pcl::toROSMsg (*clusteredCloud, clu.cloudColor);
}

void Clustering::ListAndCentroid(void)
{

  map_merging::Index index;
	std::vector<map_merging::Index> cluIndices;

  geometry_msgs::Point centroid;
	std::vector<geometry_msgs::Point> cluCentroids;

  float sumX;
	float sumY;
	float sumZ;

	for(int i=0;i<clusterIndicesM.size();i++)
	{
    sumX = 0;
		sumY = 0;
		sumZ = 0;

		index.index = clusterIndicesM[i].indices;
		cluIndices.push_back(index);

		for(int j=0;j<clusterIndicesM[i].indices.size();j++)
		{
				sumX += inputCloud->points[clusterIndicesM[i].indices[j]].x;
				sumY += inputCloud->points[clusterIndicesM[i].indices[j]].y;
				sumZ += inputCloud->points[clusterIndicesM[i].indices[j]].z;
		}

		centroid.x = sumX/clusterIndicesM[i].indices.size();
		centroid.y = sumY/clusterIndicesM[i].indices.size();
		centroid.z = sumZ/clusterIndicesM[i].indices.size();

		cluCentroids.push_back(centroid);
	}

  clu.clusterList = cluIndices;
	clu.centroids = cluCentroids;
}

void Clustering::clusterPublisher1(void)
{
  clu.header.stamp = ros::Time::now();
  pubC1.publish(clu);
  std::cout << "published" << '\n';
}

void Clustering::clusterPublisher2(void)
{
  clu.header.stamp = ros::Time::now();
  pubC2.publish(clu);
  std::cout << "published" << '\n';
}
