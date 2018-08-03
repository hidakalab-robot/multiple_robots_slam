#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/Point.h>
#include <map_merging/TowMap.h>
#include <map_merging/Cluster.h>

#include <std_msgs/Empty.h>

class EuclideanClustering
{
private:

  ros::NodeHandle sC;
  ros::NodeHandle pC;
  ros::Subscriber subC;
  ros::Publisher pubC;

  ros::NodeHandle sR;
  ros::Subscriber subR;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;

  std::vector<pcl::PointIndices> clusterIndicesM;

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  map_merging::Cluster clu;

  bool input;

  bool inputR;

  ros::CallbackQueue queueR;

  void inputCombine(const map_merging::TowMap::ConstPtr& sCMsg);
  void ReceiveCheck(const std_msgs::Empty::ConstPtr& msg);
  bool isInputR(void);

public:

  ros::CallbackQueue queueC;

  EuclideanClustering(int nodeType);//nodeType 0:source, 1:merged
	~EuclideanClustering(){};

  bool isInput(void);
  void resetFlag(void);
  void euclideanClustering(void);
  void coloring(void);
  void ListAndCentroid(void);
  void clusterPublisher(void);

};

EuclideanClustering::EuclideanClustering(int nodeType)
:inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
tree(new pcl::search::KdTree<pcl::PointXYZRGB>)
{
  sC.setCallbackQueue(&queueC);
  sR.setCallbackQueue(&queueR);

  if(nodeType == 0)
  {
    subC = sC.subscribe("/map_merging/combining/sCombining",1,&EuclideanClustering::inputCombine,this);
    pubC = pC.advertise<map_merging::Cluster>("/map_merging/clustering/sClustering", 1);
    subR = sR.subscribe("/map_merging/clustering/sReceiveCheck",1,&EuclideanClustering::ReceiveCheck,this);
  }
  else if(nodeType == 1)
  {
    subC = sC.subscribe("/map_merging/combining/mCombining",1,&EuclideanClustering::inputCombine,this);
    pubC = pC.advertise<map_merging::Cluster>("/map_merging/clustering/mClustering", 1);
    subR = sR.subscribe("/map_merging/clustering/mReceiveCheck",1,&EuclideanClustering::ReceiveCheck,this);
  }

  input = false;
  inputR = false;

  ec.setClusterTolerance (0.1);//同じクラスタとみなす距離
  ec.setMinClusterSize (100);//クラスタを構成する最小の点数
  ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
}

void EuclideanClustering::inputCombine(const map_merging::TowMap::ConstPtr& sCMsg)
{
  pcl::fromROSMsg (sCMsg -> cloudObstacles, *inputCloud);

  clu.cloudObstacles = sCMsg -> cloudObstacles;
  clu.cloudMap = sCMsg -> cloudMap;


  input = true;
  std::cout << "input combined map" << '\n';
}

bool EuclideanClustering::isInput(void)
{
  return input;
}

void EuclideanClustering::resetFlag(void)
{
  input = false;
  inputR = false;
}

void EuclideanClustering::euclideanClustering(void)
{
	tree->setInputCloud (inputCloud);
	std::vector<pcl::PointIndices> clusterIndices;//<-何故かここで宣言しないとだめ???????
	ec.setSearchMethod (tree);

	ec.setInputCloud (inputCloud);

	ec.extract (clusterIndices);

	clusterIndicesM = clusterIndices;//インデックスをメンバ変数に保存するやつ

	std::cout << "map_is_clustered" << std::endl;

}

void EuclideanClustering::coloring(void)
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

void EuclideanClustering::ListAndCentroid(void)
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

void EuclideanClustering::clusterPublisher(void)
{
  clu.header.stamp = ros::Time::now();
  while(ros::ok())
  {
    queueR.callOne(ros::WallDuration(1));
    if(isInputR())
    {
      std::cout << "received" << '\n';
      break;
    }
    else
    {
      pubC.publish(clu);
      std::cout << "published" << '\n';
    }
  }
}

void EuclideanClustering::ReceiveCheck(const std_msgs::Empty::ConstPtr& msg)
{
  inputR = true;
}

bool EuclideanClustering::isInputR(void)
{
  return inputR;
}
