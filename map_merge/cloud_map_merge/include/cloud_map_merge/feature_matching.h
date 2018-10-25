#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cloud_map_merge/Cluster.h>
#include <cloud_map_merge/EigenValueFeature.h>

namespace FeatureMatching
{
	class Eigenvalue
	{
	private:
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;

		double ceilingHeight;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
		std::vector<cloud_map_merge::Cluster> clusterData;
		std::vector< std::vector<pcl::PointIndices> > clusterIndices;

		std::vector< std::vector<cloud_map_merge::EigenValueFeature> > features;

	public:
		Eigenvalue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud2);
		~Eigenvalue(){};

		void clustering(void);
		void deleteCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
		void euclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& indices);
		void coloringCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& indices);
		void listAndCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cloud_map_merge::Cluster& data, std::vector<pcl::PointIndices>& indices);

		void featureExtraction(void);
		void matching(void);
		void getGap(geometry_msgs::Point& gap);
	};

}

FeatureMatching::Eigenvalue::Eigenvalue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud0, pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud1)
:tree(new pcl::search::KdTree<pcl::PointXYZRGB>)
{
	/*引数として受け取ったpointcloudを使って初期化*/
	//cloud1 = argCloud1;
	//cloud2 = argCloud2;
	clouds.resize(2);
	clouds[0] = argCloud0;
	clouds[1] = argCloud1;

	clusterData.resize(2);
	clusterIndices.resize(2);

	ceilingHeight = 2.5;

	ec.setClusterTolerance (0.1);//同じクラスタとみなす距離
  	ec.setMinClusterSize (100);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
}



void FeatureMatching::Eigenvalue::clustering(void)
{
	//これをやると
	//clouds, clusterData にクラスタリングした情報が作成される
	for(int i=0;i<2;i++)
	{
		deleteCeiling(clouds[i]);
		euclideanClustering(clouds[i],clusterIndices[i]);
		coloringCloud(clouds[i],clusterIndices[i]);
		listAndCentroid(clouds[i],clusterData[i],clusterIndices[i]);
	}
}

void FeatureMatching::Eigenvalue::deleteCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  /*天井が邪魔なので消します*/
  /*天井の高さceilingHeight*/

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i=0;i<cloud->points.size();i++)
  {
    if((cloud->points[i].z < ceilingHeight) && (cloud->points[i].z > -0.05))
    {
      tempCloud->points.push_back(cloud->points[i]);
    }
  }

  cloud -> points = tempCloud -> points;
  cloud -> width = tempCloud -> points.size();
  cloud -> height = 1;
  cloud -> is_dense = false;
}

void FeatureMatching::Eigenvalue::euclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& indices)
{
	tree->setInputCloud (cloud);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (indices);
}

void FeatureMatching::Eigenvalue::coloringCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& indices)
{
  	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト

	for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  	{
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			cloud->points[*pit].r = colors[j%12][0];
			cloud->points[*pit].g = colors[j%12][1];
			cloud->points[*pit].b = colors[j%12][2];
      	}
    	j++;
  	}

 	std::cout << "coloring is finished" << std::endl;

}

void FeatureMatching::Eigenvalue::listAndCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cloud_map_merge::Cluster& data, std::vector<pcl::PointIndices>& indices)
{

 	cloud_map_merge::Index index;
	std::vector<cloud_map_merge::Index> cluIndices;

  	geometry_msgs::Point centroid;
	std::vector<geometry_msgs::Point> cluCentroids;

  	float sumX;
	float sumY;
	float sumZ;

	for(int i=0;i<indices.size();i++)
	{
   		sumX = 0;
		sumY = 0;
		sumZ = 0;

		index.index = indices[i].indices;
		cluIndices.push_back(index);

		for(int j=0;j<indices[i].indices.size();j++)
		{
			sumX += cloud->points[indices[i].indices[j]].x;
			sumY += cloud->points[indices[i].indices[j]].y;
			sumZ += cloud->points[indices[i].indices[j]].z;
		}

		centroid.x = sumX/indices[i].indices.size();
		centroid.y = sumY/indices[i].indices.size();
		centroid.z = sumZ/indices[i].indices.size();

		cluCentroids.push_back(centroid);
	}

  	data.clusterList = cluIndices;
	data.centroids = cluCentroids;
}

void FeatureMatching::Eigenvalue::featureExtraction(void)
{
	for (int k=0;k<2;k++)
	{
		/*3*3の共分散行列を作る*/
		std::vector<Eigen::Matrix3f> vcMatrices;
		Eigen::Matrix3f vcMatrix;
		Eigen::Vector3f point;
		Eigen::Vector3f p;
		Eigen::RowVector3f pT;
		Eigen::Matrix3f matSum = Eigen::Matrix3f::Zero();
  		Eigen::Vector3f centroid;


  		for(int i=0;i<feaExt.clusterList.size();i++)
		{
    		centroid << feaExt.centroids[i].x,feaExt.centroids[i].y,feaExt.centroids[i].z;

			for(int j=0;j<feaExt.clusterList[i].index.size();j++)
			{
				point << inputCloud->points[feaExt.clusterList[i].index[j]].x,inputCloud->points[feaExt.clusterList[i].index[j]].y,inputCloud->points[feaExt.clusterList[i].index[j]].z;
				p = point-centroid;
				pT << p(0),p(1),p(2);
				matSum = matSum + p*pT;
			}
			vcMatrix = matSum/feaExt.clusterList[i].index.size();
			vcMatrices.push_back(vcMatrix);
			matSum = Eigen::Matrix3f::Zero();
		}

		/*共分散行列の固有値、固有ベクトルを算出*/

		std::vector<Eigen::Vector3f> eValue;
		float e[3];
  		float eN[3];
		float tmp;

		Eigen::Vector3f eV;

		for(int i=0;i<vcMatrices.size();i++)
		{
			Eigen::EigenSolver<Eigen::Matrix3f> es(vcMatrices[i]);

			/*ここで固有値を正規化と大きい順に並び替えする*/
			e[0] = es.eigenvalues().real()[0];
			e[1] = es.eigenvalues().real()[1];
			e[2] = es.eigenvalues().real()[2];

			for(int i=0;i<3-1;i++)
			{
				for(int j=i+1;j<3;j++)
				{
					if(e[i]<e[j])
					{
						tmp = e[i];
						e[i] = e[i+1];
						e[i+1] = tmp;
					}
				}
			}

			for(int i=0;i<3;i++)
			{
      		//eN[i] = e[i]/(e[0]+e[1]+e[2]);
      		eN[i] = e[i]/(e[0]+e[2]);
			}

    		eV << eN[0],eN[1],eN[2];
			eValue.push_back(eV);

		}

		/*とりあえず固有値ベースの特徴7個を計算して特徴ベクトルとする*/
		float linearity;
		float planarity;
		float scattering;
		float omnivariance;
		float anisotropy;
		float eigenentropy;
		float changeOfCurvature;

  		map_merging::EigenValueFeature feature;
		std::vector<map_merging::EigenValueFeature> features;

		for(int i=0;i<eValue.size();i++)
		{
			linearity = (eValue[i](0)-eValue[i](1))/eValue[i](0);
			planarity = (eValue[i](1)-eValue[i](2))/eValue[i](0);
			scattering = eValue[i](2)/eValue[i](0);
			omnivariance = cbrtf(eValue[i](0)*eValue[i](1)*eValue[i](2));//三重根を計算するやつ
			anisotropy = (eValue[i](0)-eValue[i](2))/eValue[i](0);
			eigenentropy =0;
			for (int j=0;j<3;j++)
			{
				eigenentropy -= eValue[i](j)*log(eValue[i](j));
			}
			changeOfCurvature = eValue[i](2)/(eValue[i](0)+eValue[i](1)+eValue[i](2));

			feature.linearity = linearity;
			feature.planarity = planarity;
			feature.scattering = scattering;
			feature.omnivariance = omnivariance;
			feature.anisotropy = anisotropy;
			feature.eigenentropy = eigenentropy;
			feature.changeOfCurvature = changeOfCurvature;

			features.push_back(feature);
		}

		feaExt.featureList = features;

	}

	std::cout << "feature_is_extracted" << std::endl;
}

void FeatureMatching::Eigenvalue::matching(void)
{

}

void FeatureMatching::Eigenvalue::getGap(geometry_msgs::Point& gap)
{

}