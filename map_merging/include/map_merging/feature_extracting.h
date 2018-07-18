#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>
#include <map_merging/Cluster.h>
#include <map_merging/FeatureExtraction.h>

class FeatureExtracting
{
private:
  ros::NodeHandle sC;
  ros::NodeHandle pF;
  ros::Subscriber subC;
  ros::Publisher pubF;

  map_merging::FeatureExtraction feaExt;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;

  bool input;

public:
  ros::CallbackQueue queueC;

  FeatureExtracting();
	~FeatureExtracting(){};

  void inputCluster(const map_merging::Cluster::ConstPtr& sCMsg);
  bool isInput(void);
  void resetFlag(void);
  void featureExtraction(void);
  void featurePublisher(void);
};

FeatureExtracting::FeatureExtracting()
:inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  sC.setCallbackQueue(&queueC);
  subC = sC.subscribe("map_merging/sClustering",1,&FeatureExtracting::inputCluster,this);
  pubF = pF.advertise<map_merging::FeatureExtraction>("map_merging/sFeature", 1);

  input = false;
}

void FeatureExtracting::inputCluster(const map_merging::Cluster::ConstPtr& sCMsg)
{
  feaExt.cloudMap = sCMsg -> cloudMap;
  feaExt.cloudObstacles = sCMsg -> cloudObstacles;
  feaExt.cloudColor = sCMsg -> cloudColor;
  feaExt.clusterList = sCMsg -> clusterList;
  feaExt.centroids = sCMsg -> centroids;

  pcl::fromROSMsg (sCMsg -> cloudObstacles, *inputCloud);

  std::cout << "input cluster" << '\n';
}

bool FeatureExtracting::isInput(void)
{
  return input;
}

void FeatureExtracting::resetFlag(void)
{
  input = false;
}

void FeatureExtracting::featureExtraction(void)
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

	std::cout << "feature_is_extracted" << std::endl;

}

void FeatureExtracting::featurePublisher(void)
{
  feaExt.header.stamp = ros::Time::now();
  pubF.publish(feaExt);
  std::cout << "published" << '\n';
}
