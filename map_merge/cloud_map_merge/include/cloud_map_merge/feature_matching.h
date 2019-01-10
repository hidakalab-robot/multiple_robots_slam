#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cloud_map_merge/Cluster.h>
#include <cloud_map_merge/EigenValueFeature.h>
#include <cloud_map_merge/MatchList.h>
#include <visualization_msgs/MarkerArray.h>

namespace FeatureMatching
{
	class Eigenvalue
	{
	private:

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
		std::vector<cloud_map_merge::Cluster> clusterData;
		std::vector< std::vector<pcl::PointIndices> > clusterIndices;
		std::vector< std::vector<cloud_map_merge::EigenValueFeature> > clusterFeatures;
		cloud_map_merge::MatchList clusterMatchList;

		void deleteCeiling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
		void euclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& indices);
		void coloringCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<pcl::PointIndices>& indices);
		void listAndCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cloud_map_merge::Cluster& data, std::vector<pcl::PointIndices>& indices);
		void extract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cloud_map_merge::Cluster& clusterData, std::vector<cloud_map_merge::EigenValueFeature>& clusterFeature);
		void calcMatch(void);
		void missMatchDetection(void);


	public:

		Eigenvalue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud2);
		~Eigenvalue(){};

		void clustering(void);
		void featureExtraction(void);
		void matching(void);
		void getMatchingGap(geometry_msgs::Point& gap);
		void writeMatchingLine(ros::Publisher& pubLine);
	};
}

FeatureMatching::Eigenvalue::Eigenvalue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud0, pcl::PointCloud<pcl::PointXYZRGB>::Ptr argCloud1)
{
	/*引数として受け取ったpointcloudを使って初期化*/
	clouds.resize(2);
	clouds[0] = argCloud0;
	clouds[1] = argCloud1;

	std::cout << "check arg clouds size << " << argCloud0 -> points.size() << " << " << argCloud1 -> points.size() << std::endl; 
	std::cout << "check clouds size << " << clouds[0] -> points.size() << " << " << clouds[1] -> points.size() << std::endl; 

	clusterData.resize(2);
	clusterIndices.resize(2);
	clusterFeatures.resize(2);

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
	const double ceilingHeight = 2.5;

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
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	ec.setClusterTolerance (0.1);//同じクラスタとみなす距離
  	ec.setMinClusterSize (100);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数

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
	for (int i=0;i<2;i++)
	{
		extract(clouds[i],clusterData[i],clusterFeatures[i]);
	}

	std::cout << "feature_is_extracted" << std::endl;
}

void FeatureMatching::Eigenvalue::extract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cloud_map_merge::Cluster& clusterData, std::vector<cloud_map_merge::EigenValueFeature>& clusterFeature)
{
	/*3*3の共分散行列を作る*/
	std::vector<Eigen::Matrix3f> vcMatrices;
	Eigen::Matrix3f vcMatrix;
	Eigen::Vector3f point;
	Eigen::Vector3f p;
	Eigen::RowVector3f pT;
	Eigen::Matrix3f matSum = Eigen::Matrix3f::Zero();
	Eigen::Vector3f centroid;


	for(int i=0;i<clusterData.clusterList.size();i++)
	{
		centroid << clusterData.centroids[i].x,clusterData.centroids[i].y,clusterData.centroids[i].z;

		for(int j=0;j<clusterData.clusterList[i].index.size();j++)
		{
			point << cloud->points[clusterData.clusterList[i].index[j]].x,cloud->points[clusterData.clusterList[i].index[j]].y,cloud->points[clusterData.clusterList[i].index[j]].z;
			p = point-centroid;
			pT << p(0),p(1),p(2);
			matSum = matSum + p*pT;
		}
		vcMatrix = matSum/clusterData.clusterList[i].index.size();
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

	cloud_map_merge::EigenValueFeature feature;
	std::vector<cloud_map_merge::EigenValueFeature> features;

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

	clusterFeature = features;
}

void FeatureMatching::Eigenvalue::matching(void)
{
	calcMatch();
	missMatchDetection();
}

void FeatureMatching::Eigenvalue::calcMatch(void)
{
  	//ソースクラウド
  	float diffLinearity;
	float diffPlanarity;
	float diffScattering;
	float diffOmnivariance;
	float diffAnisotropy;
	float diffEigenentropy;
	float diffChangeOfCurvature;

  	float diffVector;

  	/*重心確認用*/
  	float scX,scY,scZ,mcX,mcY,mcZ;

  	std::vector<Eigen::Vector3f> preMatchList;
  	Eigen::Vector3f preMatchPair;//[0]master,[1]source

  	const float matchThreshold = 0.17;

  	float minDiff;
  	int minNum;

  	float zThre = 0.5;

	//clusterData[i].clusterList.size();

	for(int i=0;i<clusterData[0].clusterList.size();i++)
  	{
    	minDiff = 100.0;
    	minNum = -1;

    	for(int j=0;j<clusterData[1].clusterList.size();j++)
    	{
      		diffLinearity = clusterFeatures[0][i].linearity - clusterFeatures[1][j].linearity;
			diffPlanarity = clusterFeatures[0][i].planarity - clusterFeatures[1][j].planarity;
			diffScattering = clusterFeatures[0][i].scattering - clusterFeatures[1][j].scattering;
			diffOmnivariance = clusterFeatures[0][i].omnivariance - clusterFeatures[1][j].omnivariance;
			diffAnisotropy = clusterFeatures[0][i].anisotropy - clusterFeatures[1][j].anisotropy;
			diffEigenentropy = clusterFeatures[0][i].eigenentropy - clusterFeatures[1][j].eigenentropy;
			diffChangeOfCurvature = clusterFeatures[0][i].changeOfCurvature - clusterFeatures[1][j].changeOfCurvature;

			diffVector = sqrt(pow(diffLinearity,2)+pow(diffPlanarity,2)+pow(diffScattering,2)+pow(diffOmnivariance,2)+pow(diffAnisotropy,2)+pow(diffEigenentropy,2)+pow(diffChangeOfCurvature,2));

			if(diffVector < matchThreshold)
			{
				if(std::abs(clusterData[0].centroids[i].z - clusterData[1].centroids[j].z) < zThre)
				{
					if(diffVector < minDiff)
					{
						mcX = clusterData[0].centroids[i].x;
						mcY = clusterData[0].centroids[i].y;
						mcZ = clusterData[0].centroids[i].z;

						scX = clusterData[1].centroids[j].x;
						scY = clusterData[1].centroids[j].y;
						scZ = clusterData[1].centroids[j].z;

						minNum = j;
						minDiff = diffVector;
					}
				}
			}
    	}
		if(minNum > -1)
		{
			std::cout << "[matching!!] master_cloud[" << i << "] ("<< mcX << "," << mcY << "," << mcZ << ") and source_cloud[" << minNum << "] (" << scX << "," << scY << "," << scZ << ")" << '\n';

			preMatchPair[0] = i;
			preMatchPair[1] = minNum;
			preMatchPair[2] = minDiff;
			preMatchList.push_back(preMatchPair);
		}

  	}

  /*第一次誤マッチング削除*/

	std::vector<int> num;
	std::vector<geometry_msgs::Point> centroid;

	num.resize(2);
	centroid.resize(2);

	cloud_map_merge::MatchPair matchPair;


	for(int i=0;i<clusterData[1].clusterList.size();i++)
	{
		minDiff = 100.0;
		minNum = -1;

		for(int j=0;j<preMatchList.size();j++)
		{
			if((int)preMatchList[j](1) == i)
			{
				if(preMatchList[j](2) < minDiff)
				{
					minDiff = preMatchList[j](2);
					minNum = (int)preMatchList[j](0);
				}
			}
		}
		if(minNum > -1)
		{
			num[0] = minNum;
			num[1] = i;

			centroid[0].x = clusterData[0].centroids[minNum].x;
			centroid[0].y = clusterData[0].centroids[minNum].y;
			centroid[0].z = clusterData[0].centroids[minNum].z;

			centroid[1].x = clusterData[1].centroids[i].x;
			centroid[1].y = clusterData[1].centroids[i].y;
			centroid[1].z = clusterData[1].centroids[i].z;


			matchPair.num = num;
			matchPair.centroid = centroid;

			clusterMatchList.list.push_back(matchPair);
		}
	}

	for(int i=0;i<clusterMatchList.list.size();i++)
	{
		std::cout << "[true-matching!!] master_cloud[" << clusterMatchList.list[i].num[0] << "]" <<  " and source_cloud[" << clusterMatchList.list[i].num[1] << "]" << '\n';
		std::cout << "[centroid_z_diff] " << std::abs(clusterData[0].centroids[clusterMatchList.list[i].num[0]].z - clusterData[1].centroids[clusterMatchList.list[i].num[1]].z) << '\n';
	}

}

void FeatureMatching::Eigenvalue::missMatchDetection(void)
{
	/*calcMatchで初めのマッチングをした後に、それのミスマッチを探す*/

	/*マッチリストが3つ未満のときは終了*/
	if(clusterMatchList.list.size() < 3)
	{
		return;
	}

	/*マッチリスト中の各重心間ベクトル(全て)を計算*/
	std::vector<Eigen::Vector3f> sourceVectors;//ソース地図中のベクトルを格納
	Eigen::Vector3f sVector;//計算用
	std::vector<Eigen::Vector3f> mergedVectors;//マージ地図中のベクトルを格納
	Eigen::Vector3f mVector;//計算用
	//std::vector<Eigen::Vector3f> diffVectors;//ソース中とマージド中のベクトルの差を格納
	std::vector<float> diffVectors;//ソース中とマージド中のベクトルの差を格納
	Eigen::Vector3f dVector;//計算用

	std::vector<Eigen::Vector2i> pairNumbers;//ベクトルを計算するときに使ったマッチングペアの番号
	Eigen::Vector2i pNum;//計算用

	for(int i=0;i<clusterMatchList.list.size();i++)
	{
		for(int j=i+1;j<clusterMatchList.list.size();j++)
		{
			sVector << clusterMatchList.list[i].centroid[1].x - clusterMatchList.list[j].centroid[1].x, clusterMatchList.list[i].centroid[1].y - clusterMatchList.list[j].centroid[1].y, clusterMatchList.list[i].centroid[1].z - clusterMatchList.list[j].centroid[1].z;
			mVector << clusterMatchList.list[i].centroid[0].x - clusterMatchList.list[j].centroid[0].x, clusterMatchList.list[i].centroid[0].y - clusterMatchList.list[j].centroid[0].y, clusterMatchList.list[i].centroid[0].z - clusterMatchList.list[j].centroid[0].z;

			sourceVectors.push_back(sVector);
			mergedVectors.push_back(mVector);

			dVector = sVector - mVector;
			diffVectors.push_back(dVector.norm());

			pNum << i, j;
			pairNumbers.push_back(pNum);
		}
	}
	//ここまでで重心間ベクトルとベクトルの差を計算

	/*ベクトルの差を見て正しそうかを検討//しきい値を使って判断*/

	const float normThreshold = 0.35;

	const int size = clusterMatchList.list.size();

	//int matchEvaluation[size] = {};//マッチが正しいかどうかを記述しておく//1:合ってる -1:間違ってる 0:不明

	int matchEvaluation[size];

	for(int i=0;i<size;i++)
	{
		matchEvaluation[i] = 0;
	}

	for(int i=0;i<diffVectors.size();i++)
	{
		/*すでに判定が行われているマッチングについては処理を行わないようにする*/
		//std::cout << "diffVectors[" << i << "] << " << diffVectors[i] <<  '\n';
		//std::cout << "pairNumbers[" << i << "] << (" << pairNumbers[i][0] << "," << pairNumbers[i][1] << ")" << '\n';

		if((matchEvaluation[pairNumbers[i][0]] != 0) && (matchEvaluation[pairNumbers[i][1]] != 0))
		{
			//std::cout << "skip!!" << '\n';
			continue;
		}

		if(diffVectors[i] > normThreshold)//差のノルムとしきい値を比較
		{
			/*i番の差を計算するのに使われているマッチングを使った別のマッチングを見る*/
			/*pairNumbers[i][0]とpairNumbers[i][1]を見る*/
			/*まず、matchEvaluationを見て、すでにどちらかの判定がでてないか見る*/
			//std::cout << "pointA" << '\n';
			if((matchEvaluation[pairNumbers[i][0]] == 1) || (matchEvaluation[pairNumbers[i][1]] == 1))
			{
				/*片方に合ってる判定が出てればもう片方が間違っている*/
				if(matchEvaluation[pairNumbers[i][0]] == 1)
				{
					matchEvaluation[pairNumbers[i][1]] = -1;
					//std::cout << "pointA-1" << '\n';
				}
				else
				{
					matchEvaluation[pairNumbers[i][0]] = -1;
					//std::cout << "pointA-2" << '\n';
				}
			}
			else/*どちらにも合っている判定がないので周りのベクトルと比較*/
			{
				//std::cout << "pointB" << '\n';
				/*片方が間違っていることがわかってればもう片方について調べる*/
				if(matchEvaluation[pairNumbers[i][0]] == -1)
				{
					//pairNumbers[i][1]について調べる//pairNumbers[i][1]を使った他のマッチングを見る
					for(int j=0;j<diffVectors.size();j++)
					{
						if(i == j)
						{
							continue;
						}
						if((pairNumbers[j][0] == pairNumbers[i][1]) || (pairNumbers[j][1] == pairNumbers[i][1]))
						{
							/*pairNumbers[i][1]を使った他のマッチングが合ってるかどうか判断*/
							if(diffVectors[j] <= normThreshold)
							{
								matchEvaluation[pairNumbers[i][1]] == 1;
								//std::cout << "pointB-1" << '\n';
								break;
							}
						}
					}
				}
				else if(matchEvaluation[pairNumbers[i][1]] == -1)
				{
					//pairNumbers[i][0]について調べる
					//std::cout << "pointC" << '\n';
					for(int j=0;j<diffVectors.size();j++)
					{
						if(i == j)
						{
							continue;
						}
						if((pairNumbers[j][0] == pairNumbers[i][0]) || (pairNumbers[j][1] == pairNumbers[i][0]))
						{
							/*pairNumbers[i][1]を使った他のマッチングが合ってるかどうか判断*/
							if(diffVectors[j] <= normThreshold)
							{
								matchEvaluation[pairNumbers[i][0]] == 1;
								//std::cout << "pointC-1" << '\n';
								break;
							}
						}
					}
				}
				else /*両方不明のとき*/
				{
					//std::cout << "pointD" << '\n';
					for(int j=0;j<diffVectors.size();j++)
					{
						if(i == j)
						{
							continue;
						}
						if((pairNumbers[j][0] == pairNumbers[i][1]) || (pairNumbers[j][1] == pairNumbers[i][1]))
						{
							/*pairNumbers[i][1]を使った他のマッチングが合ってるかどうか判断*/
							if(diffVectors[j] <= normThreshold)
							{
								matchEvaluation[pairNumbers[i][1]] == 1;
								//std::cout << "pointD-1" << '\n';
								break;
							}
						}
					}
					for(int j=0;j<diffVectors.size();j++)
					{
						if(i == j)
						{
							continue;
						}
						if((pairNumbers[j][0] == pairNumbers[i][0]) || (pairNumbers[j][1] == pairNumbers[i][0]))
						{
							/*pairNumbers[i][1]を使った他のマッチングが合ってるかどうか判断*/
							if(diffVectors[j] <= normThreshold)
							{
								matchEvaluation[pairNumbers[i][0]] == 1;
								//std::cout << "pointD-2" << '\n';
								break;
							}
						}
					}
				}
			}
		}
		else//ベクトルの計算に使われたマッチングが合ってそうなので、そのように記述
		{
			/*pairNumbers[i][0]とpairNumbers[i][1]はどちらも合ってそう*/
			//std::cout << "pointE" << '\n';
			matchEvaluation[pairNumbers[i][0]] = 1;
			matchEvaluation[pairNumbers[i][1]] = 1;
		}
	}

	/*ここまでで、matchEvaluationに合ってそうなマッチングの番号が入ったのでそれを使ってリストを作成*/
	std::vector<cloud_map_merge::MatchPair> noMissMatchList;//正しいと判定された物のみ格納

	for(int i=0;i<clusterMatchList.list.size();i++)
	{
		//std::cout << "matchEvaluation[" << i << "] << " << matchEvaluation[i] << '\n';
		if(matchEvaluation[i] == 1)
		{
			noMissMatchList.push_back(clusterMatchList.list[i]);
		}
	}

	/*これで正しいマッチング結果のみが入ったリストができたはず*/

	clusterMatchList.list = noMissMatchList;

}

void FeatureMatching::Eigenvalue::getMatchingGap(geometry_msgs::Point& gap)
{
	//double diffX;
	//double diffY;

	double sumDiffX = 0;
	double sumDiffY = 0;
	double sumDiffZ = 0;

	for(int i=0;i<clusterMatchList.list.size();i++)
	{
		sumDiffX += clusterMatchList.list[i].centroid[1].x - clusterMatchList.list[i].centroid[0].x;
		sumDiffY += clusterMatchList.list[i].centroid[1].y - clusterMatchList.list[i].centroid[0].y;
		sumDiffZ += clusterMatchList.list[i].centroid[1].z - clusterMatchList.list[i].centroid[0].z;
	}

	gap.x = sumDiffX / clusterMatchList.list.size();
	gap.y = sumDiffY / clusterMatchList.list.size();
	gap.z = sumDiffZ / clusterMatchList.list.size();

}

void FeatureMatching::Eigenvalue::writeMatchingLine(ros::Publisher& pubLine)
{

	std::cout << "***** write matching list *****" << std::endl;

	visualization_msgs::Marker matchLine;

	matchLine.header.frame_id = "/server/merge_map";
	matchLine.header.stamp = ros::Time::now();
	matchLine.type = visualization_msgs::Marker::LINE_LIST;
	matchLine.action = visualization_msgs::Marker::ADD;
	matchLine.pose.orientation.w = 1.0;
	matchLine.scale.x = 0.1;
	matchLine.color.a = 1.0;
	matchLine.lifetime = ros::Duration(0);

	double SHIFT_X = 1.5;

	/*表示するマッチングの種類によっての設定*/

	matchLine.ns = "EigenValue";
	matchLine.color.r = 0.0f;
	matchLine.color.g = 1.0f;
	matchLine.color.b = 1.0f;
	
//clusterMatchList.list

	visualization_msgs::MarkerArray matchLineList;

	geometry_msgs::Point sCentroid;
	geometry_msgs::Point mCentroid;

	for(int i=0;i<clusterMatchList.list.size();i++)
	{

		sCentroid.x = clusterMatchList.list[i].centroid[0].x;
		sCentroid.y = clusterMatchList.list[i].centroid[0].y;
		sCentroid.z = clusterMatchList.list[i].centroid[0].z;

		mCentroid.x = clusterMatchList.list[i].centroid[1].x;
		mCentroid.y = clusterMatchList.list[i].centroid[1].y;
		mCentroid.z = clusterMatchList.list[i].centroid[1].z;

		matchLine.id = i;
		matchLine.points.push_back(sCentroid);
		matchLine.points.push_back(mCentroid);
		matchLineList.markers.push_back(matchLine);
	}

	pubLine.publish(matchLineList);
}