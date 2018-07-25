#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Dense>
#include <map_merging/EigenValueFeature.h>
#include <map_merging/FeatureExtraction.h>
#include <map_merging/PairNumber.h>
#include <map_merging/Match.h>

#define SOURCE eMatch.sourceMap
#define MERGED eMatch.mergedMap

class EigenValueMatching
{
private:
  ros::NodeHandle sS;
  ros::NodeHandle sM;

  ros::NodeHandle pE;


  ros::Subscriber subS;
	ros::Subscriber subM;

  ros::Publisher pubEigenMatch;

  std::vector<map_merging::EigenValueFeature> sFeature;
  std::vector<map_merging::EigenValueFeature> mFeature;


  map_merging::Match eMatch;

  bool inputS;
	bool inputM;

  bool matching;

public:
  ros::CallbackQueue queueS;
	ros::CallbackQueue queueM;

  EigenValueMatching();
	~EigenValueMatching(){};

  void inputSource(const map_merging::FeatureExtraction::ConstPtr& sSMsg);
  void inputMerged(const map_merging::FeatureExtraction::ConstPtr& sMMsg);
  bool isInputS(void);
  bool isInputM(void);
  void resetFlag(void);
  bool isMatch(void);
  void calcMatch(void);
  void emPublisher(void);

};

EigenValueMatching::EigenValueMatching()
{
  sS.setCallbackQueue(&queueS);
  sM.setCallbackQueue(&queueM);

  subS = sS.subscribe("/map_merging/feature/sFeature",1,&EigenValueMatching::inputSource,this);
  subM = sM.subscribe("/map_merging/feature/mFeature",1,&EigenValueMatching::inputMerged,this);

  pubEigenMatch = pE.advertise<map_merging::Match>("/map_merging/matching/eigenValueMatching", 1);

  inputS = false;
  inputM = false;
  matching = false;
}

void EigenValueMatching::inputSource(const map_merging::FeatureExtraction::ConstPtr& sSMsg)
{
  eMatch.sourceMap.cloudMap = sSMsg -> cloudMap;
  eMatch.sourceMap.cloudObstacles = sSMsg -> cloudObstacles;
  eMatch.sourceMap.cloudColor = sSMsg -> cloudColor;
  eMatch.sourceMap.clusterList = sSMsg -> clusterList;
  eMatch.sourceMap.centroids = sSMsg -> centroids;

  sFeature = sSMsg -> featureList;

  inputS = true;

  std::cout << "input S_map " << std::endl;
}

void EigenValueMatching::inputMerged(const map_merging::FeatureExtraction::ConstPtr& sMMsg)
{
  eMatch.mergedMap.cloudMap = sMMsg -> cloudMap;
  eMatch.mergedMap.cloudObstacles = sMMsg -> cloudObstacles;
  eMatch.mergedMap.cloudColor = sMMsg -> cloudColor;
  eMatch.mergedMap.clusterList = sMMsg -> clusterList;
  eMatch.mergedMap.centroids = sMMsg -> centroids;

  mFeature = sMMsg -> featureList;

  inputM = true;

  std::cout << "input M_map " << std::endl;
}

bool EigenValueMatching::isInputS(void)
{
  return inputS;
}

bool EigenValueMatching::isInputM(void)
{
  return inputM;
}

void EigenValueMatching::resetFlag(void)
{
  inputS = false;
  inputM = false;
}

bool EigenValueMatching::isMatch(void)
{
  return matching;
}

void EigenValueMatching::calcMatch(void)
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

  for(int i=0;i<MERGED.clusterList.size();i++)
  {
    minDiff = 100.0;
    minNum = -1;

    for(int j=0;j<SOURCE.clusterList.size();j++)
    {
      diffLinearity = mFeature[i].linearity - sFeature[j].linearity;
      diffPlanarity = mFeature[i].planarity - sFeature[j].planarity;
      diffScattering = mFeature[i].scattering - sFeature[j].scattering;
      diffOmnivariance = mFeature[i].omnivariance - sFeature[j].omnivariance;
      diffAnisotropy = mFeature[i].anisotropy - sFeature[j].anisotropy;
      diffEigenentropy = mFeature[i].eigenentropy - sFeature[j].eigenentropy;
      diffChangeOfCurvature = mFeature[i].changeOfCurvature - sFeature[j].changeOfCurvature;

      diffVector = sqrt(pow(diffLinearity,2)+pow(diffPlanarity,2)+pow(diffScattering,2)+pow(diffOmnivariance,2)+pow(diffAnisotropy,2)+pow(diffEigenentropy,2)+pow(diffChangeOfCurvature,2));

      if(diffVector < matchThreshold)
      {
        if(std::abs(MERGED.centroids[i].z - SOURCE.centroids[j].z) < zThre)
        {
          if(diffVector < minDiff)
          {
            mcX = MERGED.centroids[i].x;
            mcY = MERGED.centroids[i].y;
            mcZ = MERGED.centroids[i].z;

            scX = SOURCE.centroids[j].x;
            scY = SOURCE.centroids[j].y;
            scZ = SOURCE.centroids[j].z;

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


  /*誤マッチングを削除*/

  map_merging::PairNumber matchPair;
  std::vector<map_merging::PairNumber> matchList;

  for(int i=0;i<SOURCE.clusterList.size();i++)
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
      matchPair.mergedNumber = minNum;
      matchPair.sourceNumber = i;
      matchList.push_back(matchPair);
    }
  }

  for(int i=0;i<matchList.size();i++)
  {
    std::cout << "[true-matching!!] master_cloud[" << matchList[i].mergedNumber << "]" <<  " and source_cloud[" << matchList[i].sourceNumber << "]" << '\n';
    std::cout << "[centroid_z_diff] " << std::abs(MERGED.centroids[matchList[i].mergedNumber].z - SOURCE.centroids[matchList[i].sourceNumber].z) << '\n';
  }


  if(matchList.size()>0)
  {
    matching = true;
    std::cout << "matching is success" << '\n';
  }
  else
  {
    matching = false;
    std::cout << "no matching cloud" << '\n';
  }

  eMatch.matchList = matchList;

}

void EigenValueMatching::emPublisher(void)
{
  eMatch.header.stamp = ros::Time::now();
  pubEigenMatch.publish(eMatch);
  std::cout << "published" << '\n';
}
