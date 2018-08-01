#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Dense>
#include <map_merging/EigenValueFeature.h>
#include <map_merging/FeatureExtraction.h>
#include <map_merging/PairNumber.h>
#include <map_merging/Match.h>

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
  void testMatch(void);

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
  eMatch.sourceMap.header = sSMsg -> cluHeader;
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
  eMatch.mergedMap.header = sMMsg -> cluHeader;
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

  for(int i=0;i<eMatch.mergedMap.clusterList.size();i++)
  {
    minDiff = 100.0;
    minNum = -1;

    for(int j=0;j<eMatch.sourceMap.clusterList.size();j++)
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
        if(std::abs(eMatch.mergedMap.centroids[i].z - eMatch.sourceMap.centroids[j].z) < zThre)
        {
          if(diffVector < minDiff)
          {
            mcX = eMatch.mergedMap.centroids[i].x;
            mcY = eMatch.mergedMap.centroids[i].y;
            mcZ = eMatch.mergedMap.centroids[i].z;

            scX = eMatch.sourceMap.centroids[j].x;
            scY = eMatch.sourceMap.centroids[j].y;
            scZ = eMatch.sourceMap.centroids[j].z;

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
  geometry_msgs::Point centroid;

  for(int i=0;i<eMatch.sourceMap.clusterList.size();i++)
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

      centroid.x = eMatch.sourceMap.centroids[i].x;
      centroid.y = eMatch.sourceMap.centroids[i].y;
      centroid.z = eMatch.sourceMap.centroids[i].z;
      matchPair.sourceCentroid = centroid;

      centroid.x = eMatch.mergedMap.centroids[minNum].x;
      centroid.y = eMatch.mergedMap.centroids[minNum].y;
      centroid.z = eMatch.mergedMap.centroids[minNum].z;
      matchPair.mergedCentroid = centroid;

      matchList.push_back(matchPair);
    }
  }

  for(int i=0;i<matchList.size();i++)
  {
    std::cout << "[true-matching!!] master_cloud[" << matchList[i].mergedNumber << "]" <<  " and source_cloud[" << matchList[i].sourceNumber << "]" << '\n';
    std::cout << "[centroid_z_diff] " << std::abs(eMatch.mergedMap.centroids[matchList[i].mergedNumber].z - eMatch.sourceMap.centroids[matchList[i].sourceNumber].z) << '\n';
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

void EigenValueMatching::testMatch(void)
{
  float diffLinearity;
	float diffPlanarity;
	float diffScattering;
	float diffOmnivariance;
	float diffAnisotropy;
	float diffEigenentropy;
	float diffChangeOfCurvature;
  float diffVector;

  float minDiff;
  int minNum;

  const float zThre = 0.5;
  const float matchThreshold = 0.17;

  float scX,scY,scZ,mcX,mcY,mcZ;

  Eigen::Vector3f sourceVector;//位置の制約確認のためのベクトル
  Eigen::Vector3f mergedVector;//位置の制約確認のためのベクトル

  Eigen::Vector3f sourceNewCentroid;//source側の新しく追加予定の重心
  Eigen::Vector3f mergedNewCentroid;//merged側の新しく追加予定の重心

  Eigen::Vector3f sourceListCentroid;//source側のリストにある重心
  Eigen::Vector3f mergedListCentroid;//merged側のリストにある重心

  Eigen::Vector3f compareVector;//ベクトルの差を入れるやつ//比較用
  const float normThreshold = 0.5;


  //std::vector< std::vector<Eigen::Vector4f> > preMatchListList;
  std::vector<Eigen::Vector4f> preMatchList;
  Eigen::Vector4f preMatchPair;//[0]master,[1]source

  //std::vector<int> ;//各回の結果を保存していくもの

    for(int i=0;i<eMatch.mergedMap.clusterList.size();i++)
    {
      minDiff = 100.0;
      minNum = -1;

      for(int j=0;j<eMatch.sourceMap.clusterList.size();j++)
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
          if(std::abs(eMatch.mergedMap.centroids[i].z - eMatch.sourceMap.centroids[j].z) < zThre)
          {
            if(diffVector < minDiff)
            {
              minNum = j;
              minDiff = diffVector;
            }
          }
        }
      }
      //ここまでで、mergedのi番に対して最もらしいsourceの番号が出てくる
      //minNumに何かしらの番号が入っていればそれがマッチングした番号だとする
      if(minNum > -1)
      {
        /*ここで位置の制約*/
        if(preMatchList.size() > 0)//一番最初のマッチングだと位置の条件がないため
        {
          /*初めに追加する(後で真偽の結果を[3]に入れる)*/
          preMatchPair[0] = i;
          preMatchPair[1] = minNum;
          preMatchPair[2] = minDiff;
          preMatchPair[3] = 0;//0:真偽不明、-1:だめぽ、1:良い
          preMatchList.push_back(preMatchPair);

          sourceNewCentroid << eMatch.sourceMap.centroids[minNum].x, eMatch.sourceMap.centroids[minNum].y, eMatch.sourceMap.centroids[minNum].z;
          mergedNewCentroid << eMatch.mergedMap.centroids[i].x, eMatch.mergedMap.centroids[i].y, eMatch.mergedMap.centroids[i].z;




          for(int k=0;k<preMatchList.size()-1;k++)//すでにマッチングリストにあるものとのベクトルを計算
          {
            sourceListCentroid << eMatch.sourceMap.centroids[preMatchList[k](1)].x, eMatch.sourceMap.centroids[preMatchList[k](1)].y, eMatch.sourceMap.centroids[preMatchList[k](1)].z;
            mergedListCentroid << eMatch.mergedMap.centroids[preMatchList[k](0)].x, eMatch.mergedMap.centroids[preMatchList[k](0)].y, eMatch.mergedMap.centroids[preMatchList[k](0)].z;

            sourceVector = sourceListCentroid - sourceNewCentroid;
            mergedVector = mergedListCentroid - mergedNewCentroid;

            /*sourceVectorとmergedVectorを比較*/
            compareVector = mergedVector - sourceVector;

            /*二個のときとそれ以上のときではベクトルの比較をする対象が違う*/
            /*二個のときはお互い、それ以上のときは怪しいベクトルに対して*/

            /*三個のときは、怪しいベクトルがあれば、その２つと、無ければどちらかのベクトルの近くになってるか*/

            /*二個のときと、それ以外のときで処理を分けるべきか？？*/
            if(preMatchList.size() == 2)//二個のときは判定が初期
            {
              if(compareVector.norm() < normThreshold)//比較結果がしきい値に満たない時//同じっぽいとき
              {
                preMatchList[k](3) = 1;
                preMatchList[preMatchList.size()-1](3) = 1;
              }
              else//違うっぽいとき
              {
                preMatchList[k](3) = -1;
                preMatchList[preMatchList.size()-1](3) = -1;
              }
            }
            else//三個以上のときは前の判定を考慮して判定を行う//すでに二個での判定があるため
            {
              if(compareVector.norm() < normThreshold)
              {

              }
              else
              {

              }
            }

          }
          // mcX = eMatch.mergedMap.centroids[i].x;
          // mcY = eMatch.mergedMap.centroids[i].y;
          // mcZ = eMatch.mergedMap.centroids[i].z;
          //
          // scX = eMatch.sourceMap.centroids[j].x;
          // scY = eMatch.sourceMap.centroids[j].y;
          // scZ = eMatch.sourceMap.centroids[j].z;
        }
        else//一番最初のマッチングはそのまま採用する
        {
          preMatchPair[0] = i;
          preMatchPair[1] = minNum;
          preMatchPair[2] = minDiff;
          preMatchPair[3] = 0;//0:真偽不明、-1:だめぽ、1:良い
          preMatchList.push_back(preMatchPair);
        }
        //std::cout << "[matching!!] master_cloud[" << i << "] ("<< mcX << "," << mcY << "," << mcZ << ") and source_cloud[" << minNum << "] (" << scX << "," << scY << "," << scZ << ")" << '\n';
      }
    }
    //ここから上で決めた初期のマッチングポイントを基準として制約をつけたやつをする

}

void EigenValueMatching::emPublisher(void)
{
  eMatch.matchType = 0;
  eMatch.header.stamp = ros::Time::now();
  pubEigenMatch.publish(eMatch);
  std::cout << "published" << '\n';
}
