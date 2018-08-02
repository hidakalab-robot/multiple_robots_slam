/*全部のマッチング結果を受信して処理する*/
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <map_merging/Match.h>
#include <map_merging/PairNumber.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Empty.h>

class FinalMatching
{
private:

  ros::NodeHandle sE;
  ros::Subscriber subE;

  ros::NodeHandle sS;
  ros::Subscriber subS;

  ros::NodeHandle sN;
  ros::Subscriber subN;

  ros::NodeHandle pM;
  ros::Publisher pubM;

  ros::NodeHandle pL;
  ros::Publisher pubL;

  ros::NodeHandle pC1;
  ros::Publisher pubC1;

  ros::NodeHandle pC2;
  ros::Publisher pubC2;

  ros::NodeHandle pR1;
  ros::Publisher pubR1;

  ros::NodeHandle pR2;
  ros::Publisher pubR2;

  bool inputE;
  bool inputS;
  bool inputN;

  //float shiftPos;
  float SHIFT_POS;


  map_merging::Match inputEigen;
  map_merging::Match inputShot;
  map_merging::Match inputNoMissEigen;

  map_merging::Match finalMatch;

public:

  ros::CallbackQueue queueE;
  ros::CallbackQueue queueS;
  ros::CallbackQueue queueN;

  FinalMatching();
  ~FinalMatching(){};

  void inputEigenMatch(const map_merging::Match::ConstPtr& sEMsg);
  void inputShotMatch(const map_merging::Match::ConstPtr& sSMsg);
  void inputNoMissEigenMatch(const map_merging::Match::ConstPtr& sNMsg);
  bool isInputE(void);
  bool isInputS(void);
  bool isInputN(void);
  void resetFlag(void);
  bool isSameCluster(void);
  void echoMatch(int type);
  void echoClouds(void);
  void finalMatchProcess(void);
  void finalMatchPublisher(void);
  void receiveReport(void);
};

FinalMatching::FinalMatching()
{
  sE.setCallbackQueue(&queueE);
  subE = sE.subscribe("/map_merging/matching/eigenValueMatching",1,&FinalMatching::inputEigenMatch,this);

  sS.setCallbackQueue(&queueS);
  subS = sS.subscribe("/map_merging/matching/shotMatching",1,&FinalMatching::inputShotMatch,this);

  sN.setCallbackQueue(&queueN);
  subN = sN.subscribe("/map_merging/matching/eigenValueMatchingNoMiss",1,&FinalMatching::inputNoMissEigenMatch,this);

  pubM = pM.advertise<map_merging::Match>("/map_merging/matching/finalMatching", 1);

  pubL = pL.advertise<visualization_msgs::MarkerArray>("/map_merging/visualization/Matching", 1);

  pubC1 = pC1.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/matchSourceCloud", 1);
  pubC2 = pC2.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/matchMergedCloud", 1);

  pubR1 = pR1.advertise<std_msgs::Empty>("/map_merging/clustering/sReceiveCheck", 1);
  pubR2 = pR2.advertise<std_msgs::Empty>("/map_merging/clustering/mReceiveCheck", 1);

  inputE = false;
  inputS = false;

  SHIFT_POS = 5.0;
}

void FinalMatching::inputEigenMatch(const map_merging::Match::ConstPtr& sEMsg)
{
  inputEigen = *sEMsg;
  inputE = true;
  std::cout << "input Eigen" << '\n';
}

void FinalMatching::inputShotMatch(const map_merging::Match::ConstPtr& sSMsg)
{
  inputShot = *sSMsg;
  inputS = true;
  std::cout << "input SHOT" << '\n';
}

void FinalMatching::inputNoMissEigenMatch(const map_merging::Match::ConstPtr& sNMsg)
{
  inputNoMissEigen = *sNMsg;
  inputN = true;
  std::cout << "input NoMiss" << '\n';
}

bool FinalMatching::isInputE(void)
{
  return inputE;
}

bool FinalMatching::isInputS(void)
{
  return inputS;
}

bool FinalMatching::isInputN(void)
{
  return inputN;
}

void FinalMatching::resetFlag(void)
{
  inputE = false;
  inputS = false;
  inputN = false;
}

bool FinalMatching::isSameCluster(void)
{
  /*クラスタについているheaderの時刻を見て二種類のマッチングが同じ時刻のクラスタを処理してたらtrue*/
  std::cout << "eigen source stamp >>"  << inputEigen.sourceMap.header.stamp << '\n';
  std::cout << "shot source stamp >>"  << inputShot.sourceMap.header.stamp << '\n';
  std::cout << "eigen merged stamp >>"  << inputEigen.mergedMap.header.stamp << '\n';
  std::cout << "shot merged stamp >>"  << inputShot.mergedMap.header.stamp << '\n';
  if((inputEigen.sourceMap.header.stamp == inputShot.sourceMap.header.stamp) && (inputEigen.mergedMap.header.stamp == inputShot.mergedMap.header.stamp))
  {
    std::cout << "same" << '\n';
    return true;
  }
  else
  {
    std::cout << "not same" << '\n';
    return false;
  }
}

void FinalMatching::receiveReport(void)
{
  std_msgs::Empty e;

  pubR1.publish(e);
  pubR2.publish(e);
}

void FinalMatching::echoClouds(void)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (inputEigen.sourceMap.cloudObstacles, *inputSource);

  for(int i=0;i<inputSource -> points.size();i++)
  {
    inputSource -> points[i].x -= SHIFT_POS;
  }

  sensor_msgs::PointCloud2 shiftSource;
  pcl::toROSMsg (*inputSource, shiftSource);

  pubC1.publish(shiftSource);
  pubC2.publish(inputEigen.mergedMap.cloudObstacles);

}

void FinalMatching::echoMatch(int type)
{
  visualization_msgs::Marker matchLine;

  matchLine.header.frame_id = "map";
  matchLine.header.stamp = ros::Time::now();
  matchLine.type = visualization_msgs::Marker::LINE_LIST;
  matchLine.action = visualization_msgs::Marker::ADD;
  matchLine.pose.orientation.w = 1.0;
  matchLine.scale.x = 0.1;
  matchLine.color.a = 1.0;
  matchLine.lifetime = ros::Duration(0);

  /*表示するマッチングの種類によっての設定*/
  map_merging::Match input;

  switch (type)
  {
    case 0:
      input = inputEigen;
      matchLine.ns = "EigenValue";
      matchLine.color.r = 1.0f;
      matchLine.color.g = 0.0f;
      matchLine.color.b = 0.0f;
      break;
    case 1:
      input = inputShot;
      matchLine.ns = "SHOT";
      matchLine.color.r = 0.0f;
      matchLine.color.g = 1.0f;
      matchLine.color.b = 0.0f;
      break;
    case 2:
      input = finalMatch;
      matchLine.ns = "Final";
      matchLine.color.r = 0.0f;
      matchLine.color.g = 0.0f;
      matchLine.color.b = 1.0f;
      break;
    case 3:
      input = inputNoMissEigen;
      matchLine.ns = "NoMissEigen";
      matchLine.color.r = 1.0f;
      matchLine.color.g = 1.0f;
      matchLine.color.b = 0.0f;
      break;
    default:
      std::cout << "echoMatch arg error : " << type << '\n';
      return;
  }

  visualization_msgs::MarkerArray matchLineList;

  geometry_msgs::Point sCentroid;
	geometry_msgs::Point mCentroid;

  for(int i=0;i<input.matchList.size();i++)
  {
    //sCentroid.x = input.sourceMap.centroids[input.matchList[i].sourceNumber].x - SHIFT_POS;
    //sCentroid.y = input.sourceMap.centroids[input.matchList[i].sourceNumber].y;
    //sCentroid.z = input.sourceMap.centroids[input.matchList[i].sourceNumber].z;

    sCentroid.x = input.matchList[i].sourceCentroid.x - SHIFT_POS;
    sCentroid.y = input.matchList[i].sourceCentroid.y;
    sCentroid.z = input.matchList[i].sourceCentroid.z;

    //mCentroid.x = input.mergedMap.centroids[input.matchList[i].mergedNumber].x;
    //mCentroid.y = input.mergedMap.centroids[input.matchList[i].mergedNumber].y;
    //mCentroid.z = input.mergedMap.centroids[input.matchList[i].mergedNumber].z;

    mCentroid.x = input.matchList[i].mergedCentroid.x;
    mCentroid.y = input.matchList[i].mergedCentroid.y;
    mCentroid.z = input.matchList[i].mergedCentroid.z;

    matchLine.id = i;
    matchLine.points.push_back(sCentroid);
    matchLine.points.push_back(mCentroid);
    matchLineList.markers.push_back(matchLine);
  }

  pubL.publish(matchLineList);

}

void FinalMatching::finalMatchProcess(void)
{
  /*全てのマッチング結果を考慮した何らかの処理*/
  /*例えば or や and などの処理*/
  /*処理結果を新たなMatch型の変数に格納 finalMatch*/
  /*
  両方にマッチングがあり、andしても無くならなければand
  両方にマッチングがあるが、andするとなくなる場合orで信頼度が高い方、もしくは、できる場合は誤マッチング除去の処理
  片方にしか無ければ、それを採用(するしかない)

  両方の同じソースから別のマージドへのマッチングがあった場合、誤マッチング除去の処理を使う
  両方の同じマージドへ別のソースがあった場合、同様

  一致の判断、ソースはナンバーがあるからできるけど、マージドはナンバーがないので重心間の距離がしきい値以下
  //もしくは、マージド側の重心と一番近いクラスタをの番号として設定(一応受信データには入ってる)

  */

  std::vector<map_merging::PairNumber> finalPairList;
  map_merging::PairNumber pair;

  /*両方にマッチングがあるか確認*/

  /*ソース、もしくはマージドが被っていないか確認*/

  /*被っていた奴に関しては両方被っているかどうか確認*/

  /*両方かぶっていた場合、それを採用*/



  for(int i=0;i<inputEigen.matchList.size();i++)
  {
    std::cout << "固有値の方のsource重心表示 << " << inputEigen.matchList[i].sourceCentroid.x << "," << inputEigen.matchList[i].sourceCentroid.y << "," << inputEigen.matchList[i].sourceCentroid.z << '\n';
    std::cout << "固有値の方のmerged重心表示 << " << inputEigen.matchList[i].mergedCentroid.x << "," << inputEigen.matchList[i].mergedCentroid.y << "," << inputEigen.matchList[i].mergedCentroid.z << '\n';
    std::cout << "クラスタ番号 << ( " << inputEigen.matchList[i].sourceNumber << "," << inputEigen.matchList[i].mergedNumber << " )"  << '\n';
  }

  for(int i=0;i<inputShot.matchList.size();i++)
  {
    std::cout << "SHOTの方のsource重心表示 << " << inputShot.matchList[i].sourceCentroid.x << "," << inputShot.matchList[i].sourceCentroid.y << "," << inputShot.matchList[i].sourceCentroid.z << '\n';
    std::cout << "SHOTの方のmerged重心表示 << " << inputShot.matchList[i].mergedCentroid.x << "," << inputShot.matchList[i].mergedCentroid.y << "," << inputShot.matchList[i].mergedCentroid.z << '\n';
    std::cout << "クラスタ番号 << ( " << inputShot.matchList[i].sourceNumber << "," << inputShot.matchList[i].mergedNumber << " )"  << '\n';
  }

  //finalMatch.matchList =
}

void FinalMatching::finalMatchPublisher(void)
{
  finalMatch.header.stamp = ros::Time::now();
  finalMatch.matchType = 2;
  finalMatch.sourceMap = inputEigen.sourceMap;
  finalMatch.mergedMap = inputEigen.mergedMap;

  pubM.publish(finalMatch);
  std::cout << "published" << '\n';
}
