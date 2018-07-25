/*全部のマッチング結果を受信して処理する*/
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <map_merging/Match.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

class FinalMatching
{
private:

  ros::NodeHandle sE;
  ros::Subscriber subE;

  ros::NodeHandle sS;
  ros::Subscriber subS;


  ros::NodeHandle pM;
  ros::Publisher pubM;

  ros::NodeHandle pL;
  ros::Publisher pubL;

  ros::NodeHandle pC1;
  ros::Publisher pubC1;

  ros::NodeHandle pC2;
  ros::Publisher pubC2;

  bool inputE;
  bool inputS;

  float shiftPos;

  map_merging::Match inputEigen;
  map_merging::Match inputShot;

  map_merging::Match finalMatch;

public:

  ros::CallbackQueue queueE;
  ros::CallbackQueue queueS;

  FinalMatching();
  ~FinalMatching(){};

  void inputEigenMatch(const map_merging::Match::ConstPtr& sEMsg);
  void inputShotMatch(const map_merging::Match::ConstPtr& sSMsg);
  bool isInputE(void);
  bool isInputS(void);
  void resetFlag(void);
  bool isSameCluster(void);
  void echoMatch(int type);
  void echoClouds(void);
  void finalMatchProcess(void);
  void finalMatchPublisher(void);
};

FinalMatching::FinalMatching()
{
  sE.setCallbackQueue(&queueE);
  subE = sE.subscribe("/map_merging/matching/eigenValueMatching",1,&FinalMatching::inputEigenMatch,this);

  sS.setCallbackQueue(&queueS);
  subS = sS.subscribe("/map_merging/matching/shotMatching",1,&FinalMatching::inputShotMatch,this);


  pubM = pM.advertise<map_merging::Match>("/map_merging/matching/finalMatching", 1);

  pubL = pL.advertise<visualization_msgs::MarkerArray>("/map_merging/visualization/Matching", 1);

  pubC1 = pC1.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/MatchSourceCloud", 1);
  pubC2 = pC2.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/MatchMergedCloud", 1);

  inputE = false;
  inputS = false;

  shiftPos = 5.0;
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

bool FinalMatching::isInputE(void)
{
  return inputE;
}

bool FinalMatching::isInputS(void)
{
  return inputS;
}

void FinalMatching::resetFlag(void)
{
  inputE = false;
  inputS = false;
}

bool FinalMatching::isSameCluster(void)
{
  /*クラスタについているheaderの時刻を見て二種類のマッチングが同じ時刻のクラスタを処理してたらtrue*/
  if((inputEigen.sourceMap.header.stamp == inputShot.sourceMap.header.stamp) && (inputEigen.mergedMap.header.stamp == inputShot.mergedMap.header.stamp))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void FinalMatching::echoClouds(void)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputSource(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (inputEigen.sourceMap.cloudObstacles, *inputSource);

  for(int i=0;i<inputSource -> points.size();i++)
  {
    inputSource -> points[i].x -= shiftPos;
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
  matchLine.lifetime = ros::Duration(3);

  /*表示するマッチングの種類によっての設定*/
  map_merging::Match input;

  switch (type) {
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
    default:
      std::cout << "echoMatch arg error" << '\n';
      return;
  }

  visualization_msgs::MarkerArray matchLineList;

  geometry_msgs::Point sCentroid;
	geometry_msgs::Point mCentroid;

  for(int i=0;i<input.matchList.size();i++)
  {
    sCentroid.x = input.sourceMap.centroids[input.matchList[i].sourceNumber].x - shiftPos;
    sCentroid.y = input.sourceMap.centroids[input.matchList[i].sourceNumber].y;
    sCentroid.z = input.sourceMap.centroids[input.matchList[i].sourceNumber].z;

    mCentroid.x = input.mergedMap.centroids[input.matchList[i].mergedNumber].x;
    mCentroid.y = input.mergedMap.centroids[input.matchList[i].mergedNumber].y;
    mCentroid.z = input.mergedMap.centroids[input.matchList[i].mergedNumber].z;

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
  /*処理結果を新たなMatch型の変数に格納*/
}

void FinalMatching::finalMatchPublisher(void)
{
  finalMatch.header.stamp = ros::Time::now();
  pubM.publish(finalMatch);
  std::cout << "published" << '\n';
}
