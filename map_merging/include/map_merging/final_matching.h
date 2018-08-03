/*全部のマッチング結果を受信して処理する*/
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <map_merging/Match.h>
#include <map_merging/PairNumber.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Empty.h>
#include <map_merging/ProcessTime.h>

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

  ros::NodeHandle pCC1;
  ros::Publisher pubCC1;

  ros::NodeHandle pCC2;
  ros::Publisher pubCC2;

  ros::NodeHandle pR1;
  ros::Publisher pubR1;

  ros::NodeHandle pR2;
  ros::Publisher pubR2;

  ros::NodeHandle pT;
  ros::Publisher pubT;

  bool inputE;
  bool inputS;

  float SHIFT_POS;

  map_merging::Match inputEigen;
  map_merging::Match inputShot;

  map_merging::Match finalMatch;


  map_merging::ProcessTime processTime;
  ros::Time start;

  void inputEigenMatch(const map_merging::Match::ConstPtr& sEMsg);
  void inputShotMatch(const map_merging::Match::ConstPtr& sSMsg);
  void missMatchDetection(std::vector<map_merging::PairNumber> &finalPairList);

public:

  ros::CallbackQueue queueE;
  ros::CallbackQueue queueS;

  FinalMatching();
  ~FinalMatching(){};


  bool isInputE(void);
  bool isInputS(void);
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

  pubM = pM.advertise<map_merging::Match>("/map_merging/matching/finalMatching", 1);

  pubL = pL.advertise<visualization_msgs::MarkerArray>("/map_merging/visualization/Matching", 1);

  pubC1 = pC1.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/matchSourceCloud", 1);
  pubC2 = pC2.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/matchMergedCloud", 1);

  pubCC1 = pC1.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/colorSourceCloud", 1);
  pubCC2 = pC2.advertise<sensor_msgs::PointCloud2>("/map_merging/visualization/colorMergedCloud", 1);

  pubR1 = pR1.advertise<std_msgs::Empty>("/map_merging/clustering/sReceiveCheck", 1);
  pubR2 = pR2.advertise<std_msgs::Empty>("/map_merging/clustering/mReceiveCheck", 1);

  pubT = pT.advertise<map_merging::ProcessTime>("/map_merging/processTime/finalMatching", 1);

  inputE = false;
  inputS = false;

  SHIFT_POS = 5.0;

  processTime.processName = "FinalMatching";
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
  /*
  std::cout << "eigen source stamp >>"  << inputEigen.sourceMap.header.stamp << '\n';
  std::cout << "shot source stamp >>"  << inputShot.sourceMap.header.stamp << '\n';
  std::cout << "eigen merged stamp >>"  << inputEigen.mergedMap.header.stamp << '\n';
  std::cout << "shot merged stamp >>"  << inputShot.mergedMap.header.stamp << '\n';
  */
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


  pcl::fromROSMsg (inputEigen.sourceMap.cloudColor, *inputSource);

  for(int i=0;i<inputSource -> points.size();i++)
  {
    inputSource -> points[i].x -= SHIFT_POS;
  }

  pcl::toROSMsg (*inputSource, shiftSource);

  pubCC1.publish(shiftSource);
  pubCC2.publish(inputEigen.mergedMap.cloudColor);

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
    default:
      std::cout << "echoMatch arg error : " << type << '\n';
      return;
  }

  visualization_msgs::MarkerArray matchLineList;

  geometry_msgs::Point sCentroid;
	geometry_msgs::Point mCentroid;

  for(int i=0;i<input.matchList.size();i++)
  {
    sCentroid.x = input.matchList[i].sourceCentroid.x - SHIFT_POS;
    sCentroid.y = input.matchList[i].sourceCentroid.y;
    sCentroid.z = input.matchList[i].sourceCentroid.z;

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
  start = ros::Time::now();

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

  bool flagAnd = false;

  /*両方にマッチングがあるか確認*/
  if((inputEigen.matchList.size() > 0) && (inputShot.matchList.size() > 0))
  {
    /*ソース、マージドが被っていないか確認*/
    std::cout << "pointX" << '\n';
    for(int i=0;i<inputEigen.matchList.size();i++)
    {
      std::cout << "pointX-1" << '\n';
      for(int j=0;j<inputShot.matchList.size();j++)
      {
        std::cout << "pointX-2" << '\n';
        if((inputEigen.matchList[i].sourceNumber == inputShot.matchList[j].sourceNumber) && (inputEigen.matchList[i].mergedNumber == inputShot.matchList[j].mergedNumber))
        {
          std::cout << "pointX-3" << '\n';
          finalPairList.push_back(inputShot.matchList[j]);
          flagAnd = true;
        }
      }
    }
  }
  else if(inputEigen.matchList.size() > 0)
  {
    /*EigenMatchしかない場合*/
    std::cout << "pointY" << '\n';
    finalMatch = inputEigen;
    return;
  }
  else
  {
    /*ShotMatchしかない場合*/
    std::cout << "pointZ" << '\n';
    finalMatch = inputShot;
    return;
  }

  /*全部のやつを合わせた誤マッチングの検出のやつ*/
  if(!flagAnd)
  {
    std::cout << "pointW" << '\n';
    missMatchDetection(finalPairList);
  }

  finalMatch.matchList = finalPairList;

}

void FinalMatching::missMatchDetection(std::vector<map_merging::PairNumber> &finalPairList)
{
  /*calcMatchで初めのマッチングをした後に、それのミスマッチを探す*/

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

  map_merging::Match orMatch;

  orMatch = inputEigen;
  std::vector<map_merging::PairNumber> orList;
  orList = inputEigen.matchList;
  for(int i=0;i<inputShot.matchList.size();i++)
  {
    orList.push_back(inputShot.matchList[i]);
  }

  orMatch.matchList = orList;

  for(int i=0;i<orMatch.matchList.size();i++)
  {
    for(int j=i+1;j<orMatch.matchList.size();j++)
    {
      sVector << orMatch.matchList[i].sourceCentroid.x - orMatch.matchList[j].sourceCentroid.x, orMatch.matchList[i].sourceCentroid.y - orMatch.matchList[j].sourceCentroid.y, orMatch.matchList[i].sourceCentroid.z - orMatch.matchList[j].sourceCentroid.z;
      mVector << orMatch.matchList[i].mergedCentroid.x - orMatch.matchList[j].mergedCentroid.x, orMatch.matchList[i].mergedCentroid.y - orMatch.matchList[j].mergedCentroid.y, orMatch.matchList[i].mergedCentroid.z - orMatch.matchList[j].mergedCentroid.z;

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

  const float normThreshold = 0.3;

  const int size = orMatch.matchList.size();

  int matchEvaluation[size] = {};//マッチが正しいかどうかを記述しておく//1:合ってる -1:間違ってる 0:不明

  for(int i=0;i<diffVectors.size();i++)
  {
    /*すでに判定が行われているマッチングについては処理を行わないようにする*/
    std::cout << "diffVectors[" << i << "] << " << diffVectors[i] <<  '\n';
    std::cout << "pairNumbers[" << i << "] << (" << pairNumbers[i][0] << "," << pairNumbers[i][1] << ")" << '\n';

    if((matchEvaluation[pairNumbers[i][0]] != 0) && (matchEvaluation[pairNumbers[i][1]] != 0))
    {
      std::cout << "skip!!" << '\n';
      continue;
    }

    if(diffVectors[i] > normThreshold)//差のノルムとしきい値を比較
    {
      /*i番の差を計算するのに使われているマッチングを使った別のマッチングを見る*/
      /*pairNumbers[i][0]とpairNumbers[i][1]を見る*/
      /*まず、matchEvaluationを見て、すでにどちらかの判定がでてないか見る*/
      std::cout << "pointA" << '\n';
      if((matchEvaluation[pairNumbers[i][0]] == 1) || (matchEvaluation[pairNumbers[i][1]] == 1))
      {
        /*片方に合ってる判定が出てればもう片方が間違っている*/
        if(matchEvaluation[pairNumbers[i][0]] == 1)
        {
          matchEvaluation[pairNumbers[i][1]] = -1;
          std::cout << "pointA-1" << '\n';
        }
        else
        {
          matchEvaluation[pairNumbers[i][0]] = -1;
          std::cout << "pointA-2" << '\n';
        }
      }
      else/*どちらにも合っている判定がないので周りのベクトルと比較*/
      {
        std::cout << "pointB" << '\n';
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
                std::cout << "pointB-1" << '\n';
                break;
              }
            }
          }
        }
        else if(matchEvaluation[pairNumbers[i][1]] == -1)
        {
          //pairNumbers[i][0]について調べる
          std::cout << "pointC" << '\n';
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
                std::cout << "pointC-1" << '\n';
                break;
              }
            }
          }
        }
        else /*両方不明のとき*/
        {
          std::cout << "pointD" << '\n';
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
                std::cout << "pointD-1" << '\n';
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
                std::cout << "pointD-2" << '\n';
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
      std::cout << "pointE" << '\n';
      matchEvaluation[pairNumbers[i][0]] = 1;
      matchEvaluation[pairNumbers[i][1]] = 1;
    }
  }

  /*ここまでで、matchEvaluationに合ってそうなマッチングの番号が入ったのでそれを使ってリストを作成*/
  //std::vector<map_merging::PairNumber> noMissMatchList;//正しいと判定された物のみ格納

  for(int i=0;i<orMatch.matchList.size();i++)
  {
    std::cout << "matchEvaluation[" << i << "] << " << matchEvaluation[i] << '\n';
    if(matchEvaluation[i] == 1)
    {
      finalPairList.push_back(orMatch.matchList[i]);
    }
  }

  /*これで正しいマッチング結果のみが入ったリストができたはず*/

}

void FinalMatching::finalMatchPublisher(void)
{
  finalMatch.header.stamp = ros::Time::now();
  finalMatch.matchType = 2;
  finalMatch.sourceMap = inputEigen.sourceMap;
  finalMatch.mergedMap = inputEigen.mergedMap;

  /*処理時間計算*/
  ros::Duration time;
  time = finalMatch.header.stamp - start;
  processTime.processTime = time.toSec();
  time = finalMatch.header.stamp - inputEigen.header.stamp;
  processTime.sourceProcessTime = time.toSec();
  time = finalMatch.header.stamp - inputShot.header.stamp;
  processTime.mergedProcessTime = time.toSec();

  pubM.publish(finalMatch);
  pubT.publish(processTime);
  std::cout << "published" << '\n';
}
