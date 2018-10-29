#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <cloud_map_merge/RobotPose.h>
#include <cloud_map_merge/AllRobotData.h>
#include <pcl_ros/point_cloud.h>
#include <cloud_map_merge/feature_matching.h>

class CloudMapMerge
{
private:
    ros::NodeHandle cmm;

    ros::NodeHandle s;

    ros::NodeHandle p;

    ros::Subscriber sub;
    ros::Publisher pub;
    cloud_map_merge::AllRobotData robotData;
    sensor_msgs::PointCloud2 pubCloud;
    std::string subTopic;
    std::string mergeMapFrame;
    std::string pubTopic;


    ros::NodeHandle pO;
    ros::Publisher pubOver0;
    ros::Publisher pubOver1;
    ros::Publisher pubLine;

    bool alignment;

    bool input;

    void callback(const cloud_map_merge::AllRobotData::ConstPtr& msg);

public:
    ros::CallbackQueue queue;
    CloudMapMerge();
    ~CloudMapMerge(){};

    void translation(void);
    void createOverlapCloud(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& origins, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& overlaps, const cloud_map_merge::Overlap& overlapInfo);
    void matchingCloud(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& overlaps, std::vector<cloud_map_merge::RobotPose>& poseErrors, const std::vector<cloud_map_merge::RobotPose>& poses, const cloud_map_merge::Overlap& overlapInfo);
    std::vector<cloud_map_merge::RobotPose> cloudAlignment(void);
    void checkOverlapCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> overlaps, const std::vector<cloud_map_merge::RobotPose>& poses, const cloud_map_merge::Overlap& overlapInfo);
    void merge(void);
    bool isInput(void);
    void resetFlag(void);
    void publish(void);

};

CloudMapMerge::CloudMapMerge()
:cmm("~")
{
    input = false;
    s.setCallbackQueue(&queue);
    cmm.getParam("sub_topic", subTopic);
    cmm.getParam("pub_topic", pubTopic);
    cmm.getParam("merge_map_frame", mergeMapFrame);
    cmm.getParam("alignment", alignment);
    sub = s.subscribe(subTopic,1,&CloudMapMerge::callback,this);
    pub = p.advertise<sensor_msgs::PointCloud2>(pubTopic, 1);

    pubOver0 = pO.advertise<sensor_msgs::PointCloud2>("cloud_map_merge/checkOverlap/cloud0", 1);
    pubOver1 = pO.advertise<sensor_msgs::PointCloud2>("cloud_map_merge/checkOverlap/cloud1", 1);

    pubLine = pO.advertise<visualization_msgs::MarkerArray>("cloud_map_merge/matchingLine", 1);
}

void CloudMapMerge::callback(const cloud_map_merge::AllRobotData::ConstPtr& msg)
{
    robotData = *msg;
    input = true;
    std::cout << "input robot data" << std::endl;
}

bool CloudMapMerge::isInput(void)
{
  return input;
}

void CloudMapMerge::resetFlag(void)
{
  input = false;
}

std::vector<cloud_map_merge::RobotPose> CloudMapMerge::cloudAlignment(void)
{
    //robotData.overlaps に被り位置のデータがある
    //robotData.maps に地図(3D)のデータがある

    std::vector<cloud_map_merge::RobotPose> poseErrors;
    poseErrors.resize(robotData.poses.size());

    // std::vector<int> a;
    // a.resize(2);

    // a[0] = 0;

    //全ての被り用ループ
    for(int i=0;i<robotData.overlaps.size();i++)
    {
        //std::cout << "***** bag check 1 *****" << std::endl;
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > originClouds;
        //originClouds.resize(2);

        //std::cout << "***** bag check 1.5 *****" << std::endl;
        //std::cout << "***** overlaps *****\n" << std::endl;
        //std::cout << robotData.overlaps[i] << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        //std::cout << "***** bag check 2 *****" << std::endl;
        //pcl::fromROSMsg (robotData.maps[robotData.overlaps[i].order[0]], originClouds[0]);
        //std::cout << "check frame_id 1 << " << robotData.maps[robotData.overlaps[i].order[0]].header.frame_id << std::endl;
        pcl::fromROSMsg (robotData.maps[robotData.overlaps[i].order[0]], *tempCloud1);
        //std::cout << "***** bag check 2.1 *****" << std::endl;
        originClouds.push_back(tempCloud1);
        //std::cout << "***** bag check 2.2 *****" << std::endl;
        //pcl::fromROSMsg (robotData.maps[robotData.overlaps[i].order[1]], originClouds[1]);
        //std::cout << "check frame_id 2 << " << robotData.maps[robotData.overlaps[i].order[1]].header.frame_id << std::endl;
        pcl::fromROSMsg (robotData.maps[robotData.overlaps[i].order[1]], *tempCloud2);
        //std::cout << "***** bag check 2.3 *****" << std::endl;
        originClouds.push_back(tempCloud2);

        //std::cout << "***** bag check 3 *****" << std::endl;
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > overlapClouds;
        //overlapClouds.resize(2);

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr nullCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // overlapClouds.push_back(nullCloud);
        // overlapClouds.push_back(nullCloud);

        /*被っている部分だけ抽出したクラウドを作る*/
        //std::cout << "***** bag check 4 *****" << std::endl;
        createOverlapCloud(originClouds,overlapClouds,robotData.overlaps[i]);

        /*抽出した点群同士でマッチング*/
        //std::cout << "***** bag check 6 *****" << std::endl;
        matchingCloud(overlapClouds, poseErrors, robotData.poses, robotData.overlaps[i]);

        /*被っている点群を出力して確かめる*/
        //std::cout << "***** bag check 5 *****" << std::endl;
        checkOverlapCloud(overlapClouds,robotData.poses, robotData.overlaps[i]);
    }

    std::cout << "**** end alignment ****" << std::endl;

    return poseErrors;
}

void CloudMapMerge::createOverlapCloud(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& origins, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& overlaps, const cloud_map_merge::Overlap& overlapInfo)
{
    /*被っている部分だけ抽出したクラウドを作る*/
    // 被り位置 robotData.overlaps.tl & robotData.overlaps.br & robotData.overlaps.resolution & robotData.overlaps.origin
   //std::cout << "***** bag check 4.1 *****" << std::endl;
    double rangeXmax; //被っている部分のXの負側の端
    double rangeXmin; //被っている部分のXの正側の端

    double rangeYmax; //被っている部分のYの負側の端
    double rangeYmin; //被っている部分のYの正側の端

    //std::cout << "**** overlaps array size << " << overlaps.size() << std::endl;

    for(int i=0;i<2;i++)
    {   
        //std::cout << "***** bag check 4.2 *****" << std::endl;
        /*ここでoverlap情報から被っている部分だけ抽出する*/

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        rangeXmax = (overlapInfo.br[i].x * overlapInfo.resolution) + overlapInfo.origin[i].x;
        rangeXmin = (overlapInfo.tl[i].x * overlapInfo.resolution) + overlapInfo.origin[i].x;

        rangeYmax = (overlapInfo.br[i].y * overlapInfo.resolution) + overlapInfo.origin[i].y;
        rangeYmin = (overlapInfo.tl[i].y * overlapInfo.resolution) + overlapInfo.origin[i].y;

        std::cout << "range" << std::endl;
        std::cout << "Xmax << " << rangeXmax << std::endl;
        std::cout << "Xmin << " << rangeXmin << std::endl;
        std::cout << "Ymax << " << rangeYmax << std::endl;
        std::cout << "Ymin << " << rangeYmin << std::endl;

        //std::cout << "***** bag check 4.3 *****" << std::endl;
        for(int j=0;j<origins[i] -> points.size();j++)
        {
            if(origins[i]->points[j].x > rangeXmin && origins[i]->points[j].x < rangeXmax)
                if(origins[i]->points[j].y > rangeYmin && origins[i]->points[j].y < rangeYmax)
                {
                    //std::cout << "***** bag check 4.4*****" << std::endl;
                    //overlaps[i] -> points.push_back(origins[i]->points[j]);
                    tempCloud -> points.push_back(origins[i]->points[j]);
                }
        }

        //std::cout << "***** bag check 4.5 *****" << std::endl;
        // std::cout << "***** overlaps size *****\n" << overlaps[i] -> points.size() << std::endl;
        // overlaps[i] -> width = overlaps[i] -> points.size();
        // overlaps[i] -> height = 1;
        // overlaps[i] -> is_dense = false;
        
        std::cout << "***** overlaps size *****\n" << tempCloud -> points.size() << std::endl;
        tempCloud -> width = tempCloud -> points.size();
        tempCloud -> height = 1;
        tempCloud -> is_dense = false;

        overlaps.push_back(tempCloud);

    }

    std::cout << "check create clouds size << " << overlaps[0] -> points.size() << " << " << overlaps[1] -> points.size() << std::endl; 

}

void CloudMapMerge::checkOverlapCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> overlaps, const std::vector<cloud_map_merge::RobotPose>& poses, const cloud_map_merge::Overlap& overlapInfo)
{
    std::cout << "******* check overlap *******" << std::endl;


    std::vector<sensor_msgs::PointCloud2> overlapMsgs;
    overlapMsgs.resize(2);

    for(int i=0;i<2;i++)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        Eigen::Matrix2d rotation;
        rotation << cos(robotData.poses[overlapInfo.order[i]].yaw) , -sin(robotData.poses[overlapInfo.order[i]].yaw) , sin(robotData.poses[overlapInfo.order[i]].yaw) , cos(robotData.poses[overlapInfo.order[i]].yaw);

        Eigen::Vector2d point;
        Eigen::Vector2d rotatePoint;

        for(int j=0;j<overlaps[i] -> points.size();j++)
        {
            point << overlaps[i] -> points[j].x , overlaps[i] -> points[j].y;
            rotatePoint = rotation * point;
            overlaps[i] -> points[j].x = rotatePoint[0] + robotData.poses[i].x;
            overlaps[i] -> points[j].y = rotatePoint[1] + robotData.poses[i].y;
        }

        pcl::toROSMsg (*overlaps[i], overlapMsgs[i]);
        overlapMsgs[i].header.stamp = ros::Time::now();
        overlapMsgs[i].header.frame_id = "/server/merge_map";
    }


    std::cout << "** checkOverlap **" << std::endl;

    pubOver0.publish(overlapMsgs[0]);
    pubOver1.publish(overlapMsgs[1]);


}

void CloudMapMerge::matchingCloud(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& overlaps, std::vector<cloud_map_merge::RobotPose>& poseErrors, const std::vector<cloud_map_merge::RobotPose>& poses, const cloud_map_merge::Overlap& overlapInfo)
{
    /*抽出した点群同士でマッチングを行ってズレを確認*/
    /*マッチングのやつは二つの点群を入れたら、クラスタリング、特徴抽出、マッチングまでやってくれる(どのくらい動かせば位置が合うかを算出)*/
    /*二つのpointcloudで初期化*/
    /*マッチング結果のどれくらい動かせば良いのか、とポーズのやつを比較なりして最終的なズレを算出*/

    std::cout << "check pass clouds size << " << overlaps[0] -> points.size() << " << " << overlaps[1] -> points.size() << std::endl; 

    FeatureMatching::Eigenvalue ev(overlaps[0],overlaps[1]);

    ev.clustering();
    ev.featureExtraction();
    ev.matching();

    geometry_msgs::Point matchGap;

    ev.getMatchingGap(matchGap); //Point型の位置ずれがでてくる //overlaps[1] - overlaps[0]のgapが出る

    std::cout << "***** matchGap *****\n" <<  matchGap << std::endl;

    ev.writeMatchingLine(pubLine);

    //poseErrors = 

    //matchGapとinitPoseの差を算出

}

void CloudMapMerge::merge(void)
{
    std::vector<cloud_map_merge::RobotPose> poseErrors;

    if(alignment)
    {
        std::cout << "***** call alignment function *****" << std::endl;
        poseErrors = cloudAlignment();
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<robotData.poses.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg (robotData.maps[i], *transCloud);

        Eigen::Matrix2d rotation;
        rotation << cos(robotData.poses[i].yaw) , -sin(robotData.poses[i].yaw) , sin(robotData.poses[i].yaw) , cos(robotData.poses[i].yaw);

        Eigen::Vector2d point;
        Eigen::Vector2d rotatePoint;

        for(int j=0;j<transCloud -> points.size();j++)
        {
            point << transCloud -> points[j].x , transCloud -> points[j].y;
            rotatePoint = rotation * point;
            transCloud -> points[j].x = rotatePoint[0] + robotData.poses[i].x;
            transCloud -> points[j].y = rotatePoint[1] + robotData.poses[i].y;
        }

        *mergedCloud += *transCloud;

    }

    mergedCloud -> width = mergedCloud -> points.size();
    mergedCloud -> height = 1;
    mergedCloud -> is_dense = false;
    pcl::toROSMsg (*mergedCloud, pubCloud);


}

void CloudMapMerge::publish(void)
{

    std::cout << "***** publish merge cloud" << std::endl;
    pubCloud.header.stamp = ros::Time::now();
    pubCloud.header.frame_id = mergeMapFrame;
    pub.publish(pubCloud);
}
