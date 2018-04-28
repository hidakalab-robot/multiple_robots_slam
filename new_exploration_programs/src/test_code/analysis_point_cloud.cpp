#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


/*用途不明ヘッダー*/
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>



class AnalysisPointCloud
{
private:
	ros::NodeHandle apcs;
	ros::NodeHandle apcp;

	ros::Subscriber pc_sub;

	ros::Publisher pc_pub1;
	ros::Publisher pc_pub2;
	ros::Publisher pc_pub3;
	ros::Publisher pc_pub4;
	ros::Publisher pc_pub5;

public:
	ros::CallbackQueue pc_queue;
	AnalysisPointCloud()
	{
		apcs.setCallbackQueue(&pc_queue);
		pc_sub = apcs.subscribe("/camera/depth_registered/points",1,&AnalysisPointCloud::processing_pc,this);
		pc_pub1 = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud1", 1);
		pc_pub2 = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud2", 1);
		pc_pub3 = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud3", 1);
		pc_pub4 = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud4", 1);
		pc_pub5 = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud5", 1);
	};
	~AnalysisPointCloud(){};
	void processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg);
};


void AnalysisPointCloud::processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg)
{
	std::cout << "1" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "2" << std::endl;
	pcl::fromROSMsg (*ppc_msg, *test_cloud);
	std::cout << "get_point_cloud" << std::endl;
	//
	/// saaaatd::cout << "tes_seq: " << test_cloud->header.seq << std::endl;
	// std::cout << "taaes_stamp: " << test_cloud->header.stamp << std::endl;
	//a std::cout << "tes_frame: " << test_cloud->header.frame_id << std::endl;

/*初期状態を出力*/
	sensor_msgs::PointCloud2 edit_cloud1;
	pcl::toROSMsg (*test_cloud, edit_cloud1);

	for(int i=0;i<test_cloud->points.size();i++)
	{
		test_cloud->points[i].x+=5.0;
	}

/*voxel_grid処理*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (test_cloud);
	vg.setLeafSize (0.1f, 0.1f, 0.1f);//voxel size (x,y,z)
	vg.filter (*voxeled_cloud);

/*voxel_grid時点での出力*/
	// sensor_msgs::PointCloud2 edit_cloud2;
	// pcl::toROSMsg (*voxeled_cloud, edit_cloud2);
	//
	// // std::cout << "vox_seq: " << test_cloud->header.seq << std::endl;
	// // std::cout << "vox_stamp: " << test_cloud->header.stamp << std::endl;
	// // std::cout << "vox_frame: " << test_cloud->header.frame_id << std::endl;
	// //
	//
	// std::cout << "before_size: " << test_cloud->points.size() << std::endl;
	// std::cout << "after_size: " << voxeled_cloud->points.size() << std::endl;
	//
	// for(int i=0;i<voxeled_cloud->points.size();i++)
	// {
	// 	voxeled_cloud->points[i].x+=5.0;
	// }

/*論文ではここで床面除去を入れてるけど、取り敢えず平面除去*/
/*地面除去も追加、平面抽出は繰り返さない場合一番大きい平面のみ抽出する*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_deleted_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (500);//RANSACの繰り返し回数
	seg.setDistanceThreshold (0.1);//モデルとどのくらい離れていてもいいか???謎

	seg.setAxis(Eigen::Vector3f (0.0,1.0,0.0));//法線ベクトル
	seg.setEpsAngle(30.0f * (M_PI/180.0f));//許容出来る平面の傾きラジアン

	//int i=0;
	//int nr_points = (int) voxeled_cloud->points.size ();

//	while (voxeled_cloud->points.size () > 0.3 * nr_points)//whileで繰り返すと複数の平面が消せる
	//{
	 seg.setInputCloud (voxeled_cloud);
	 seg.segment (*inliers, *coefficients);
	 if (inliers->indices.size () == 0)
	 {
		 std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		 //break;
	 }
	 else
	 {
		 std::cout << "inliers->indices.size () : " << inliers->indices.size () << std::endl;
	 }

	 /*view_ground 推定した地面部分の色を赤で表示するだけ*/
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr view_ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::copyPointCloud(*voxeled_cloud, *view_ground_cloud);

	 for (int i = 0; i < view_ground_cloud->points.size (); i++)
	 {
		 view_ground_cloud->points[i].r = 255;
		 view_ground_cloud->points[i].g = 255;
		 view_ground_cloud->points[i].b = 255;
	 }

	 for (int i = 0; i < inliers->indices.size(); ++i)
	 {
		 view_ground_cloud->points[inliers->indices[i]].r = 255;
		 view_ground_cloud->points[inliers->indices[i]].g = 0;
		 view_ground_cloud->points[inliers->indices[i]].b = 0;
	 }
	 /*↑ここまで色付け*/

	 pcl::ExtractIndices<pcl::PointXYZ> extract;
	 extract.setInputCloud (voxeled_cloud);
	 extract.setIndices (inliers);

	 extract.setNegative (true);//true:平面を削除、false:平面以外削除
	 extract.filter (*ground_deleted_cloud);
	 //*voxeled_cloud = *cloud_f;
 //}

/*平面除去出力*/

	/*地面色付け出力*/
	sensor_msgs::PointCloud2 edit_cloud5;
	pcl::toROSMsg (*view_ground_cloud, edit_cloud5);

	for(int i=0;i<ground_deleted_cloud->points.size();i++)
	{
		ground_deleted_cloud->points[i].x+=5.0;
	}

	/*地面削除出力*/
	// sensor_msgs::PointCloud2 edit_cloud3;
	// pcl::toROSMsg (*ground_deleted_cloud, edit_cloud3);
	//
	// for(int i=0;i<ground_deleted_cloud->points.size();i++)
	// {
	// 	ground_deleted_cloud->points[i].x+=5.0;
	// }

	std::cout << "del_ground_size: " << ground_deleted_cloud->points.size() << std::endl;

/*ユークリッドクラスタリング処理*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//何か探索用にツリーを作る
	tree->setInputCloud (ground_deleted_cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.2);//同じクラスタとみなす距離
	ec.setMinClusterSize (100);//クラスタを構成する最小の点数
	ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);
	ec.setInputCloud (ground_deleted_cloud);
	ec.extract (cluster_indices);

	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ground_deleted_cloud, *clustered_cloud);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
					clustered_cloud->points[*pit].r = colors[j%12][0];
					clustered_cloud->points[*pit].g = colors[j%12][1];
					clustered_cloud->points[*pit].b = colors[j%12][2];
      }
      j++;
  }

/*ユークリッドクラスタリング出力*/
	sensor_msgs::PointCloud2 edit_cloud4;
	pcl::toROSMsg (*clustered_cloud, edit_cloud4);

	std::cout << "edit_cloud" << std::endl;

	pc_pub1.publish(edit_cloud1);
	//pc_pub2.publish(edit_cloud2);
	//pc_pub3.publish(edit_cloud3);
	pc_pub4.publish(edit_cloud4);
	pc_pub5.publish(edit_cloud5);
	std::cout << "publish_cloud" << std::endl;

/*以下ごみ*/
	/*for(int i=0;i<test_cloud->points.size();i++)
	{
		test_cloud->points[i].x+=3.0;
		std::cout << test_cloud.points[i].x << ", " << test_cloud.points[i].y << ", " << test_cloud.points[i].z << std::endl;
	", " << +test_cloud.points[i].r << ", " << +test_cloud.points[i].g << ", " << +test_cloud.points[i].b << std::endl;
		if(!ros::ok())
			break;
	}*/

	/*for(int i=0;i<voxeled_cloud->points.size();i++)
	{
		std::cout << filtered_cloud->width  << '\n';
		std::cout << filtered_cloud->height  << '\n';
		std::cout << voxeled_cloud->points[i].x << ", " << voxeled_cloud->points[i].y << ", " << voxeled_cloud->points[i].z << std::endl;
	", " << +test_cloud.points[i].r << ", " << +test_cloud.points[i].g << ", " << +test_cloud.points[i].b << std::endl;
}*/
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "analysis_point_cloud");
	AnalysisPointCloud apc;
	while(ros::ok()){
		std::cout << "0" << std::endl;
		apc.pc_queue.callOne(ros::WallDuration(1));
	}
	return 0;
}
