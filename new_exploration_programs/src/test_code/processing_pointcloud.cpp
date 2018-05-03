#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <limits>

#include <new_exploration_programs/segmented_cloud.h>

//#include <Eigen/Dense>

class ProcessingPointCloud
{
private:
	ros::NodeHandle ppcs;
	ros::NodeHandle ppcp;

	ros::Subscriber pc_sub;

	ros::Publisher pc_pub1;
	ros::Publisher pc_pub2;
	ros::Publisher pc_pub3;
	ros::Publisher pc_pub4;
	ros::Publisher pc_pub5;
	ros::Publisher seg_pub;

	float camera_position_y;//カメラの高さ
	float ground_position_y;//どのくらいの高さまで床とするか

	float cloud_position;//表示の時それぞれの点群をどれくらい離すか

	float nan_c;

	/*ポイントクラウドの変数定義*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud;
	pcl::VoxelGrid<pcl::PointXYZ> vg;

	pcl::PointCloud<pcl::PointXYZ>::Ptr deleted_ground_cloud;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr for_detect_ground_cloud;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr for_view_ground_cloud;
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;//何か探索用にツリーを作る

	std::vector<pcl::PointIndices> cluster_indices_m;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud;

	sensor_msgs::PointCloud2 orig_cloud;
	sensor_msgs::PointCloud2 vox_cloud;
	sensor_msgs::PointCloud2 gro_cloud;
	sensor_msgs::PointCloud2 del_cloud;
	sensor_msgs::PointCloud2 clu_cloud;

	new_exploration_programs::segmented_cloud source_cloud;



public:
	ros::CallbackQueue pc_queue;
	ProcessingPointCloud()
		:input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		voxeled_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		deleted_ground_cloud (new pcl::PointCloud<pcl::PointXYZ>),
		inliers(new pcl::PointIndices),
		coefficients(new pcl::ModelCoefficients),
		//for_detect_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		//for_view_ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		tree (new pcl::search::KdTree<pcl::PointXYZ>)
		//clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
	{
		ppcs.setCallbackQueue(&pc_queue);
		pc_sub = ppcs.subscribe("/camera/depth_registered/points",1,&ProcessingPointCloud::input_pointcloud,this);
		pc_pub1 = ppcp.advertise<sensor_msgs::PointCloud2>("vox_cloud", 1);
		pc_pub2 = ppcp.advertise<sensor_msgs::PointCloud2>("gro_cloud", 1);
		pc_pub3 = ppcp.advertise<sensor_msgs::PointCloud2>("del_cloud", 1);
		pc_pub4 = ppcp.advertise<sensor_msgs::PointCloud2>("clu_cloud", 1);
		seg_pub = ppcp.advertise<new_exploration_programs::segmented_cloud>("pointcloud_segmentation/master_cloud", 1);
		//seg_pub = ppcp.advertise<new_exploration_programs::segmented_cloud>("pointcloud_segmentation/source_cloud", 1);
		//pc_pub5 = ppcp.advertise<sensor_msgs::PointCloud2>("edit_cloud5", 1);

		camera_position_y = 0.41;
		ground_position_y = 0.3;

		cloud_position = 5.0;

		nan_c = std::numeric_limits<float>::quiet_NaN();

		vg.setLeafSize (0.1f, 0.1f, 0.1f);

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000);//RANSACの繰り返し回数
		seg.setDistanceThreshold (0.1);//モデルとどのくらい離れていてもいいか???謎
		seg.setAxis(Eigen::Vector3f (0.0,1.0,0.0));//法線ベクトル
		seg.setEpsAngle(15.0f * (M_PI/180.0f));//許容出来る平面の傾きラジアン

		ec.setClusterTolerance (0.2);//同じクラスタとみなす距離
		ec.setMinClusterSize (100);//クラスタを構成する最小の点数
		ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
	};
	~ProcessingPointCloud(){};

	void input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
	void apply_voxelgrid(void);
	void delete_ground(void);
	void euclidean_clustering(void);
	void publish_pointcloud(void);
	void feature_extraction(void);
	void publish_segmented(void);
};


void ProcessingPointCloud::input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*pc_msg, *input_cloud);
	pcl::toROSMsg (*input_cloud, orig_cloud);
	std::cout << "input_pointcloud" << std::endl;
}


void ProcessingPointCloud::apply_voxelgrid(void)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (input_cloud);
	vg.filter (*voxeled_cloud);
	std::cout << "pointcloud_voxeled" << std::endl;

/*表示用*/

	for(int i=0;i<voxeled_cloud->points.size();i++)
	{
		voxeled_cloud->points[i].x+=cloud_position;
	}

	pcl::toROSMsg (*voxeled_cloud, vox_cloud);

}

void ProcessingPointCloud::delete_ground(void)
{
	/*ポイントクラウドを複製して高さが一定以上の点のみで平面を計算できるようにする(一定以上の高さだったらNanかInfにする)*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr for_detect_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*voxeled_cloud, *for_detect_ground_cloud);

	for(int i=0;i<for_detect_ground_cloud->points.size();i++)
	{
		if(for_detect_ground_cloud->points[i].y < camera_position_y - ground_position_y)
		{
			for_detect_ground_cloud->points[i].x = nan_c;
			for_detect_ground_cloud->points[i].y = nan_c;
			for_detect_ground_cloud->points[i].z = nan_c;
		}
	}

	 seg.setInputCloud (for_detect_ground_cloud);
	 seg.segment (*inliers, *coefficients);

	 if (inliers->indices.size () == 0)
	 {
		 std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	 }

	 /*view_ground 推定した地面部分の色を赤で表示するだけ*/
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr for_view_ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::copyPointCloud(*voxeled_cloud, *for_view_ground_cloud);

	 for (int i = 0; i < for_view_ground_cloud->points.size (); i++)
	 {
		 for_view_ground_cloud->points[i].r = 255;
		 for_view_ground_cloud->points[i].g = 255;
		 for_view_ground_cloud->points[i].b = 255;

		 if(for_view_ground_cloud->points[i].y < camera_position_y - ground_position_y)
		 {
			 for_view_ground_cloud->points[i].r = 0;
			 for_view_ground_cloud->points[i].g = 255;
			 for_view_ground_cloud->points[i].b = 0;
		 }
	 }

	 for (int i = 0; i < inliers->indices.size(); ++i)
	 {
		 for_view_ground_cloud->points[inliers->indices[i]].r = 255;
		 for_view_ground_cloud->points[inliers->indices[i]].g = 0;
		 for_view_ground_cloud->points[inliers->indices[i]].b = 0;
	 }

	 for(int i=0;i<for_view_ground_cloud->points.size();i++)
	 {
		 for_view_ground_cloud->points[i].x+=cloud_position;
	 }

	 pcl::toROSMsg (*for_view_ground_cloud, gro_cloud);

	 //pcl::ExtractIndices<pcl::PointXYZ> extract;
	 extract.setInputCloud (voxeled_cloud);
	 extract.setIndices (inliers);

	 extract.setNegative (true);//true:平面を削除、false:平面以外削除
	 extract.filter (*deleted_ground_cloud);

	 std::cout << "ground_deleted" << std::endl;

	 for(int i=0;i<deleted_ground_cloud->points.size();i++)
	 {
		 deleted_ground_cloud->points[i].x+=2*cloud_position;
	 }

	 pcl::toROSMsg (*deleted_ground_cloud, del_cloud);

}




void ProcessingPointCloud::euclidean_clustering(void)
{
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//何か探索用にツリーを作る
	tree->setInputCloud (deleted_ground_cloud);
	std::vector<pcl::PointIndices> cluster_indices;//<-何故かここで宣言しないとだめ???????
	//std::cout << "1" << '\n';
	//pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setSearchMethod (tree);
	//std::cout << "2" << '\n';
	ec.setInputCloud (deleted_ground_cloud);
	//std::cout << "3" << '\n';
	// ec.setClusterTolerance (0.2);//同じクラスタとみなす距離
	// ec.setMinClusterSize (100);//クラスタを構成する最小の点数
	// ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
	ec.extract (cluster_indices);

	cluster_indices_m = cluster_indices;//インデックスをメンバ変数に保存するやつ

	std::cout << "size: " << cluster_indices.size() << '\n';
	std::cout << "size_m: " << cluster_indices_m.size() << '\n';

	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*deleted_ground_cloud, *clustered_cloud);

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

	std::cout << "pointcloud_clustered" << std::endl;

	for(int i=0;i<clustered_cloud->points.size();i++)
	{
		clustered_cloud->points[i].x+=cloud_position;
	}

	pcl::toROSMsg (*clustered_cloud, clu_cloud);
}

void ProcessingPointCloud::publish_pointcloud(void)
{
	pc_pub1.publish(vox_cloud);
	pc_pub2.publish(gro_cloud);
	pc_pub3.publish(del_cloud);
	pc_pub4.publish(clu_cloud);
}

void ProcessingPointCloud::feature_extraction(void)
{
	/*クラスタの重心を算出*/
	std::vector<Eigen::Vector3f> centroids;
	//std::vector<Eigen::MatrixXf> centroidsss;
	Eigen::Vector3f centroid;

	float sum_x = 0.0;
	float sum_y = 0.0;
	float sum_z = 0.0;

	for(int i=0;i<cluster_indices_m.size();i++)
	{
		//std::cout << "cluster_indices_m[" << i << "].indices.size():" << cluster_indices_m[i].indices.size() << '\n';
		for(int j=0;j<cluster_indices_m[i].indices.size();j++)
		{
				//std::cout << "j: " << cluster_indices_m[i].indices[j] << '\n';
				sum_x += deleted_ground_cloud->points[cluster_indices_m[i].indices[j]].x;
				sum_y += deleted_ground_cloud->points[cluster_indices_m[i].indices[j]].y;
				sum_z += deleted_ground_cloud->points[cluster_indices_m[i].indices[j]].z;
		}
		centroid << sum_x/cluster_indices_m[i].indices.size(),sum_y/cluster_indices_m[i].indices.size(),sum_z/cluster_indices_m[i].indices.size();
		//centroids.push_back(sum_x/cluster_indices_m[i].indices.size(),sum_y/cluster_indices_m[i].indices.size(),sum_z/cluster_indices_m[i].indices.size());
		centroids.push_back(centroid);
		sum_x = 0;
		sum_y = 0;
		sum_z = 0;
	}

	/*3*3の共分散行列を作る*/
	std::vector<Eigen::Matrix3f> vc_matrices;
	Eigen::Matrix3f vc_matrix;
	Eigen::Vector3f point;
	Eigen::Vector3f p;
	Eigen::RowVector3f p_t;
	Eigen::Matrix3f mat_sum = Eigen::Matrix3f::Zero();

	for(int i=0;i<cluster_indices_m.size();i++)
	{
		for(int j=0;j<cluster_indices_m[i].indices.size();j++)
		{
			point << deleted_ground_cloud->points[cluster_indices_m[i].indices[j]].x,deleted_ground_cloud->points[cluster_indices_m[i].indices[j]].y,deleted_ground_cloud->points[cluster_indices_m[i].indices[j]].z;
			p = point-centroids[i];
			p_t << p(0),p(1),p(2);
			mat_sum = mat_sum + p*p_t;
		}
		vc_matrix = mat_sum/cluster_indices_m[i].indices.size();
		vc_matrices.push_back(vc_matrix);
		mat_sum = Eigen::Matrix3f::Zero();
	}

	/*共分散行列の固有値、固有ベクトルを算出*/
	//std::vector<Eigen::EigenSolver<Eigen::Matrix3f>> es;
	//std::vector<Eigen::Matrix3f> e_value;
	std::vector<Eigen::Vector3f> e_value;
	float e[3];
	float tmp;
	//std::vector<Eigen::Vector3f> e_vector;//これは構造体にしたらいいかも
	//Eigen::Matrix3f v;
	Eigen::Vector3f e_v;
	for(int i=0;i<vc_matrices.size();i++)
	{
		Eigen::EigenSolver<Eigen::Matrix3f> es(vc_matrices[i]);
		//std::cout << es.eigenvalues().real()[0] << '\n';
		//std::cout << es.eigenvalues().real()[1] << '\n';
		//std::cout << es.eigenvalues().real()[2] << '\n' << '\n';
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
			e[i] = e[i]/(e[0]+e[1]+e[2]);
		}

		//v << es.eigenvalues().real()[0],es.eigenvalues().real()[1],es.eigenvalues().real()[2];
		e_v << e[0],e[1],e[2];
		e_value.push_back(e_v);
		//std::cout << es.eigenvectors().col(0).real() << '\n';

	}

	/*とりあえず固有値ベースの特徴7個を計算して特徴ベクトルとする*/
	std::vector<Eigen::VectorXf> feature_vectors;
	//std::vector<float> feature_vectors;
	Eigen::VectorXf feature_vector(7);
	//float feature_vector[7][1];
	float linearity;
	float planarity;
	float scattering;
	float omnivariance;
	float anisotropy;
	float eigenentropy;
	float change_of_curvature;

	for(int i=0;i<e_value.size();i++)
	{
		linearity = (e_value[i](0)-e_value[i](1))/e_value[i](0);
		planarity = (e_value[i](1)-e_value[i](2))/e_value[i](0);
		scattering = e_value[i](2)/e_value[i](0);
		//omnivariance = pow(e1*e2*e3,(1.0/3.0));
		omnivariance = cbrtf(e_value[i](0)*e_value[i](1)*e_value[i](2));//三重根を計算するやつ
		anisotropy = (e_value[i](0)-e_value[i](2))/e_value[i](0);
		eigenentropy =0;
		for (int j=0;j<3;j++)
		{
			eigenentropy -= e_value[i](j)*log(e_value[i](j));
		}
		change_of_curvature = e_value[i](2)/(e_value[i](0)+e_value[i](1)+e_value[i](2));

		std::cout << linearity << '\n';
		std::cout << planarity << '\n';
		std::cout << scattering << '\n';
		std::cout << omnivariance << '\n';
		std::cout << anisotropy << '\n';
		std::cout << eigenentropy << '\n';
		std::cout << change_of_curvature << '\n' << '\n';

		feature_vector << linearity,planarity,scattering,omnivariance,anisotropy,eigenentropy,change_of_curvature;
		feature_vectors.push_back(feature_vector);
	}

}

void ProcessingPointCloud::publish_segmented(void)
{
	source_cloud.orig_cloud = orig_cloud;
	source_cloud.vox_cloud = vox_cloud;
	source_cloud.del_cloud = del_cloud;
	source_cloud.clu_cloud = clu_cloud;
	seg_pub.publish(source_cloud);
}

/*計算した特徴ベクトルとクラスタリングした点群をトピックにあげたい、あと元の点群も送る必要がある*/
/*クラスタリングした点群はいらない*/
/*最後のクラウドとインデックスをトピックにあげて別のクラスで特徴点抽出してもいいかも*/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "processing_pointcloud");
	ProcessingPointCloud pp;
	while(ros::ok()){
		//std::cout << "0" << std::endl;
		pp.pc_queue.callOne(ros::WallDuration(1));
		pp.apply_voxelgrid();
		pp.delete_ground();
		pp.euclidean_clustering();
		pp.publish_pointcloud();
		pp.feature_extraction();
		pp.publish_segmented();
	}
	return 0;
}
