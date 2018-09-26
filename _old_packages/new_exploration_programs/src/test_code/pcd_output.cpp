/*受け取った点群をpcd出力する*/

#include <new_exploration_programs/feature_matching.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcd_output");
	FeatureMatching fm;

	/*ソースだけ受け取る*/

	while(ros::ok())
  {
		fm.sc_queue.callOne(ros::WallDuration(1));

    if(fm.input_source)
    {
      /*ソース点群を出力する*/
      fm.output_fullSource();//ソースマップの全体を出力
      fm.output_segSource();//ソースマップのクラスタごとに出力//インデックスから抽出する

      fm.input_source = false;

    }
    else
    {
      std::cout << "source_is_nothing" << '\n';
    }
  }

	return 0;
}
