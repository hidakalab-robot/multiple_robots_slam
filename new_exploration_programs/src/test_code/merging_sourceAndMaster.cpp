/*これはrtabmapで作成したローカルマップをクラウド上のマップと合成するノードです*/
#include <new_exploration_programs/centroid_matching.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merging_sourceAndMaster");
	CentroidMatching cm;

  while(ros::ok())
  {
    cm.mi_queue.callOne(ros::WallDuration(1));
    if(cm.input_info)
    {
      if(cm.is_merged_empty())
      {
        /*マッチングリストが空でもマージドが空だとは限らないのでは？？*/
        //cm.nan_check();
        cm.merging_cloud();
        //cm.nan_check();
        //cm.publish_mergedcloud();
        cm.publish_mergedRtab();
      }
      else
      {
        if(!cm.no_matching)
        {
          //cm.icp4allcluster();
          cm.nonicp_estimate();
          cm.merging_cloud();//点群を合成する
          //cm.publish_mergedcloud();//合成した点群を出力
          cm.voxelize();
          cm.publish_mergedRtab();
        }
        else
        {
          /*mergedがnot_emptyでno_matchingときでもそのまま出力する必要がある*/
          //cm.publish_mergedcloud();
          cm.publish_mergedRtab();
        }
      }

    }
    else
    {
      std::cout << "not_input_matchinginfo" << '\n';
    }
    cm.input_info =false;
    cm.one_matching = false;
    cm.no_matching = false;
  }


  return 0;
}
