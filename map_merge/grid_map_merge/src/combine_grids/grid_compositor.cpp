/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <combine_grids/grid_compositor.h>

#include <opencv2/stitching/detail/util.hpp>

//#include <cloud_map_merge/OverlapArray.h>

#include <ros/assert.h>


namespace combine_grids
{
namespace internal
{
nav_msgs::OccupancyGrid::Ptr GridCompositor::compose(
    const std::vector<cv::Mat>& grids, const std::vector<cv::Rect>& rois)
{
  ROS_ASSERT(grids.size() == rois.size());

  nav_msgs::OccupancyGrid::Ptr result_grid(new nav_msgs::OccupancyGrid());

  std::vector<cv::Point> corners;
  corners.reserve(grids.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(grids.size());

  for (auto& roi : rois) {
    std::cout << "roi\n" << roi << '\n';
    corners.push_back(roi.tl());
    sizes.push_back(roi.size());
  }

  for(int i=0;i<corners.size();i++)
  {
    std::cout << "before_corner" << '\n';
    std::cout << corners[i] << '\n';
  }

  std::cout << "fix" << '\n';

  //cv::Point fix{30,0};

  //corners[1] = fix;

  for (auto& roi : rois) {
    std::cout << "roi\n" << roi << '\n';
  }

  for(int i=0;i<corners.size();i++)
  {
    std::cout << "after_corner" << '\n';
    std::cout << corners[i] << '\n';
  }

  cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

  std::cout << "fix_end" << '\n';

  /*このへんでマップの重なりを検知する*/

  std::vector<cv::Rect> arg_rois;
  arg_rois.resize(2);

  cloud_map_merge::OverlapArray overlaps;
  //std::vector<cloud_map_merge::Overlap> localOverlaps;

  //ros::Publisher pubOverlap;
  //ros::NodeHandle p;

  for (int i=0;i<rois.size()-1;i++)
  {
    arg_rois[0] = rois[i];
    arg_rois[1] = rois[i+1];
    publishOverlap(arg_rois,i,i+1,overlaps);
  }
  std::cout << "overlaps\n" << overlaps << std::endl;
  //overlaps.overlapArray = localOverlaps;
  overlaps.header.stamp = ros::Time::now();
  std::cout << "publish overlap" << std::endl;

  //これを入れないとpublish出来ないため一時的にいれてる
  //ほんとは最初のコンストラクタでpublisherを宣言するのが良い
  sleep(1);

  pubOverlap.publish(overlaps);

  result_grid->info.width = static_cast<uint>(dst_roi.width);
  result_grid->info.height = static_cast<uint>(dst_roi.height);
  result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
  // create view for opencv pointing to newly allocated grid
  cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

  for (size_t i = 0; i < grids.size(); ++i) {
    // we need to compensate global offset
    cv::Rect roi = cv::Rect(corners[i] - dst_roi.tl(), sizes[i]);
    cv::Mat result_roi(result, roi);
    // reinterpret warped matrix as signed
    // we will not change this matrix, but opencv does not support const matrices
    cv::Mat warped_signed (grids[i].size(), CV_8S, const_cast<uchar*>(grids[i].ptr()));
    // compose img into result matrix
    cv::max(result_roi, warped_signed, result_roi);
  }

  return result_grid;
}

void GridCompositor::publishOverlap(const std::vector<cv::Rect>& rois, const int& num_a, const int& num_b, cloud_map_merge::OverlapArray& overlaps)
{

  std::cout << "*********overlap********" << std::endl;

  cloud_map_merge::Overlap overlap;

  std::vector<int> map_num;
  std::vector<geometry_msgs::Point> tl;
  std::vector<geometry_msgs::Point> br;
  std::vector<geometry_msgs::Point> rect_tl;
  std::vector<geometry_msgs::Point> rect_br;

  map_num.resize(2);
  tl.resize(2);
  br.resize(2);
  rect_tl.resize(2);
  rect_br.resize(2);

  map_num[0] = num_a;
  map_num[1] = num_b;

  for(int i=0;i<2;i++)
  {
    rect_tl[i].x = rois[i].x;
    rect_tl[i].y = rois[i].y;

    rect_br[i].x = rois[i].x;
    rect_br[i].y = rois[i].y;
  }

  //レフトトップの座標値の差
  int diff_x;
  int diff_y;

  int top;
  int bottom;
  int left;
  int right;

  int over_w;
  int over_h;

  int over_wt;
  int over_ht;

  diff_x = rois[1].x - rois[0].x;
  diff_y = rois[1].y - rois[0].y;

  //二つの地図の位置関係をチェック
  if(diff_x > 0)
  {
    left = 0;
    right = 1;
  }
  else{
    left = 1;
    right = 0;
  }
  if(diff_y > 0)
  {
    top = 0;
    bottom = 1;
  }
  else
  {
    top = 1;
    bottom = 0;
  }
  
  over_w = rois[left].size().width - std::abs(diff_x);
  over_h = rois[top].size().height - std::abs(diff_x);

  if(over_w < 0)
  {
    std::cout << "x方向で重なってない" << std::endl;
    return;
  }
  if(over_w < rois[right].size().width)
  {
    over_wt = over_w;
  }
  else
  {
    over_wt = rois[right].size().width;
  }

  if(over_h < 0)
  {
    std::cout << "y方向で重なってない" << std::endl;
    return;
  }
  if(over_h < rois[bottom].size().height)
  {
    over_ht = over_h;
  }
  else
  {
    over_ht = rois[bottom].size().height;
  }

  //右側 tl:{0,?}  br:{over_wt,?}
  //下側 tl:{?,0}  br:{?,over_ht}

  tl[right].x = 0;
  br[right].x = over_wt;

  tl[bottom].y = 0;
  br[bottom].y = over_ht;

  if(right = 1)
  {
    tl[left].x = tl[right].x + diff_x;
    br[left].x = br[right].x + diff_x;
  }
  else
  {
    tl[left].x = tl[right].x - diff_x;
    br[left].x = br[right].x - diff_x;
  }

  if(bottom = 1)
  {
    tl[top].y = tl[bottom].y + diff_y;
    br[top].y = br[bottom].y + diff_y;
  }
  else
  {
    tl[top].y = tl[bottom].y - diff_y;
    br[top].y = br[bottom].y - diff_y;
  }

  overlap.header.stamp = ros::Time::now();
  overlap.overlap = true;
  overlap.resolution = 0.05;
  overlap.num = map_num;
  overlap.tl = tl;
  overlap.br = br;
  overlap.rect_tl = rect_tl;
  overlap.rect_br = rect_br;

  overlaps.overlapArray.push_back(overlap);
}

}  // namespace internal

}  // namespace combine_grids
