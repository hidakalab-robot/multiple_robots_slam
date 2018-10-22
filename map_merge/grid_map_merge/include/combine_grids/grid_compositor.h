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

#ifndef GRID_COMPOSITOR_H_
#define GRID_COMPOSITOR_H_

#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/utility.hpp>

#include <cloud_map_merge/OverlapArray.h>

#include <ros/ros.h>

namespace combine_grids
{
namespace internal
{
class GridCompositor
{
private:
  ros::Publisher pubOverlap;
  ros::NodeHandle p;
public:
  GridCompositor(){
    std::cout << "コンストラクタ" << std::endl;
    pubOverlap = p.advertise<cloud_map_merge::OverlapArray>("grid_map_merge/overlap", 10);};
  ~GridCompositor(){};
  nav_msgs::OccupancyGrid::Ptr compose(const std::vector<cv::Mat>& grids,
                                       const std::vector<cv::Rect>& rois, const std::vector<nav_msgs::OccupancyGrid::ConstPtr>& grids_, const std::vector<int>& mapOrder);
  void publishOverlap(const std::vector<cv::Rect>& rois, const std::vector<nav_msgs::OccupancyGrid>& grids_, const int& num_a, const int& num_b, cloud_map_merge::OverlapArray& overlaps);
};

}  // namespace internal

}  // namespace combine_grids

#endif  // GRID_COMPOSITOR_H_
