/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_PLANNER_BUILDER_H
#define ARTI_MOVE_BASE_PLANNER_BUILDER_H

#include <arti_async_utils/pipeline.h>
#include <arti_async_utils/pipeline_stage_base.h>
#include <arti_move_base/network_planner.h>
#include <arti_move_base/global_planner.h>
#include <arti_move_base/local_planner.h>
#include <arti_move_base/path_follower.h>
#include <arti_nav_core/transformer.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <memory>
#include <ros/node_handle.h>
#include <string>
#include <vector>

namespace arti_move_base
{
class PipelineBuilder
{
public:
  void addNetworkPlanner(
    const ros::NodeHandle& node_handle, const NetworkPlanner::ErrorCallback& error_cb,
    const NetworkPlanner::SuccessCallback& success_cb, const std::shared_ptr<arti_nav_core::Transformer>& transformer);

  void addGlobalPlanner(
    const ros::NodeHandle& node_handle, const GlobalPlanner::ErrorCallback& error_cb,
    const GlobalPlanner::SuccessCallback& success_cb, const std::shared_ptr<arti_nav_core::Transformer>& transformer,
    const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap);

  void addLocalPlanner(
    const ros::NodeHandle& node_handle, const LocalPlanner::ErrorCallback& error_cb,
    const LocalPlanner::SuccessCallback& success_cb, const std::shared_ptr<arti_nav_core::Transformer>& transformer,
    const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap);

  template<class O>
  void addPathFollower(
    const ros::NodeHandle& node_handle, const typename PathFollower<O>::ErrorCallback& error_cb,
    const typename PathFollower<O>::SuccessCallback& success_cb,
    const std::shared_ptr<arti_nav_core::Transformer>& transformer,
    const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap);

  template<class O>
  arti_async_utils::Pipeline<arti_nav_core_msgs::Pose2DStampedWithLimits, O> getPipeline();

protected:
  template<class I>
  std::shared_ptr<arti_async_utils::BufferedResource<I>> getInput();

  template<class O>
  std::shared_ptr<arti_async_utils::BufferedResource<O>> getOutput();

  template<class E, class I>
  std::function<void(const E&, const boost::optional<I>&)> getErrorCallback();

  template<class I>
  std::function<void(const boost::optional<I>&)> getSuccessCallback();

  template<class I>
  std::function<void(const boost::optional<I>&)> getCloseToSuccessCallback();

  template<class I, class O>
  arti_async_utils::Pipeline<I, O> doGetPipeline();

  template<typename T>
  static std::vector<T> makeVector(std::initializer_list<T> elements);

  std::vector<std::shared_ptr<arti_async_utils::PipelineStageBase>> pipeline_stages_;
};

}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_PLANNER_BUILDER_H
