/*
Created by clemens on 19.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_PATH_FOLLOWER_H
#define ARTI_MOVE_BASE_PATH_FOLLOWER_H

#include <ackermann_msgs/AckermannDrive.h>
#include <arti_async_utils/periodic_pipeline_stage.h>
#include <arti_move_base/limit_utils.h>
#include <arti_move_base/PathFollowerConfig.h>
#include <arti_move_base/plugin_helper.h>
#include <arti_move_base/utils.h>
#include <arti_nav_core/base_path_follower.h>
#include <arti_nav_core/base_path_follower_ackermann.h>
#include <arti_nav_core/transformer.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <string>
#include <vector>

namespace arti_move_base
{
template<class O, class E, class P>
class AbstractPathFollower
  : public arti_async_utils::PeriodicPipelineStage<arti_nav_core_msgs::Trajectory2DWithLimits, O, E>
{
public:
  using PeriodicPipelineStageT = arti_async_utils::PeriodicPipelineStage<
    arti_nav_core_msgs::Trajectory2DWithLimits, O, E>;
  using typename PeriodicPipelineStageT::InputBufferPtr;
  using typename PeriodicPipelineStageT::OutputBufferPtr;
  using typename PeriodicPipelineStageT::ErrorCallback;
  using typename PeriodicPipelineStageT::SuccessCallback;

  AbstractPathFollower(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap, const std::string& base_class_name);

  void cancel() override;

protected:
  static constexpr const char* const LOGGER_NAME = "path_follower";

  void reconfigure(const PathFollowerConfig& config);

  boost::optional<O> performTask(const arti_nav_core_msgs::Trajectory2DWithLimits& input, bool new_input) override;

  bool checkIfClose();

  PluginHelper<P> planner_;
  ros::NodeHandle node_handle_;
  std::shared_ptr<arti_nav_core::Transformer> transformer_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  dynamic_reconfigure::Server<PathFollowerConfig> config_server_;
  PathFollowerConfig config_;
  boost::optional<LimitUtils> limit_utils_;

  boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> current_trajectory_;
};

template<class O>
class PathFollower;

template<>
class PathFollower<geometry_msgs::Twist> : public AbstractPathFollower<geometry_msgs::Twist,
  arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum, arti_nav_core::BasePathFollower>
{
public:
  PathFollower(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap);
};

template<>
class PathFollower<ackermann_msgs::AckermannDrive> : public AbstractPathFollower<ackermann_msgs::AckermannDrive,
  arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum,
  arti_nav_core::BasePathFollowerAckermann>
{
public:
  PathFollower(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap);
};
}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_PATH_FOLLOWER_H
