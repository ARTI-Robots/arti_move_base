/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_LOCAL_PLANNER_H
#define ARTI_MOVE_BASE_LOCAL_PLANNER_H

#include <arti_async_utils/event_pipeline_stage.h>
#include <arti_move_base/error_receptor.h>
#include <arti_move_base/global_planner_result.h>
#include <arti_move_base/limit_utils.h>
#include <arti_move_base/local_planner_error.h>
#include <arti_move_base/LocalPlannerConfig.h>
#include <arti_move_base/plugin_helper.h>
#include <arti_nav_core/base_local_planner.h>
#include <arti_nav_core/base_path_follower.h>
#include <arti_nav_core/base_path_follower_ackermann.h>
#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/common.h>
#include <ros/node_handle.h>
#include <string>
#include <tug_profiling/profiler.h>
#include <tug_profiling/statistics_printer.h>

namespace arti_move_base
{
class LocalPlanner
  : public arti_async_utils::EventPipelineStage<GlobalPlannerResult, arti_nav_core_msgs::Trajectory2DWithLimits,
    LocalPlannerError>,
    public ErrorReceptor<arti_nav_core_msgs::Trajectory2DWithLimits,
      arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum,
      arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum>
{
public:
  LocalPlanner(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap);

  void cancel() override;

protected:
  static const char LOGGER_NAME[];

  void reconfigure(const LocalPlannerConfig& config);

  boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> performTask(const GlobalPlannerResult& input) override;

  void propagationError() override;
  void handleSuccess(const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& input) override;
  void handleCloseToSuccess(const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& input) override;

  void doHandleSuccess(bool success, const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& input);

  void handleError(
    const arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum&/*error*/,
    const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& /*input*/) override;
  void handleError(
    const arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum&/*error*/,
    const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& /*input*/) override;

  boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> transformTrajectory(
    const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory);

  boost::optional<GlobalPlannerResult> transformGlobalPlannerResult(const GlobalPlannerResult& global_planner_result);

  PluginHelper<arti_nav_core::BaseLocalPlanner> planner_;
  ros::NodeHandle node_handle_;
  std::shared_ptr<arti_nav_core::Transformer> transformer_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  dynamic_reconfigure::Server<LocalPlannerConfig> config_server_;
  LocalPlannerConfig config_;
  boost::optional<LimitUtils> limit_utils_;

  boost::optional<GlobalPlannerResult> current_path_;

  tug_profiling::Profiler profiler_;
  tug_profiling::StatisticsPrinter statistic_printer_;
  bool checkTrajectoryValid(arti_nav_core_msgs::Trajectory2DWithLimits& limits);
};
}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_LOCAL_PLANNER_H
