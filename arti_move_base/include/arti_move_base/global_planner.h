/*
Created by clemens on 19.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_GLOBAL_PLANNER_H
#define ARTI_MOVE_BASE_GLOBAL_PLANNER_H

#include <arti_async_utils/event_pipeline_stage.h>
#include <arti_move_base/error_receptor.h>
#include <arti_move_base/global_planner_error.h>
#include <arti_move_base/global_planner_result.h>
#include <arti_move_base/GlobalPlannerConfig.h>
#include <arti_move_base/local_planner.h>
#include <arti_move_base/plugin_helper.h>
#include <arti_nav_core/base_path_follower.h>
#include <arti_nav_core/base_path_follower_ackermann.h>
#include <arti_nav_core/base_global_planner.h>
#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/common.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>

namespace arti_move_base
{
class GlobalPlanner
  : public arti_async_utils::EventPipelineStage<arti_nav_core_msgs::Movement2DGoalWithConstraints,
    GlobalPlannerResult, GlobalPlannerError>,
    public ErrorReceptor<GlobalPlannerResult, LocalPlannerError,
      arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum,
      arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum>
{
public:
  GlobalPlanner(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs,
    std::shared_ptr<arti_nav_core::Transformer> transformer,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap);

  void cancel() override;

protected:
  static const char LOGGER_NAME[];

  void reconfigure(const GlobalPlannerConfig& config);

  boost::optional<GlobalPlannerResult> performTask(
    const arti_nav_core_msgs::Movement2DGoalWithConstraints& input) override;

  void propagationError() override;
  void handleSuccess(const boost::optional<GlobalPlannerResult>& input) override;
  void handleCloseToSuccess(const boost::optional<GlobalPlannerResult>& input) override;
  void handleError(const LocalPlannerError& error, const boost::optional<GlobalPlannerResult>& /*input*/) override;
  void handleError(
    const arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum&/*error*/,
    const boost::optional<GlobalPlannerResult>& /*input*/) override;
  void handleError(
    const arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum&/*error*/,
    const boost::optional<GlobalPlannerResult>& /*input*/) override;

  void doHandleSuccess(bool success, const boost::optional<GlobalPlannerResult>& input);

  boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints> transformGoal(
    const arti_nav_core_msgs::Movement2DGoalWithConstraints& movement_goal, const std::string& target_frame);

  PluginHelper<arti_nav_core::BaseGlobalPlanner> planner_;
  ros::NodeHandle node_handle_;
  std::shared_ptr<arti_nav_core::Transformer> transformer_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  dynamic_reconfigure::Server<GlobalPlannerConfig> config_server_;
  GlobalPlannerConfig config_;
  boost::optional<LimitUtils> limit_utils_;

  boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints> current_goal_;
};
}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_GLOBAL_PLANNER_H
