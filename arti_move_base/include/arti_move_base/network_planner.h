/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_NETWORK_PLANNER_H
#define ARTI_MOVE_BASE_NETWORK_PLANNER_H

#include <arti_async_utils/event_pipeline_stage.h>
#include <arti_move_base/error_receptor.h>
#include <arti_move_base/global_planner_error.h>
#include <arti_move_base/NetworkPlannerConfig.h>
#include <arti_move_base/plugin_helper.h>
#include <arti_nav_core/base_network_planner.h>
#include <arti_nav_core/base_global_planner.h>
#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <vector>

namespace arti_move_base
{
class NetworkPlanner
  : public arti_async_utils::EventPipelineStage<arti_nav_core_msgs::Pose2DStampedWithLimits,
    arti_nav_core_msgs::Movement2DGoalWithConstraints, arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum>,
    public ErrorReceptor<arti_nav_core_msgs::Movement2DGoalWithConstraints, GlobalPlannerError>
{
public:
  NetworkPlanner(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer);

  void cancel() override;

protected:
  static const char LOGGER_NAME[];

  void reconfigure(const NetworkPlannerConfig& config);

  boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints> performTask(
    const arti_nav_core_msgs::Pose2DStampedWithLimits& input) override;

  void propagationError() override;
  void handleSuccess(const boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints>& /*input*/) override;
  void handleCloseToSuccess(
    const boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints>& /*input*/) override;
  void handleError(
    const GlobalPlannerError& error,
    const boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints>& /*input*/) override;

  PluginHelper<arti_nav_core::BaseNetworkPlanner> planner_;
  ros::NodeHandle node_handle_;
  std::shared_ptr<arti_nav_core::Transformer> transformer_;
  std::mutex config_mutex_;
  dynamic_reconfigure::Server<NetworkPlannerConfig> config_server_;
  NetworkPlannerConfig config_;

  boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> current_goal_;
};
}

#endif //ARTI_MOVE_BASE_NETWORK_PLANNER_H
