/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_move_base/network_planner.h>
#include <arti_move_base/utils.h>
#include <functional>

namespace arti_move_base
{
const char NetworkPlanner::LOGGER_NAME[] = "network_planner";

NetworkPlanner::NetworkPlanner(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer)
  : EventPipelineStage(input, output, error_cbs, success_cbs, close_to_success_cbs, "NetworkPlanner"), ErrorReceptor(0),
    planner_("arti_nav_core", "arti_nav_core::BaseNetworkPlanner"), node_handle_(node_handle),
    transformer_(std::move(transformer)), config_server_(node_handle_)
{
  config_server_.setCallback(std::bind(&NetworkPlanner::reconfigure, this, std::placeholders::_1));

  const ros::NodeHandle planner_node_handle{node_handle_, "planner"};
  const std::string planner_type = planner_node_handle.param<std::string>("type", "");

  try
  {
    planner_.loadAndInitialize(planner_type, utils::getRelativeNamespace(planner_node_handle), transformer_.get());
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("failed to load the network planner plugin: " << ex.what());
  }
}

void NetworkPlanner::cancel()
{
  current_goal_.reset();
  EventPipelineStage::cancel();
}

void NetworkPlanner::reconfigure(const NetworkPlannerConfig& config)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
  setErrorPropagationCount(config_.error_propagation_count);
}

boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints> NetworkPlanner::performTask(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& input)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  if (!planner_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "NetworkPlanner::performTask called before planner was initialized");
    callErrorCB(arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE);
    return boost::none;
  }

  current_goal_ = input;

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "construct new path for " << *current_goal_);

  if (!planner_->setGoal(*current_goal_))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "cannot set goal for network planner");
    callErrorCB(arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE);
    return boost::none;
  }

  arti_nav_core_msgs::Movement2DGoalWithConstraints result;
  result.goal.pose = input;

  const arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum planner_result
    = planner_->makePlan(result);
  if (planner_result != arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::PLAN_FOUND)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't find path to " << *current_goal_);
    callErrorCB(planner_result);
    return boost::none;
  }

  if (result.path_limits.poses.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "empty path to " << *current_goal_);
    callErrorCB(arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE);
    return boost::none;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "result of network planning: " << result);

  return result;
}

void NetworkPlanner::propagationError()
{
  callErrorCB(arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE);
}

void NetworkPlanner::handleSuccess(const boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints>& /*input*/)
{
  callSuccessCB();
}

void NetworkPlanner::handleCloseToSuccess(
  const boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints>& /*input*/)
{
  callCloseToSuccessCB();
}

void NetworkPlanner::handleError(
  const GlobalPlannerError& error, const boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints>& /*input*/)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "NetworkPlanner::handleError called without a current goal");
    return;
  }

  if (planner_)
  {
    planner_->handlePlannerError(error.error_pose_a, error.error_pose_b);
  }

  input_->set(*current_goal_);
}
}
