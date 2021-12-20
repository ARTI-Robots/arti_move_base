/*
Created by abuchegger on 2020-10-16.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <arti_move_base/path_follower.h>
#include <arti_nav_core_utils/transformer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arti_move_base
{
template<class O, class E, class P>
AbstractPathFollower<O, E, P>::AbstractPathFollower(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap, const std::string& base_class_name)
  : arti_async_utils::PeriodicPipelineStage<arti_nav_core_msgs::Trajectory2DWithLimits, O, E>(input, output, error_cbs,
                                                                                              success_cbs,
                                                                                              close_to_success_cbs,
                                                                                              std::chrono::seconds(1)),
    planner_("arti_nav_core", base_class_name), node_handle_(node_handle), transformer_(std::move(transformer)),
    costmap_(std::move(costmap)), config_server_(node_handle_)
{
  config_server_.setCallback(std::bind(&AbstractPathFollower<O, E, P>::reconfigure, this, std::placeholders::_1));

  const ros::NodeHandle planner_node_handle{node_handle_, "planner"};
  const std::string planner_type = planner_node_handle.param<std::string>("type", "");

  try
  {
    planner_.loadAndInitialize(planner_type, utils::getRelativeNamespace(planner_node_handle), transformer_.get(),
                               costmap_.get());
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("failed to load the path follower plugin: " << ex.what());
  }
}

template<class O, class E, class P>
void AbstractPathFollower<O, E, P>::cancel()
{
  current_trajectory_.reset();
  arti_async_utils::PeriodicPipelineStage<arti_nav_core_msgs::Trajectory2DWithLimits, O, E>::cancel();
}

template<class O, class E, class P>
void AbstractPathFollower<O, E, P>::reconfigure(const PathFollowerConfig& config)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
  config_ = config;
  limit_utils_.emplace(config_.x_y_tolerance_increase, config_.yaw_tolerance_increase);
  this->setExecutionDuration(std::chrono::duration<double>(1.0 / std::max(config_.control_rate, 1.0)));
}

template<class O, class E, class P>
boost::optional<O> AbstractPathFollower<O, E, P>::performTask(
  const arti_nav_core_msgs::Trajectory2DWithLimits& input, bool new_input)
{
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "follow trajectory");

  if (!planner_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "PathFollower::performTask called before path follower was initialized");
    this->callErrorCB(E::NO_COMMAND_POSSIBLE);
    return boost::none;  // If the planner isn't initialized, don't output any command.
  }

  if (input.movements.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "path follower got empty trajectory as input");
    if (new_input)
    {
      this->callErrorCB(E::NO_COMMAND_POSSIBLE);
    }
    return O{};
  }

  if (new_input)
  {
    current_trajectory_ = input;
    if (!planner_->setTrajectory(*current_trajectory_))
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't set trajectory in order to follow the path");
      this->callErrorCB(E::NO_COMMAND_POSSIBLE);
      return O{};
    }
    //ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "path follower trajectory: " << *current_trajectory_);
  }

  if (planner_->isGoalReached())
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "call success callback");
    this->callSuccessCB();
    return O{};
  }
  else
  {
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());

    O command;
    const E computation_result = planner_->computeVelocityCommands(command);

    if (computation_result == E::GOAL_REACHED)
    {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "call success callback");
      this->callSuccessCB();
      return command;
    }
    else if (computation_result != E::COMMAND_FOUND)
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't calculate commands to follow path");
      this->callErrorCB(computation_result);
      return O{};
    }
    else if (checkIfClose())
    {
      // Calling the close to success callback only makes sense if the goal can be reached (i.e., a command could be
      // computed).
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "call close to success callback");
      this->callCloseToSuccessCB();
      return command;
    }
    else
    {
      return command;
    }
  }
}

template<class O, class E, class P>
bool AbstractPathFollower<O, E, P>::checkIfClose()
{
  const boost::optional<geometry_msgs::PoseStamped> global_pose = utils::getRobotPose(*this->costmap_);
  if (!global_pose)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "cannot get the robot pose to check if the robot is near the trajectory goal");
    return false;
  }

  if (!limit_utils_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "checkIfClose called before limit utils were initialized");
    return false;
  }

  if (!current_trajectory_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "checkIfClose called without a current trajectory");
    return false;
  }

  std::string tf_error_message;
  const boost::optional<geometry_msgs::PoseStamped> global_pose_tfd
    = arti_nav_core_utils::tryToTransform(*transformer_, *global_pose, current_trajectory_->header.frame_id,
                                          ros::Duration(config_.transformation_timeout), &tf_error_message);
  if (!global_pose_tfd)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "failed to transform global pose to trajectory frame: " << tf_error_message);
    return false;
  }

  arti_nav_core_msgs::Pose2DWithLimits close_to_goal_area = current_trajectory_->movements.back().pose;
  close_to_goal_area.theta.lower_limit = -config_.yaw_close_to_goal_tolerance;
  close_to_goal_area.theta.upper_limit = config_.yaw_close_to_goal_tolerance;
  close_to_goal_area.point.x.lower_limit = -config_.x_y_close_to_goal_tolerance;
  close_to_goal_area.point.x.upper_limit = config_.x_y_close_to_goal_tolerance;
  close_to_goal_area.point.y.lower_limit = -config_.x_y_close_to_goal_tolerance;
  close_to_goal_area.point.y.upper_limit = config_.x_y_close_to_goal_tolerance;

  return limit_utils_->poseWithinTolerance(global_pose_tfd->pose, close_to_goal_area);
}

PathFollower<geometry_msgs::Twist>::PathFollower(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap)
  : AbstractPathFollower(node_handle, input, output, error_cbs, success_cbs, close_to_success_cbs,
                         std::move(transformer), std::move(costmap), "arti_nav_core::BasePathFollower")
{
}

PathFollower<ackermann_msgs::AckermannDrive>::PathFollower(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap)
  : AbstractPathFollower(node_handle, input, output, error_cbs, success_cbs, close_to_success_cbs,
                         std::move(transformer), std::move(costmap), "arti_nav_core::BasePathFollowerAckermann")
{
}

}
