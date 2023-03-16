/*
Created by clemens on 19.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_move_base/global_planner.h>
#include <arti_move_base/utils.h>
#include <arti_nav_core_utils/transformations.h>
#include <arti_nav_core_utils/transformer.h>

namespace arti_move_base
{
const char GlobalPlanner::LOGGER_NAME[] = "global_planner";

GlobalPlanner::GlobalPlanner(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs,
  std::shared_ptr<arti_nav_core::Transformer> transformer,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap)
  : EventPipelineStage(input, output, error_cbs, success_cbs, close_to_success_cbs, "GlobalPlanner"),
    ErrorReceptor(0),
    planner_("arti_nav_core", "arti_nav_core::BaseGlobalPlanner"), node_handle_(node_handle),
    transformer_(std::move(transformer)), costmap_(std::move(costmap)), config_server_(node_handle_)
{
  config_server_.setCallback(std::bind(&GlobalPlanner::reconfigure, this, std::placeholders::_1));

  const ros::NodeHandle planner_node_handle{node_handle_, "planner"};
  const std::string planner_type = planner_node_handle.param<std::string>("type", "");

  try
  {
    planner_.loadAndInitialize(planner_type, utils::getRelativeNamespace(planner_node_handle), transformer_.get(),
                               costmap_.get());
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("failed to load the global planner plugin: " << ex.what());
  }
}

void GlobalPlanner::cancel()
{
  current_goal_.reset();
  EventPipelineStage::cancel();
}

void GlobalPlanner::reconfigure(const GlobalPlannerConfig& config)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());
  config_ = config;
  limit_utils_.emplace(config_.x_y_tolerance_increase, config_.yaw_tolerance_increase);
  setErrorPropagationCount(config_.error_propagation_count);
}

boost::optional<GlobalPlannerResult> GlobalPlanner::performTask(
  const arti_nav_core_msgs::Movement2DGoalWithConstraints& input)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!planner_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::performTask called before planner was initialized");
    GlobalPlannerError planner_error;
    planner_error.error_enum = arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
    if (!input.path_limits.poses.empty())
    {
      planner_error.error_pose_a = input.path_limits.poses.front();
      planner_error.error_pose_b = input.path_limits.poses.back();
    }
    else
    {
      planner_error.error_pose_a = input.goal.pose.pose;
      planner_error.error_pose_b = input.goal.pose.pose;
    }
    callErrorCB(planner_error);
    return boost::none;
  }

  GlobalPlannerResult result;
  result.final_twist = input.goal.twist;

  current_goal_ = transformGoal(input, costmap_->getGlobalFrameID());
  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "failed to transform path given as input to global planner");
    GlobalPlannerError planner_error;
    planner_error.error_enum = arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
    if (!input.path_limits.poses.empty())
    {
      planner_error.error_pose_a = input.path_limits.poses.front();
      planner_error.error_pose_b = input.path_limits.poses.back();
    }
    else
    {
      planner_error.error_pose_a = input.goal.pose.pose;
      planner_error.error_pose_b = input.goal.pose.pose;
    }
    callErrorCB(planner_error);
    return boost::none;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "global planner input: " << input);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "global planner current_goal: " << *current_goal_);

  arti_nav_core_msgs::Movement2DGoalWithConstraints current_planner_input;
  if (current_goal_->path_limits.poses.empty())
  {
    // only goal given drive directly to it
    current_planner_input = *current_goal_;
  }
  else
  {
    current_planner_input.path_limits.header = current_goal_->path_limits.header;
    current_planner_input.path_limits.poses.reserve(current_goal_->path_limits.poses.size());

    double accumulated_path_length = 0.;
    auto last_pose = current_goal_->path_limits.poses.front();
    for (const auto& pose: current_goal_->path_limits.poses)
    {
      accumulated_path_length += std::hypot(pose.point.x.value - last_pose.point.x.value,
                                            pose.point.y.value - last_pose.point.y.value);
      if (accumulated_path_length <= config_.lookahead)
      {
        current_planner_input.path_limits.poses.push_back(pose);
      }
      else
      {
        break;
      }

      last_pose = pose;
    }

    if (current_goal_->path_limits.poses.size() == current_planner_input.path_limits.poses.size())
    {
      current_planner_input.goal = current_goal_->goal;
    }
    else
    {
      current_planner_input.goal.pose.pose = current_planner_input.path_limits.poses.back();
      current_planner_input.goal.pose.header = current_planner_input.path_limits.header;

      // we do not have limits for intermediate goals
      current_planner_input.goal.twist.theta.value = std::numeric_limits<double>::infinity();
      current_planner_input.goal.twist.x.value = std::numeric_limits<double>::infinity();
      current_planner_input.goal.twist.y.value = std::numeric_limits<double>::infinity();

      // we do not have limits for intermediate goals
      result.final_twist.theta.value = std::numeric_limits<double>::infinity();
      result.final_twist.x.value = std::numeric_limits<double>::infinity();
      result.final_twist.y.value = std::numeric_limits<double>::infinity();
    }
  }
  current_planner_input.goal.pose.header.stamp = ros::Time::now();
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "set global plan: " << current_planner_input);

  if (!planner_->setGoal(current_planner_input.goal.pose, current_planner_input.path_limits))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "cannot set goal for global planner");
    GlobalPlannerError planner_error;
    planner_error.error_enum = arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
    if (!current_goal_->path_limits.poses.empty())
    {
      planner_error.error_pose_a = current_goal_->path_limits.poses.front();
      planner_error.error_pose_b = current_goal_->path_limits.poses.back();
    }
    else
    {
      planner_error.error_pose_a = current_goal_->goal.pose.pose;
      planner_error.error_pose_b = current_goal_->goal.pose.pose;
    }
    callErrorCB(planner_error);
    return boost::none;
  }

  const arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum planner_result = planner_->makePlan(result.plan);
  if (planner_result != arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::PLAN_FOUND)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can't find path to " << current_planner_input);
    GlobalPlannerError planner_error;
    planner_error.error_enum = planner_result;
    if (!current_goal_->path_limits.poses.empty())
    {
      planner_error.error_pose_a = current_goal_->path_limits.poses.front();
      planner_error.error_pose_b = current_goal_->path_limits.poses.back();
    }
    else
    {
      planner_error.error_pose_a = current_goal_->goal.pose.pose;
      planner_error.error_pose_b = current_goal_->goal.pose.pose;
    }
    callErrorCB(planner_error);
    return boost::none;
  }

  if (result.plan.poses.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "empty path to " << current_planner_input);
    GlobalPlannerError planner_error;
    planner_error.error_enum = arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;
    if (!current_goal_->path_limits.poses.empty())
    {
      planner_error.error_pose_a = current_goal_->path_limits.poses.front();
      planner_error.error_pose_b = current_goal_->path_limits.poses.back();
    }
    else
    {
      planner_error.error_pose_a = current_goal_->goal.pose.pose;
      planner_error.error_pose_b = current_goal_->goal.pose.pose;
    }
    callErrorCB(planner_error);
    return boost::none;
  }

  if (!result.plan.header.stamp.isZero())
  {
    result.plan.header.stamp = current_planner_input.goal.pose.header.stamp;
  }
  if (result.plan.header.frame_id.empty())
  {
    result.plan.header.frame_id = costmap_->getGlobalFrameID();
  }

  return result;
}

void GlobalPlanner::propagationError()
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  GlobalPlannerError planner_error;
  planner_error.error_enum = arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::NO_PATH_POSSIBLE;

  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::propagationError: got error without a current goal");
  }
  else if (!current_goal_->path_limits.poses.empty() && limit_utils_)
  {
    const boost::optional<geometry_msgs::PoseStamped> robot_pose = utils::getRobotPose(*costmap_);
    const boost::optional<size_t> index
      = robot_pose ? utils::findPose(*limit_utils_, current_goal_->path_limits.poses, robot_pose->pose.position.x,
                                     robot_pose->pose.position.y, tf::getYaw(robot_pose->pose.orientation))
                   : boost::none;
    if (index)
    {
      planner_error.error_pose_a = current_goal_->path_limits.poses[*index];
      planner_error.error_pose_b
        = current_goal_->path_limits.poses[std::min(*index + 1, current_goal_->path_limits.poses.size() - 1)];
    }
    else
    {
      planner_error.error_pose_a = current_goal_->path_limits.poses.front();
      planner_error.error_pose_b = current_goal_->path_limits.poses.back();
    }
  }
  else
  {
    planner_error.error_pose_a = current_goal_->goal.pose.pose;
    planner_error.error_pose_b = current_goal_->goal.pose.pose;
  }

  callErrorCB(planner_error);
}

void GlobalPlanner::handleSuccess(const boost::optional<GlobalPlannerResult>& input)
{
  doHandleSuccess(true, input);
}

void GlobalPlanner::handleCloseToSuccess(const boost::optional<GlobalPlannerResult>& input)
{
  doHandleSuccess(false, input);
}

void GlobalPlanner::doHandleSuccess(bool success, const boost::optional<GlobalPlannerResult>& input)
{
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess 1");

  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess 2");

  if (!input || input->plan.poses.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess called without input");
    return;
  }

  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess called without a current goal");
    return;
  }

  if (!limit_utils_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess called before limit utils were initialized");
    input_->set(*current_goal_);
    return;
  }

  arti_nav_core_msgs::Pose2DWithLimits trajectory_goal = input->plan.poses.back();
  if (input->plan.header.frame_id != current_goal_->path_limits.header.frame_id)
  {
    arti_nav_core_msgs::Pose2DStampedWithLimits trajectory_goal_stamped;
    trajectory_goal_stamped.header = input->plan.header;
    trajectory_goal_stamped.pose = trajectory_goal;
    std::string tf_error_message;
    const auto trajectory_goal_stamped_tfd
      = arti_nav_core_utils::tryToTransform(*transformer_, trajectory_goal_stamped,
                                            current_goal_->path_limits.header.frame_id,
                                            ros::Duration(config_.transformation_timeout), &tf_error_message);
    if (trajectory_goal_stamped_tfd)
    {
      trajectory_goal = trajectory_goal_stamped_tfd->pose;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME,
                             "GlobalPlanner::doHandleSuccess: failed to transform trajectory goal: "
                               << tf_error_message);
      input_->set(*current_goal_);
      return;
    }
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess 3");

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess"
    << " success: " << success << ", trajectory frame: " << input->plan.header.frame_id);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess trajectory_goal:\n" << trajectory_goal);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess current_goal_.goal:\n" << current_goal_->goal);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess current_goal_ path limits frame: "
    << current_goal_->path_limits.header.frame_id);
  // if we the path follower has the goal of the current plan we can notify the higher instance that we reached the goal
  if (limit_utils_->poseWithinTolerance(trajectory_goal.point.x.value, trajectory_goal.point.y.value,
                                        trajectory_goal.theta.value, current_goal_->goal.pose.pose))
  {
    if (success)
    {
      output_->clear();
      callSuccessCB();
    }
    else
    {
      callCloseToSuccessCB();
    }
    return;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess 4");
  // search pose in current path which correspond to to the pose used for the path follower as a goal
  // this goal may be not the same as the input as only a local look-a-head is used for the path follower
  const boost::optional<size_t> index
    = utils::findPose(*limit_utils_, current_goal_->path_limits.poses, trajectory_goal.point.x.value,
                      trajectory_goal.point.y.value, trajectory_goal.theta.value);
  if (!index)
  {
    // This usually happens when there are no path limits (because the network planner isn't used).
    if (!current_goal_->path_limits.poses.empty())
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess could not find trajectory goal in current "
                                          "path; trying to redo planning");
    }
    input_->set(*current_goal_);
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess index: " << *index);

    // Else, the path needs to be shortened for the traveled part (except for overlap) and a plan needs to be made for
    // the remaining part:
    size_t next_start_index = *index;
    double overlap = 0.;
    for (; next_start_index != 0 && overlap < config_.overlap; --next_start_index)
    {
      const auto& point = current_goal_->path_limits.poses[next_start_index].point;
      const auto& prev_point = current_goal_->path_limits.poses[next_start_index - 1].point;
      overlap += std::hypot(point.x.value - prev_point.x.value, point.y.value - prev_point.y.value);
    }

    current_goal_->path_limits.poses.erase(current_goal_->path_limits.poses.begin(),
                                           current_goal_->path_limits.poses.begin() + next_start_index);
    input_->set(*current_goal_);
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::doHandleSuccess SET INPUT NEW: " << next_start_index);
  }
}

boost::optional<arti_nav_core_msgs::Movement2DGoalWithConstraints> GlobalPlanner::transformGoal(
  const arti_nav_core_msgs::Movement2DGoalWithConstraints& movement_goal, const std::string& target_frame)
{
  std::string error_message;
  const boost::optional<geometry_msgs::TransformStamped> goal_transform_msg
    = arti_nav_core_utils::tryToLookupTransform(*transformer_, target_frame, movement_goal.goal.pose.header.frame_id,
                                                movement_goal.goal.pose.header.stamp,
                                                ros::Duration(config_.transformation_timeout), &error_message);
  if (!goal_transform_msg)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "global planner cannot transform movement goal goal: " << error_message);
    return boost::none;
  }

  const boost::optional<geometry_msgs::TransformStamped> path_limits_transform_msg
    = arti_nav_core_utils::tryToLookupTransform(*transformer_, target_frame, movement_goal.path_limits.header.frame_id,
                                                movement_goal.path_limits.header.stamp,
                                                ros::Duration(config_.transformation_timeout), &error_message);
  if (!path_limits_transform_msg)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "global planner cannot transform movement goal path limits: " << error_message);
    return boost::none;
  }

  return arti_nav_core_utils::transformMovementGoal(movement_goal, *goal_transform_msg, *path_limits_transform_msg);
}

void GlobalPlanner::handleError(const LocalPlannerError& error, const boost::optional<GlobalPlannerResult>& /*input*/)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::handleError called without a current goal");
    return;
  }

  if (planner_)
  {
    planner_->handlePlannerError(error.error_pose_a, error.error_pose_b);
  }

  input_->set(*current_goal_);
}

void GlobalPlanner::handleError(
  const arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum&/*error*/,
  const boost::optional<GlobalPlannerResult>& /*input*/)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::handleError called without a current goal");
    return;
  }

  input_->set(*current_goal_);
}

void GlobalPlanner::handleError(
  const arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum&/*error*/,
  const boost::optional<GlobalPlannerResult>& /*input*/)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!current_goal_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "GlobalPlanner::handleError called without a current goal");
    return;
  }

  input_->set(*current_goal_);
}

}  // namespace arti_move_base
