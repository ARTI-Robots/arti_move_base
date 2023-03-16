/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_move_base/local_planner.h>
#include <arti_move_base/utils.h>
#include <arti_nav_core_utils/transformations.h>
#include <arti_nav_core_utils/transformer.h>
#include <ros/console.h>
#include <arti_profiling/duration_measurement.h>
#include <thread>
#include <chrono>
#include <arti_ros_param/arti_ros_param.h>
#include <arti_ros_param/collections.h>

namespace arti_move_base
{
const char LocalPlanner::LOGGER_NAME[] = "local_planner";

LocalPlanner::LocalPlanner(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs, std::shared_ptr<arti_nav_core::Transformer> transformer,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap)
  : EventPipelineStage(input, output, error_cbs, success_cbs, close_to_success_cbs, "LocalPlanner"),
    ErrorReceptor(0),
    planner_("arti_nav_core", "arti_nav_core::BaseLocalPlanner"), node_handle_(node_handle),
    transformer_(std::move(transformer)), costmap_(std::move(costmap)), config_server_(node_handle_),
    profiler_("local_planner"), statistic_printer_(profiler_, ros::NodeHandle(node_handle_, "profiling"))
{
  config_server_.setCallback(std::bind(&LocalPlanner::reconfigure, this, std::placeholders::_1));

  const ros::NodeHandle planner_node_handle{node_handle_, "planner"};
  const std::string planner_type = planner_node_handle.param<std::string>("type", "");

  try
  {
    planner_.loadAndInitialize(planner_type, utils::getRelativeNamespace(planner_node_handle), transformer_.get(),
                               costmap_.get());
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("failed to load the local planner plugin: " << ex.what());
  }

  if (node_handle_.hasParam("post_processing"))
  {
    std::vector<XmlRpc::XmlRpcValue> planners_raw = arti_ros_param::getRequiredParam<std::vector<XmlRpc::XmlRpcValue>>(
      node_handle_, "post_processing");

    ROS_INFO_STREAM("post processing of local planner");
    for (const auto& planner_info_raw: planners_raw)
    {
      std::string plugin_type = arti_ros_param::getRequiredParam<std::string>(planner_info_raw, "plugin_type");
      std::string plugin_name = arti_ros_param::getRequiredParam<std::string>(planner_info_raw, "plugin_name");

      const ros::NodeHandle post_processing_node_handle{node_handle_, plugin_name};

      ROS_INFO_STREAM("plugin_type: " << plugin_type << " plugin_name: " << plugin_name);

      post_processing_steps_.emplace_back("arti_nav_core", "arti_nav_core::BaseLocalPlannerPostProcessing");
      post_processing_steps_.back().loadAndInitialize(plugin_type,
                                                      utils::getRelativeNamespace(post_processing_node_handle),
                                                      transformer_.get(),
                                                      costmap_.get());
    }
  }
}

void LocalPlanner::cancel()
{
  current_path_.reset();
  planner_->resetPlanner();
  EventPipelineStage::cancel();
}

void LocalPlanner::reconfigure(const LocalPlannerConfig& config)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());
  config_ = config;
  limit_utils_.emplace(config_.x_y_tolerance_increase, config_.yaw_tolerance_increase);
  setErrorPropagationCount(config_.error_propagation_count);
}

boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> LocalPlanner::performTask(const GlobalPlannerResult& input)
{
  //ROS_ERROR_STREAM("LocalPlanner::performTask::input: " << input.plan);

  arti_profiling::DurationMeasurement measurement(profiler_, "complete function");

  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!planner_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::performTask called before planner was initialized");
    LocalPlannerError planner_error;
    planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
    callErrorCB(planner_error);
    return boost::none;
  }

  if (input.plan.poses.empty())
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "no planning needed");
    callSuccessCB();
    return boost::none;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "construct new trajectory");

  current_path_ = transformGlobalPlannerResult(input);
  if (!current_path_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "local planner input could not be transformed");
    LocalPlannerError planner_error;
    planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
    callErrorCB(planner_error);
    return boost::none;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "local planner input: " << current_path_->plan);

  GlobalPlannerResult current_planner_input;
  current_planner_input.plan.header = current_path_->plan.header;
  current_planner_input.plan.header.stamp = ros::Time::now();
  current_planner_input.plan.poses.reserve(current_path_->plan.poses.size());

  double accumulated_path_length = 0.;
  auto last_pose = current_path_->plan.poses.front();
  for (const auto& pose: current_path_->plan.poses)
  {
    accumulated_path_length += std::hypot(pose.point.x.value - last_pose.point.x.value,
                                          pose.point.y.value - last_pose.point.y.value);
    if (accumulated_path_length <= config_.lookahead)
    {
      current_planner_input.plan.poses.push_back(pose);
    }
    else
    {
      break;
    }

    last_pose = pose;
  }

  if (current_path_->plan.poses.size() == current_planner_input.plan.poses.size())
  {
    current_planner_input.final_twist = current_path_->final_twist;
  }
  else
  {
    // we do not have limits for intermediate goals
    current_planner_input.final_twist.theta.value = std::numeric_limits<double>::infinity();
    current_planner_input.final_twist.x.value = std::numeric_limits<double>::infinity();
    current_planner_input.final_twist.y.value = std::numeric_limits<double>::infinity();
  }

  if (!planner_->setPlan(current_planner_input.plan))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can not set plan for local planner");
    LocalPlannerError planner_error;
    planner_error.error_pose_a = current_planner_input.plan.poses.front();
    planner_error.error_pose_b = current_planner_input.plan.poses.back();
    planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;

    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "sleep for " << config_.delay_s_after_invalid_trajectory << " s.");
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.delay_s_after_invalid_trajectory));

    callErrorCB(planner_error);
    return boost::none;
  }

  if (!planner_->setFinalVelocityConstraints(current_planner_input.final_twist))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "can not set final velocity constraints");
    LocalPlannerError planner_error;
    planner_error.error_pose_a = current_planner_input.plan.poses.front();
    planner_error.error_pose_b = current_planner_input.plan.poses.back();
    planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;

    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "sleep for " << config_.delay_s_after_invalid_trajectory << " s.");
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.delay_s_after_invalid_trajectory));

    callErrorCB(planner_error);
    return boost::none;
  }

  arti_nav_core_msgs::Trajectory2DWithLimits trajectory;
  arti_profiling::DurationMeasurement measurement_makeTrajectory(profiler_, "makeTrajectory");
  const arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum planner_result
    = planner_->makeTrajectory(trajectory);
  measurement_makeTrajectory.stop();

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "local planner trajectory: " << trajectory);

  if (planner_result == arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::GOAL_REACHED)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "goal reached locally");
    callSuccessCB();
    return boost::none;
  }

  if (planner_result != arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::TRAJECTORY_FOUND)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "local planner cannot produce a trajectory");
    LocalPlannerError planner_error;
    planner_error.error_pose_a = current_planner_input.plan.poses.front();
    planner_error.error_pose_b = current_planner_input.plan.poses.back();
    planner_error.error_enum = planner_result;

    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "sleep for " << config_.delay_s_after_invalid_trajectory << " s.");
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.delay_s_after_invalid_trajectory));

    callErrorCB(planner_error);
    return boost::none;
  }

  for (const auto& post_processing_step: post_processing_steps_)
  {
    arti_nav_core_msgs::Trajectory2DWithLimits new_trajectory;
    arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum post_processing_result = post_processing_step->makeTrajectory(
      trajectory, current_planner_input.final_twist, new_trajectory);

    if (post_processing_result
        != arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED)
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "local planner post processing cannot produce a trajectory");
      LocalPlannerError planner_error;
      planner_error.error_pose_a = current_planner_input.plan.poses.front();
      planner_error.error_pose_b = current_planner_input.plan.poses.back();
      switch (post_processing_result)
      {
        case arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::FAR_FROM_PATH:
          planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::FAR_FROM_PATH;
          break;
        case arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::OBSTACLE_TO_CLOSE:
          planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::OBSTACLE_TO_CLOSE;
          break;
        case arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::POST_PROCESSING_NOT_POSSIBLE:
          planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
          break;
        case arti_nav_core::BaseLocalPlannerPostProcessing::BaseLocalPlannerPostProcessingErrorEnum::TRAJECTORY_POST_PROCESSED:
          //nothing to do as this would be the good case
          break;
      }
      planner_error.error_enum = planner_result;

      ROS_WARN_STREAM_NAMED(LOGGER_NAME, "sleep for " << config_.delay_s_after_invalid_trajectory << " s.");
      std::this_thread::sleep_for(std::chrono::duration<double>(config_.delay_s_after_invalid_trajectory));

      callErrorCB(planner_error);
      return boost::none;
    }

    trajectory = new_trajectory;
  }

  arti_profiling::DurationMeasurement measurement_transformTrajectory(profiler_, "transformTrajectory");
  boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> result = transformTrajectory(trajectory);
  measurement_transformTrajectory.stop();
  if (result)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "local planner transformed trajectory: " << *result);
  }

  // CHECK VALIDITY OF LOCAL PLANNER'S TRAJECTORY
  // Return - no trajectory possible if teb planner produces an invalid trajectory
  if (config_.enable_trajectory_validation && !checkTrajectoryValid(trajectory))
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "trajectory from local planner determined to be invalid");
    LocalPlannerError planner_error;
    planner_error.error_pose_a = current_planner_input.plan.poses.front();
    planner_error.error_pose_b = current_planner_input.plan.poses.back();
    planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;

    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "sleep for " << config_.delay_s_after_invalid_trajectory << " s.");
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.delay_s_after_invalid_trajectory));

    callErrorCB(planner_error);
    return boost::none;
  }

  return result;
}

void LocalPlanner::propagationError()
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  LocalPlannerError planner_error;
  planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;

  const boost::optional<geometry_msgs::PoseStamped> robot_pose = utils::getRobotPose(*costmap_);

  if (!current_path_ || current_path_->plan.poses.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::propagationError called without a current path");
  }
  else if (robot_pose && limit_utils_)
  {
    const boost::optional<size_t> index
      = utils::findPose(*limit_utils_, current_path_->plan.poses, robot_pose->pose.position.x,
                        robot_pose->pose.position.y, tf::getYaw(robot_pose->pose.orientation));
    if (index)
    {
      planner_error.error_pose_a = current_path_->plan.poses[*index];
      planner_error.error_pose_b
        = current_path_->plan.poses[std::min(*index + 1, current_path_->plan.poses.size() - 1)];
    }
    else
    {
      planner_error.error_pose_a = current_path_->plan.poses.front();
      planner_error.error_pose_b = current_path_->plan.poses.back();
    }
  }
  else
  {
    planner_error.error_pose_a = current_path_->plan.poses.front();
    planner_error.error_pose_b = current_path_->plan.poses.back();
  }

  callErrorCB(planner_error);
}

void LocalPlanner::handleSuccess(const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& input)
{
  doHandleSuccess(true, input);
}

void LocalPlanner::handleCloseToSuccess(const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& input)
{
  doHandleSuccess(false, input);
}

void LocalPlanner::doHandleSuccess(
  const bool success, const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& input)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!current_path_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess called without a current path");
    return;
  }

  if (!limit_utils_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess called before limit utils were initialized");
    input_->set(*current_path_);
    return;
  }

  if (!input || input->movements.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess called without input; trying to redo planning");
    input_->set(*current_path_);
    return;
  }

  if (current_path_->plan.poses.size() >= 2)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess current_path.plan: "
      << "poses[0]:\n" << current_path_->plan.poses.front()
      << "...\n"
      << "poses[" << (current_path_->plan.poses.size() - 1) << "]:\n" << current_path_->plan.poses.back());
  }
  else if (!current_path_->plan.poses.empty())
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess current_path.plan: "
      << "only pose:\n" << current_path_->plan.poses.front());
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess current_path.plan: empty");
  }

  arti_nav_core_msgs::Pose2DWithLimits trajectory_goal = input->movements.back().pose;
  if (input->header.frame_id != current_path_->plan.header.frame_id)
  {
    arti_nav_core_msgs::Pose2DStampedWithLimits trajectory_goal_stamped;
    trajectory_goal_stamped.header = input->header;
    trajectory_goal_stamped.pose = trajectory_goal;
    std::string tf_error_message;
    const auto trajectory_goal_stamped_tfd
      = arti_nav_core_utils::tryToTransform(*transformer_, trajectory_goal_stamped, current_path_->plan.header.frame_id,
                                            ros::Duration(config_.transformation_timeout), &tf_error_message);
    if (trajectory_goal_stamped_tfd)
    {
      trajectory_goal = trajectory_goal_stamped_tfd->pose;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME,
                             "LocalPlanner::doHandleSuccess: failed to transform trajectory goal: "
                               << tf_error_message);
      input_->set(*current_path_);
      return;
    }
  }
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess: "
    << "success: " << success << ", trajectory_goal:\n" << trajectory_goal);

  // search pose in current path which correspond to to the pose used for the path follower as a goal
  // this goal may be not the same as the input as only a local look-a-head is used for the path follower
  const boost::optional<size_t> index
    = utils::findPose(*limit_utils_, current_path_->plan.poses, trajectory_goal.point.x.value,
                      trajectory_goal.point.y.value, trajectory_goal.theta.value);
  if (!index)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess could not find path follower goal in planned "
                                        "path; trying to redo planning");
    input_->set(*current_path_);
    return;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess index: " << *index << " of: "
                                                                              << current_path_->plan.poses.size());

  const bool trajectory_to_goal = (*index == (current_path_->plan.poses.size() - 1));
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess trajectory_to_goal: " << trajectory_to_goal);

  // If the path follower has reached the goal of the current plan, we can notify the higher-level planner that we
  // reached the goal:
  if (trajectory_to_goal)
  {
    if (success)
    {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess reached goal");
      output_->clear();
      callSuccessCB();
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess close to goal");
      callCloseToSuccessCB();
    }
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::doHandleSuccess process new path segment");

    // Else, the path needs to be shortened for the traveled part (except for overlap) and a plan needs to be made for
    // the remaining part:
    size_t next_start_index = *index;
    double overlap = 0.;
    for (; next_start_index != 0 && overlap < config_.overlap; --next_start_index)
    {
      const auto& point = current_path_->plan.poses[next_start_index].point;
      const auto& prev_point = current_path_->plan.poses[next_start_index - 1].point;
      overlap += std::hypot(point.x.value - prev_point.x.value, point.y.value - prev_point.y.value);
    }

    current_path_->plan.poses.erase(current_path_->plan.poses.begin(),
                                    current_path_->plan.poses.begin() + next_start_index);
    input_->set(*current_path_);
  }
}

void LocalPlanner::handleError(
  const arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum&/*error*/,
  const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& /*input*/)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!current_path_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::handleError called without a current path");
    return;
  }

  input_->set(*current_path_);
}

void LocalPlanner::handleError(
  const arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum&/*error*/,
  const boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& /*input*/)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getCostmap()->getMutex());

  if (!current_path_)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::handleError called without a current path");
    return;
  }

  input_->set(*current_path_);
}

boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> LocalPlanner::transformTrajectory(
  const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  const std::string frame_id = costmap_->getGlobalFrameID();
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformTrajectory: frame_id: " << frame_id);

  if (trajectory.header.frame_id == frame_id)
  {
    return trajectory;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformTrajectory: 2");

  ros::Time stamp = trajectory.header.stamp;
  if (!stamp.isZero())
  {
    stamp = ros::Time::now();
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformTrajectory: stamp: " << stamp);

  arti_profiling::DurationMeasurement measurement_getTransform(profiler_, "getTransform");
  std::string error_message;
  const boost::optional<geometry_msgs::TransformStamped> transform_msg
    = arti_nav_core_utils::tryToLookupTransform(*transformer_, frame_id, trajectory.header.frame_id, stamp,
                                                ros::Duration(config_.transformation_timeout), &error_message);
  if (!transform_msg)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "local planner cannot transform trajectory: " << error_message);
    LocalPlannerError planner_error;
    planner_error.error_pose_a = trajectory.movements.front().pose;
    planner_error.error_pose_b = trajectory.movements.back().pose;
    planner_error.error_enum = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
    callErrorCB(planner_error);
    return boost::none;
  }
  measurement_getTransform.stop();

  arti_profiling::DurationMeasurement measurement_transformMovement(profiler_, "transformMovement");
  const arti_nav_core_msgs::Trajectory2DWithLimits result
    = arti_nav_core_utils::transformTrajectory(trajectory, *transform_msg);
  measurement_transformMovement.stop();

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformTrajectory: trajectory: " << trajectory);
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformTrajectory: result: " << result);
  return result;
}

boost::optional<GlobalPlannerResult> LocalPlanner::transformGlobalPlannerResult(
  const GlobalPlannerResult& global_planner_result)
{
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformGlobalPlannerResult global_planner_result: "
    << global_planner_result.plan);

  const std::string frame_id = costmap_->getGlobalFrameID();
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformGlobalPlannerResult frame_id: " << frame_id);

  if (global_planner_result.plan.header.frame_id == frame_id)
  {
    return global_planner_result;
  }

  ros::Time stamp = global_planner_result.plan.header.stamp;
  if (!stamp.isZero())
  {
    stamp = ros::Time::now();
  }
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformGlobalPlannerResult stamp: " << stamp);

  std::string error_message;
  const boost::optional<geometry_msgs::TransformStamped> transform_msg
    = arti_nav_core_utils::tryToLookupTransform(*transformer_, frame_id, global_planner_result.plan.header.frame_id,
                                                stamp, ros::Duration(config_.transformation_timeout), &error_message);
  if (!transform_msg)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "local planner cannot transform global plan: " << error_message);
    return boost::none;
  }

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, "LocalPlanner::transformGlobalPlannerResult: transform_msg: "
    << "stamp: " << transform_msg->header.stamp << ", frame_id: " << transform_msg->header.frame_id);

  GlobalPlannerResult result;
  result.plan = arti_nav_core_utils::transformPath(global_planner_result.plan, *transform_msg);
  // As the twist is considered relative to the last pose of the path, it doesn't change when the path is transformed:
  result.final_twist = global_planner_result.final_twist;
  return result;
}

bool LocalPlanner::checkTrajectoryValid(arti_nav_core_msgs::Trajectory2DWithLimits& limits)
{
  if (limits.movements.empty())
  {
    return false; // empty trajectory
  }

  double last_theta = limits.movements.front().pose.theta.value;
  double angle_accumulated = 0.;

  arti_nav_core_msgs::Point2DWithLimits last_point = limits.movements.front().pose.point;
  double dist_accumulated = 0.;

  for (const auto& idx_pose: limits.movements)
  {
    angle_accumulated += idx_pose.pose.theta.value - last_theta;
    double delta_dist = std::hypot(idx_pose.pose.point.x.value - last_point.x.value,
                                   idx_pose.pose.point.y.value - last_point.y.value);

    dist_accumulated += delta_dist;

    if (delta_dist > config_.max_dist_between_planner_poses)
    {
      ROS_WARN_STREAM("Too big gaps within trajectory poses (" << delta_dist << " m)");
      return false;
    }


    last_theta = idx_pose.pose.theta.value;
    last_point = idx_pose.pose.point;
  }

  if (angle_accumulated > config_.max_accumulated_angle)
  {
    planner_->resetPlanner();
    ROS_WARN_STREAM("Trajectory is not valid - accumulated angle too big (" << angle_accumulated << ")");
    return false;
  }

  if (dist_accumulated > config_.max_accumulated_distance)
  {
    planner_->resetPlanner();
    ROS_WARN_STREAM("Trajectory is not valid - accumulated distance too big (" << dist_accumulated
                                                                               << "), limits.movements.size() = "
                                                                               << limits.movements.size());
    return false;
  }

  return true;
}

}  // namespace arti_move_base
