/*
Created by clemens on 19.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_move_base/arti_move_base_node.h>
#include <arti_move_base/pipeline_builder.h>
#include <arti_move_base/pipeline_base_impl.h>
#include <arti_move_base/utils.h>
#include <arti_nav_core_utils/conversions.h>
#include <arti_nav_core_utils/transformer.h>
#include <string>

namespace arti_move_base
{

ArtiMoveBase::ArtiMoveBase(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle), config_server_(node_handle_), transformer_(arti_nav_core_utils::createTransformer()),
    current_goal_publisher_(node_handle_.advertise<geometry_msgs::PoseStamped>("current_goal", 1)),
    clear_costmaps_server_(node_handle_.advertiseService("clear_costmaps", &ArtiMoveBase::clearCostmapsCB, this)),
    follow_target_action_server_(root_node_handle_, "follow_target", false),
    follow_trajectory_action_server_(root_node_handle_, "follow_trajectory", false),
    move_in_network_action_server_(root_node_handle_, "move_in_network", false), move_base_server_(root_node_handle_)
{
  config_server_.setCallback(std::bind(&ArtiMoveBase::reconfigure, this, std::placeholders::_1));

  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);

  const std::shared_ptr<costmap_2d::Costmap2DROS> global_planner_costmap =
    std::make_shared<costmap_2d::Costmap2DROS>("global_costmap", *transformer_);
  global_planner_costmap->pause();

  const std::shared_ptr<costmap_2d::Costmap2DROS> local_planner_costmap =
    std::make_shared<costmap_2d::Costmap2DROS>("local_costmap", *transformer_);
  local_planner_costmap->pause();

  const std::shared_ptr<costmap_2d::Costmap2DROS> path_follower_costmap =
    std::make_shared<costmap_2d::Costmap2DROS>("path_follower_costmap", *transformer_);
  path_follower_costmap->pause();

  costmaps_ = {global_planner_costmap, local_planner_costmap, path_follower_costmap};

  PipelineBuilder pipeline_builder;
  pipeline_builder.addNetworkPlanner(ros::NodeHandle(node_handle_, "network_planner"),
                                     std::bind(&ArtiMoveBase::networkPlannerErrorCB, this, std::placeholders::_1),
                                     std::bind(&ArtiMoveBase::networkPlannerSuccessCB, this), transformer_);
  pipeline_builder.addGlobalPlanner(ros::NodeHandle(node_handle_, "global_planner"),
                                    std::bind(&ArtiMoveBase::globalPlannerErrorCB, this, std::placeholders::_1),
                                    std::bind(&ArtiMoveBase::globalPlannerSuccessCB, this), transformer_,
                                    global_planner_costmap);
  pipeline_builder.addLocalPlanner(ros::NodeHandle(node_handle_, "local_planner"), nullptr, nullptr, transformer_,
                                   local_planner_costmap);

  const ros::NodeHandle path_follower_node_handle(node_handle_, "path_follower");
  if (config_.ackermann_steering)
  {
    pipeline_builder.addPathFollower<ackermann_msgs::AckermannDrive>(
      path_follower_node_handle,
      std::bind(&ArtiMoveBase::pathFollowerErrorCB<PathFollower<ackermann_msgs::AckermannDrive>::Error>, this,
                std::placeholders::_1),
      std::bind(&ArtiMoveBase::pathFollowerSuccessCB, this), transformer_, path_follower_costmap);
    planner_pipeline_
      = std::make_shared<PipelineBaseImpl<ackermann_msgs::AckermannDrive>>(root_node_handle_, "cmd_vel");
  }
  else
  {
    pipeline_builder.addPathFollower<geometry_msgs::Twist>(
      path_follower_node_handle,
      std::bind(&ArtiMoveBase::pathFollowerErrorCB<PathFollower<geometry_msgs::Twist>::Error>, this,
                std::placeholders::_1),
      std::bind(&ArtiMoveBase::pathFollowerSuccessCB, this), transformer_, path_follower_costmap);
    planner_pipeline_ = std::make_shared<PipelineBaseImpl<geometry_msgs::Twist>>(root_node_handle_, "cmd_vel");
  }
  planner_pipeline_->createPipeline(pipeline_builder);

  for (const auto& costmap : costmaps_)
  {
    costmap->start();
  }

  follow_target_action_server_.registerGoalCallback(std::bind(&ArtiMoveBase::followTargetGoalCB, this));
  follow_target_action_server_.registerPreemptCallback(std::bind(&ArtiMoveBase::followTargetPreemptCB, this));
  follow_target_action_server_.start();

  follow_trajectory_action_server_.registerGoalCallback(std::bind(&ArtiMoveBase::followTrajectoryGoalCB, this));
  follow_trajectory_action_server_.registerPreemptCallback(std::bind(&ArtiMoveBase::followTrajectoryPreemptCB, this));
  follow_trajectory_action_server_.start();

  move_in_network_action_server_.registerGoalCallback(std::bind(&ArtiMoveBase::moveInNetworkGoalCB, this));
  move_in_network_action_server_.registerPreemptCallback(std::bind(&ArtiMoveBase::moveInNetworkPreemptCB, this));
  move_in_network_action_server_.start();
}

void ArtiMoveBase::reconfigure(MoveBaseConfig& config)
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  if (planner_pipeline_)
  {
    // This parameter can't change after the pipeline has been created, so set it to whatever it was before:
    config.ackermann_steering = config_.ackermann_steering;
  }
  config_ = config;
}

void ArtiMoveBase::followTargetGoalCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  const arti_move_base_msgs::FollowTargetGoalConstPtr goal = follow_target_action_server_.acceptNewGoal();
  if (goal)
  {
    ROS_DEBUG_STREAM("follow target action called with new goal: " << *goal);
    // This will set the input of the global planner:
    arti_nav_core_msgs::Movement2DGoalWithConstraints pipeline_goal;
    pipeline_goal.goal.pose = addLimitsIfMissing(goal->target_movement.pose);
    pipeline_goal.goal.twist = goal->target_movement.twist;
    error_handled_ = false;
    planner_pipeline_->setInput(pipeline_goal);
    publishCurrentGoal(pipeline_goal.goal.pose);
  }
}

void ArtiMoveBase::followTargetPreemptCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  planner_pipeline_->cancel();
  follow_target_action_server_.setPreempted();
}

void ArtiMoveBase::followTrajectoryGoalCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  const arti_move_base_msgs::FollowTrajectoryGoalConstPtr goal = follow_trajectory_action_server_.acceptNewGoal();
  if (goal)
  {
    ROS_DEBUG_STREAM("follow trajectory action called with new goal: " << *goal);
    // This will set the input of the path follower:
    error_handled_ = false;
    planner_pipeline_->setInput(goal->trajectory);
    if (!goal->trajectory.movements.empty())
    {
      arti_nav_core_msgs::Pose2DStampedWithLimits last_pose;
      last_pose.header = goal->trajectory.header;
      last_pose.pose = goal->trajectory.movements.back().pose;
      publishCurrentGoal(last_pose);
    }
  }
}

void ArtiMoveBase::followTrajectoryPreemptCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  planner_pipeline_->cancel();
  follow_trajectory_action_server_.setPreempted();
}

void ArtiMoveBase::moveInNetworkGoalCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  const arti_move_base_msgs::MoveInNetworkGoalConstPtr goal = move_in_network_action_server_.acceptNewGoal();
  if (goal)
  {
    ROS_DEBUG_STREAM("move in network action called with new goal: " << *goal);
    // This will set the input of the network planner:
    const arti_nav_core_msgs::Pose2DStampedWithLimits pipeline_goal = addLimitsIfMissing(goal->target_pose);
    error_handled_ = false;
    planner_pipeline_->setInput(pipeline_goal);
    publishCurrentGoal(pipeline_goal);
  }
}

void ArtiMoveBase::moveInNetworkPreemptCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  planner_pipeline_->cancel();
  move_in_network_action_server_.setPreempted();
}

bool ArtiMoveBase::clearCostmapsCB(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*resp*/)
{
  for (const auto& costmap : costmaps_)
  {
    costmap->resetLayers();
  }
  return true;
}

void ArtiMoveBase::networkPlannerSuccessCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  planner_pipeline_->cancel();

  if (move_in_network_action_server_.isActive())
  {
    move_in_network_action_server_.setSucceeded({}, "goal reached");
  }
}

void ArtiMoveBase::networkPlannerErrorCB(const arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum& error)
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  if (!error_handled_)
  {
    error_handled_ = true;
    planner_pipeline_->cancel();

    if (move_in_network_action_server_.isActive())
    {
      move_in_network_action_server_.setAborted({}, std::string("network planner failed with error ")
                                                    + utils::toString(error));
    }
  }
}

void ArtiMoveBase::globalPlannerSuccessCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  // Only handle the success if the pipeline was started because of a call to this action:
  if (follow_target_action_server_.isActive())
  {
    planner_pipeline_->cancel();

    follow_target_action_server_.setSucceeded({}, "goal reached");
  }
}

void ArtiMoveBase::globalPlannerErrorCB(const GlobalPlannerError& error)
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  // Only handle the error if the pipeline was started because of a call to this action:
  if (follow_target_action_server_.isActive() && !error_handled_)
  {
    error_handled_ = true;
    planner_pipeline_->cancel();

    follow_target_action_server_.setAborted({}, std::string("global planner failed with error ")
                                                + utils::toString(error.error_enum));
  }
}

void ArtiMoveBase::pathFollowerSuccessCB()
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  // Only handle the success if the pipeline was started because of a call to this action:
  if (follow_trajectory_action_server_.isActive())
  {
    planner_pipeline_->cancel();

    follow_trajectory_action_server_.setSucceeded({}, "goal reached");
  }
}

template<typename E>
void ArtiMoveBase::pathFollowerErrorCB(const E& error)
{
  std::lock_guard<std::mutex> planner_pipeline_lock(planner_pipeline_mutex_);
  // Only handle the error if the pipeline was started because of a call to this action:
  if (follow_trajectory_action_server_.isActive() && !error_handled_)
  {
    error_handled_ = true;
    planner_pipeline_->cancel();

    follow_trajectory_action_server_.setAborted({}, std::string("path follower failed with error ")
                                                    + utils::toString(error));
  }
}

arti_nav_core_msgs::Pose2DStampedWithLimits ArtiMoveBase::addLimitsIfMissing(
  arti_nav_core_msgs::Pose2DStampedWithLimits goal) const
{
  if (!goal.pose.point.x.has_limits)
  {
    goal.pose.point.x.has_limits = true;
    goal.pose.point.x.upper_limit = config_.x_y_simple_goal_final_tolerance;
    goal.pose.point.x.lower_limit = -config_.x_y_simple_goal_final_tolerance;
  }

  if (!goal.pose.point.y.has_limits)
  {
    goal.pose.point.y.has_limits = true;
    goal.pose.point.y.upper_limit = config_.x_y_simple_goal_final_tolerance;
    goal.pose.point.y.lower_limit = -config_.x_y_simple_goal_final_tolerance;
  }

  if (!goal.pose.theta.has_limits)
  {
    goal.pose.theta.has_limits = true;
    goal.pose.theta.upper_limit = config_.yaw_simple_goal_final_tolerance;
    goal.pose.theta.lower_limit = -config_.yaw_simple_goal_final_tolerance;
  }

  return goal;
}

void ArtiMoveBase::publishCurrentGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& pose)
{
  current_goal_publisher_.publish(
    arti_nav_core_utils::convertToPoseStamped(pose, arti_nav_core_utils::non_finite_values::PASS_THROUGH));
}

}  // namespace arti_move_base


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arti_move_base");
  arti_move_base::ArtiMoveBase node{{"~"}};
  ros::spin();
  return 0;
}
