/*
Created by abuchegger on 2021-01-23.
This file is part of the software provided by ARTI
Copyright (c) 2021, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_move_base/move_base_server.h>
#include <functional>
#include <tf/transform_datatypes.h>

namespace arti_move_base
{

MoveBaseServer::MoveBaseServer(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle),
    move_base_simple_goal_subscriber_(
      node_handle_.subscribe("move_base_simple/goal", 1, &MoveBaseServer::moveBaseSimpleGoalCB, this)),
    move_base_action_server_(node_handle_, "move_base", false),
    move_in_network_action_client_(node_handle_, "move_in_network"),
    follow_target_action_client_(node_handle_, "follow_target")
{
  move_base_action_server_.registerGoalCallback(std::bind(&MoveBaseServer::moveBaseGoalCB, this));
  move_base_action_server_.registerPreemptCallback(std::bind(&MoveBaseServer::moveBasePreemptCB, this));
  move_base_action_server_.start();
}

bool MoveBaseServer::isActive()
{
  return move_base_action_server_.isActive();
}

void MoveBaseServer::cancleGoal()
{
  moveBasePreemptCB();
}

void MoveBaseServer::moveBaseSimpleGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  ROS_DEBUG_STREAM("received move base simple goal: " << *goal);

  arti_move_base_msgs::MoveInNetworkGoal move_in_network_goal;
  move_in_network_goal.target_pose = convertToPose2DStamped(*goal);
  move_in_network_action_client_.sendGoal(
    move_in_network_goal, std::bind(&MoveBaseServer::moveInNetworkDoneCB, this, std::placeholders::_1));
}

void MoveBaseServer::moveInNetworkDoneCB(const actionlib::SimpleClientGoalState& goal_state)
{
  switch (goal_state.state_)
  {
    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::PREEMPTED:
      ROS_DEBUG_STREAM("move base simple goal was preempted: " << goal_state.text_);
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_DEBUG_STREAM("move base simple goal was aborted: " << goal_state.text_);
      break;

    case actionlib::SimpleClientGoalState::SUCCEEDED:
      ROS_DEBUG_STREAM("move base simple goal succeeded: " << goal_state.text_);
      break;

    default:
      ROS_ERROR_STREAM("received a done callback for a move base simple goal with an unexpected goal state: "
                         << goal_state.toString());
      break;
  }
}

void MoveBaseServer::moveBaseGoalCB()
{
  const move_base_msgs::MoveBaseGoalConstPtr goal = move_base_action_server_.acceptNewGoal();
  if (goal)
  {
    ROS_DEBUG_STREAM("move base action called with new goal: " << *goal);

    arti_move_base_msgs::FollowTargetGoal follow_target_goal;
    follow_target_goal.target_movement.pose = convertToPose2DStamped(goal->target_pose);
    follow_target_action_client_.sendGoal(
      follow_target_goal, std::bind(&MoveBaseServer::followTargetDoneCB, this, std::placeholders::_1));
  }
}

void MoveBaseServer::moveBasePreemptCB()
{
  if (move_base_action_server_.isActive())
  {
    if (!follow_target_action_client_.getState().isDone())
    {
      follow_target_action_client_.cancelGoal();
    }

    move_base_action_server_.setPreempted();
  }
}

void MoveBaseServer::followTargetDoneCB(const actionlib::SimpleClientGoalState& goal_state)
{
  // Move base action server is active (has an active goal), thus we assume the goal was sent due to move base action
  // call:
  if (move_base_action_server_.isActive())
  {
    switch (goal_state.state_)
    {
      case actionlib::SimpleClientGoalState::RECALLED:
      case actionlib::SimpleClientGoalState::PREEMPTED:
        move_base_action_server_.setPreempted({}, goal_state.text_);
        break;

      case actionlib::SimpleClientGoalState::REJECTED:
      case actionlib::SimpleClientGoalState::ABORTED:
        move_base_action_server_.setAborted({}, goal_state.text_);
        break;

      case actionlib::SimpleClientGoalState::SUCCEEDED:
        move_base_action_server_.setSucceeded({}, goal_state.text_);
        break;

      default:
        ROS_ERROR_STREAM("received a done callback with an unexpected goal state: " << goal_state.toString());
        move_base_action_server_.setAborted({}, goal_state.text_);
        break;
    }
  }
  else if (goal_state != actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_ERROR_STREAM("received a done callback for a goal which was not send due to a move base goal");
  }
}

arti_nav_core_msgs::Pose2DStampedWithLimits MoveBaseServer::convertToPose2DStamped(
  const geometry_msgs::PoseStamped& goal)
{
  arti_nav_core_msgs::Pose2DStampedWithLimits result;
  result.header = goal.header;

  // Limits will be filled in ArtiMoveBase:
  result.pose.point.x.value = goal.pose.position.x;
  result.pose.point.y.value = goal.pose.position.y;
  result.pose.theta.value = tf::getYaw(goal.pose.orientation);

  return result;
}

}
