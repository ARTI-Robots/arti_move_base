/*
Created by abuchegger on 2021-01-23.
This file is part of the software provided by ARTI
Copyright (c) 2021, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_MOVE_BASE_ACTION_SERVER_H
#define ARTI_MOVE_BASE_MOVE_BASE_ACTION_SERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arti_move_base_msgs/FollowTargetAction.h>
#include <arti_move_base_msgs/MoveInNetworkAction.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

namespace arti_move_base
{

/**
 * Provides the ROS interface of the "old" move_base, namely the move_base action server and the move base simple goal
 * subscriber. Calls arti_move_base actions with the received goals.
 */
class MoveBaseServer
{
public:
  explicit MoveBaseServer(const ros::NodeHandle& node_handle);

protected:
  void moveBaseSimpleGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

  void moveInNetworkDoneCB(const actionlib::SimpleClientGoalState& goal_state);

  void moveBaseGoalCB();

  void moveBasePreemptCB();

  void followTargetDoneCB(const actionlib::SimpleClientGoalState& goal_state);

  static arti_nav_core_msgs::Pose2DStampedWithLimits convertToPose2DStamped(const geometry_msgs::PoseStamped& goal);

  ros::NodeHandle node_handle_;
  ros::Subscriber move_base_simple_goal_subscriber_;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> move_base_action_server_;
  actionlib::SimpleActionClient<arti_move_base_msgs::MoveInNetworkAction> move_in_network_action_client_;
  actionlib::SimpleActionClient<arti_move_base_msgs::FollowTargetAction> follow_target_action_client_;
};

}

#endif //ARTI_MOVE_BASE_MOVE_BASE_ACTION_SERVER_H
