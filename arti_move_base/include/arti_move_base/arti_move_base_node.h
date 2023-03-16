/*
Created by clemens on 21.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_ARTI_MOVE_BASE_NODE_H
#define ARTI_MOVE_BASE_ARTI_MOVE_BASE_NODE_H

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <actionlib/server/simple_action_server.h>
#include <arti_move_base/global_planner_error.h>
#include <arti_move_base/move_base_server.h>
#include <arti_move_base/MoveBaseConfig.h>
#include <arti_move_base/pipeline_base.h>
#include <arti_move_base_msgs/FollowTargetAction.h>
#include <arti_move_base_msgs/FollowTrajectoryAction.h>
#include <arti_move_base_msgs/MoveInNetworkAction.h>
#include <arti_nav_core/base_network_planner.h>
#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <mutex>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_srvs/Empty.h>
#include <vector>

namespace arti_move_base
{

class ArtiMoveBase
{
public:
  explicit ArtiMoveBase(const ros::NodeHandle& node_handle);

private:
  void reconfigure(MoveBaseConfig& config);

  void followTargetGoalCB();

  void followTargetPreemptCB();

  void followTrajectoryGoalCB();

  void followTrajectoryPreemptCB();

  void moveInNetworkGoalCB();

  void moveInNetworkPreemptCB();

  bool clearCostmapsCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  bool stopMovementCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  void networkPlannerSuccessCB();

  void networkPlannerErrorCB(const arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum& error);

  void globalPlannerSuccessCB();

  void globalPlannerErrorCB(const GlobalPlannerError& error);

  void pathFollowerSuccessCB();

  template<typename E>
  void pathFollowerErrorCB(const E& error);

  arti_nav_core_msgs::Pose2DStampedWithLimits addLimitsIfMissing(
    arti_nav_core_msgs::Pose2DStampedWithLimits goal) const;

  void publishCurrentGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& pose);

  ros::NodeHandle node_handle_;
  ros::NodeHandle root_node_handle_;

  dynamic_reconfigure::Server<MoveBaseConfig> config_server_;
  MoveBaseConfig config_;

  std::shared_ptr<arti_nav_core::Transformer> transformer_;

  ros::Publisher current_goal_publisher_;
  ros::ServiceServer clear_costmaps_server_;
  ros::ServiceServer stop_movement_server_;

  actionlib::SimpleActionServer<arti_move_base_msgs::FollowTargetAction> follow_target_action_server_;
  actionlib::SimpleActionServer<arti_move_base_msgs::FollowTrajectoryAction> follow_trajectory_action_server_;
  actionlib::SimpleActionServer<arti_move_base_msgs::MoveInNetworkAction> move_in_network_action_server_;

  MoveBaseServer move_base_server_;

  std::mutex planner_pipeline_mutex_;
  std::shared_ptr<PipelineBase> planner_pipeline_;
  bool error_handled_{false};
  std::vector<std::shared_ptr<costmap_2d::Costmap2DROS>> costmaps_;
};

}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_ARTI_MOVE_BASE_NODE_H
