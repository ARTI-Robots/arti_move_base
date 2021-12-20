/*
Created by abuchegger on 18.09.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_move_base/utils.h>
#include <arti_move_base/limit_utils.h>
#include <ros/console.h>
#include <ros/names.h>

namespace arti_move_base
{
namespace utils
{

boost::optional<geometry_msgs::PoseStamped> getRobotPose(const costmap_2d::Costmap2DROS& costmap)
{
#if ROS_VERSION_MINIMUM(1, 14, 0) // if MELODIC (v14)
  geometry_msgs::PoseStamped robot_pose;
  if (costmap.getRobotPose(robot_pose))
  {
    return robot_pose;
  }
#else // if KINETIC (v12)
  tf::Stamped<tf::Pose> robot_pose_tf;
  if (costmap.getRobotPose(robot_pose_tf))
  {
    geometry_msgs::PoseStamped robot_pose;
    tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);
    return robot_pose;
  }
#endif
  return boost::none;
}

boost::optional<size_t> findPose(
  const LimitUtils& limit_utils, const std::vector<arti_nav_core_msgs::Pose2DWithLimits>& poses, double x, double y,
  double theta)
{
  boost::optional<size_t> result;
  for (size_t i = 0; i < poses.size(); ++i)
  {
    const auto& pose = poses[i];
    if (limit_utils.xValueWithinTolerance(x, pose.point.x) &&
        limit_utils.yValueWithinTolerance(y, pose.point.y))
    {
      if (limit_utils.thetaValueWithinTolerance(theta, pose.theta))
      {
        result = i;
      }
      else if (result)
      {
        break;
      }
    }
  }
  return result;
}

std::string getRelativeNamespace(const ros::NodeHandle& node_handle)
{
  const std::string abs_namespace = node_handle.getNamespace();
  static const std::string private_node_namespace = ros::names::resolve("~/");
  if (abs_namespace.compare(0, private_node_namespace.size(), private_node_namespace) != 0)
  {
    throw std::invalid_argument(
      "given namespace '" + abs_namespace + "' is not within private node namespace '" + private_node_namespace + "'");
  }
  return abs_namespace.substr(private_node_namespace.size());
}

const char* toString(const arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum error)
{
  switch (error)
  {
#define X(name) case arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum::name: return #name;
    X(PLAN_FOUND)
    X(FAR_FROM_PATH)
    X(NO_PATH_POSSIBLE)
#undef X
  }
  return "BUG";
}

const char* toString(const arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum error)
{
  switch (error)
  {
#define X(name) case arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::name: return #name;
    X(PLAN_FOUND)
    X(FAR_FROM_PATH)
    X(NO_PATH_POSSIBLE)
#undef X
  }
  return "BUG";
}

const char* toString(arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum error)
{
  switch (error)
  {
#define X(name) case arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::name: return #name;
    X(GOAL_REACHED)
    X(COMMAND_FOUND)
    X(OBSTACLE_CLOSE)
    X(OBSTACLE_IN_FRONT)
    X(FAR_FROM_PATH)
    X(NO_COMMAND_POSSIBLE)
#undef X
  }
  return "BUG";
}

const char* toString(arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum error)
{
  switch (error)
  {
#define X(name) case arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum::name: return #name;
    X(GOAL_REACHED)
    X(COMMAND_FOUND)
    X(OBSTACLE_CLOSE)
    X(OBSTACLE_IN_FRONT)
    X(FAR_FROM_PATH)
    X(NO_COMMAND_POSSIBLE)
#undef X
  }
  return "BUG";
}


}
}
