/*
Created by clemens on 6/4/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_move_base/limit_utils.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>

namespace arti_move_base
{
const char LimitUtils::LOGGER_NAME[] = "limit_utils";

LimitUtils::LimitUtils(double xy_tolerance_increase, double yaw_tolerance_increase)
  : xy_tolerance_increase_(xy_tolerance_increase), yaw_tolerance_increase_(yaw_tolerance_increase)
{
}

bool LimitUtils::xValueWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const
{
  return withinTolerance((current_value - goal_value.value) / xy_tolerance_increase_, goal_value);
}

bool LimitUtils::yValueWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const
{
  return withinTolerance((current_value - goal_value.value) / xy_tolerance_increase_, goal_value);
}

bool LimitUtils::thetaValueWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const
{
  return withinTolerance(angles::normalize_angle(current_value - goal_value.value) / yaw_tolerance_increase_,
                         goal_value);
}

bool LimitUtils::poseWithinTolerance(
  double current_x, double current_y, double current_theta, const arti_nav_core_msgs::Pose2DWithLimits& goal_pose) const
{
  return xValueWithinTolerance(current_x, goal_pose.point.x)
         && yValueWithinTolerance(current_y, goal_pose.point.y)
         && thetaValueWithinTolerance(current_theta, goal_pose.theta);
}

bool LimitUtils::poseWithinTolerance(
  const geometry_msgs::Pose& current_pose, const arti_nav_core_msgs::Pose2DWithLimits& goal_pose) const
{
  return poseWithinTolerance(current_pose.position.x, current_pose.position.y, tf::getYaw(current_pose.orientation),
                             goal_pose);
}

bool LimitUtils::poseWithinTolerance(
  const geometry_msgs::Pose2D& current_pose, const arti_nav_core_msgs::Pose2DWithLimits& goal_pose) const
{
  return poseWithinTolerance(current_pose.x, current_pose.y, current_pose.theta, goal_pose);
}

bool LimitUtils::withinTolerance(double distance, const arti_nav_core_msgs::ValueWithLimits& goal_value)
{
  if (!std::isfinite(goal_value.value))
  {
    return true;
  }

  if (!goal_value.has_limits)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME,
                          "goal value has no limits, which currently means that value has to be reached exactly");
    return distance == 0.0;
  }
  return goal_value.lower_limit <= distance && distance <= goal_value.upper_limit;
}

}
