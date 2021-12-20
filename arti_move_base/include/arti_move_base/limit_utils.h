/*
Created by clemens on 6/4/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_LIMITUTILS_H
#define ARTI_MOVE_BASE_LIMITUTILS_H

#include <arti_nav_core_msgs/Pose2DWithLimits.h>
#include <arti_nav_core_msgs/ValueWithLimits.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

namespace arti_move_base
{
class LimitUtils
{
public:
  LimitUtils(double xy_tolerance_increase, double yaw_tolerance_increase);

  bool xValueWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const;
  bool yValueWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const;
  bool thetaValueWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal_value) const;

  bool poseWithinTolerance(
    double current_x, double current_y, double current_theta,
    const arti_nav_core_msgs::Pose2DWithLimits& goal_pose) const;

  bool poseWithinTolerance(
    const geometry_msgs::Pose& current_pose, const arti_nav_core_msgs::Pose2DWithLimits& goal_pose) const;

  bool poseWithinTolerance(
    const geometry_msgs::Pose2D& current_pose, const arti_nav_core_msgs::Pose2DWithLimits& goal_pose) const;

private:
  static const char LOGGER_NAME[];

  static bool withinTolerance(double distance, const arti_nav_core_msgs::ValueWithLimits& goal_value);

  double xy_tolerance_increase_;
  double yaw_tolerance_increase_;
};
}

#endif //ARTI_MOVE_BASE_LIMITUTILS_H
