/*
Created by Alexander Buchegger on 2020-09-18.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_MOVE_BASE_UTILS_H
#define ARTI_MOVE_BASE_UTILS_H

#include <boost/optional.hpp>
#include <arti_nav_core/base_global_planner.h>
#include <arti_nav_core/base_network_planner.h>
#include <arti_nav_core/base_path_follower.h>
#include <arti_nav_core/base_path_follower_ackermann.h>
#include <arti_nav_core_msgs/Pose2DWithLimits.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <costmap_2d/costmap_2d_ros.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_move_base
{
class LimitUtils;

namespace utils
{

boost::optional<geometry_msgs::PoseStamped> getRobotPose(const costmap_2d::Costmap2DROS& costmap);

boost::optional<size_t> findPose(
  const LimitUtils& limit_utils, const std::vector<arti_nav_core_msgs::Pose2DWithLimits>& poses, double x, double y,
  double theta);

std::string getRelativeNamespace(const ros::NodeHandle& node_handle);

const char* toString(arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum error);

const char* toString(arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum error);

const char* toString(arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum error);

const char* toString(arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum error);

}
}

#endif //ARTI_MOVE_BASE_UTILS_H
