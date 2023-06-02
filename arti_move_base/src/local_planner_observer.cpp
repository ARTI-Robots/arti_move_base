/*
Created by R.Hoheneder on 2022-02-07.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <arti_move_base/local_planner_observer.h>
#include <arti_nav_core_utils/transformer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arti_move_base
{

LocalPlannerObserver::LocalPlannerObserver(
  const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
  const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
  const std::vector<SuccessCallback>& close_to_success_cbs,
  std::shared_ptr<arti_nav_core::Transformer> transformer,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap)
  : arti_async_utils::PeriodicPipelineStage<arti_nav_core_msgs::Trajectory2DWithLimits,
  bool,
  arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum>(input, output, error_cbs, success_cbs,
                                                              close_to_success_cbs, std::chrono::seconds(1),
                                                              "LocalPlannerObserver"),
    node_handle_(node_handle), transformer_(std::move(transformer)),
    costmap_(std::move(costmap)), config_server_(node_handle_)
{
  config_server_.setCallback(std::bind(&LocalPlannerObserver::reconfigure, this, std::placeholders::_1));

  costmap_collision_checker_.reset(new arti_costmap_collision_checker::CostmapCollisionCheck(node_handle_,
                                                                                             costmap_.get()));

  pub_obstacle_ = node_handle_.advertise<visualization_msgs::Marker>("last_obstacle", 10, true);
  ROS_INFO_STREAM("Local Plan Observer loaded");
}

void LocalPlannerObserver::cancel()
{
  current_trajectory_.reset();
  arti_async_utils::PeriodicPipelineStage<arti_nav_core_msgs::Trajectory2DWithLimits, bool,
    arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum>::cancel();
}

void LocalPlannerObserver::reconfigure(const LocalPlannerObserverConfig& config)
{
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*this->costmap_->getCostmap()->getMutex());
  config_ = config;
  this->setExecutionDuration(std::chrono::duration<double>(1.0 / std::max(config_.control_rate, 1.0)));
}

boost::optional<bool> LocalPlannerObserver::performTask(
  const arti_nav_core_msgs::Trajectory2DWithLimits& input, bool new_input)
{
  if (!config_.enable_trajectory_validation)
  {
    return true;
  }

  if (new_input)
  {
    current_input_index_ = 0;
  }
  else
  {
    //advance current input index if we passed some points
    const boost::optional<geometry_msgs::PoseStamped> robot_pose = utils::getRobotPose(*costmap_);
    if (!robot_pose)
    {
      ROS_ERROR_STREAM("can not get robot pose from costmap for collision checking");
      return false;
    }

    double min_distance = std::numeric_limits<double>::max();
    size_t new_current_input_index = current_input_index_;
    for (size_t i = current_input_index_; i < input.movements.size(); ++i)
    {
      const double distance = getDistance(*robot_pose, input.movements[i]);
      if (distance > config_.passing_distance)
      {
        break;
      }

      if (distance < min_distance)
      {
        min_distance = distance;
      }
      else
      {
        //we stop removing points if we reached the minimum distance as this should be the current point
        break;
      }

      new_current_input_index = i;
    }

    current_input_index_ = new_current_input_index;
  }


  costmap_collision_checker_->setPadding(config_.padding);
  for (size_t i = current_input_index_; i < input.movements.size(); ++i)
  {
    if (costmap_collision_checker_->isInCollision(input.movements[i]))
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Trajectory blocked, cancel trajectory");

      ROS_ERROR_NAMED(LOGGER_NAME, "pose of obstacle: x:[%f] y:[%f], frame  [%s]", input.movements[i].pose.point.x
      .value, input.movements[i].pose.point.y.value, input.header.frame_id.c_str());

      visualization_msgs::Marker obstacle_marker;
      obstacle_marker.ns = "obstacle_in_path";
      obstacle_marker.id = 0;
      obstacle_marker.action = visualization_msgs::Marker::ADD;
      obstacle_marker.type = visualization_msgs::Marker::SPHERE;
      obstacle_marker.header.frame_id = input.header.frame_id;
      obstacle_marker.header.stamp = ros::Time::now();
      obstacle_marker.color.r = 1;
      obstacle_marker.color.g = 0.0;
      obstacle_marker.color.b = 0.0;
      obstacle_marker.color.a = 0.75;
      obstacle_marker.pose.orientation.w = 1.;
      obstacle_marker.scale.x = 1.1;
      obstacle_marker.scale.y = 1.1;
      obstacle_marker.scale.z = 1.1;
      obstacle_marker.pose.position.x = input.movements[i].pose.point.x.value;
      obstacle_marker.pose.position.y = input.movements[i].pose.point.y.value;
      obstacle_marker.pose.position.z = 0.5;

      if(last_obstacle_)
      {
        obstacle_marker.header.seq = last_obstacle_->header.seq +1;
      }
      else
      {
        obstacle_marker.header.seq = 1;
      }
      last_obstacle_.reset(obstacle_marker);
      pub_obstacle_.publish(obstacle_marker);
      this->callErrorCB(arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum::OBSTACLE_CLOSE);
      return false;
    }
    if(last_obstacle_)
    {
      pub_obstacle_.publish(last_obstacle_.get());
    }
  }

  bool check = !input.movements.empty() && (current_input_index_ == 0);
  ROS_DEBUG_STREAM("movement is empty check yielded: " << static_cast<int>(check));
  costmap_collision_checker_->publishCheckedPoses();

  return check;
}

double LocalPlannerObserver::getDistance(
  const geometry_msgs::PoseStamped& pose, const arti_nav_core_msgs::Movement2DWithLimits& movement)
{
  return std::hypot(pose.pose.position.x - movement.pose.point.x.value,
                    pose.pose.position.y - movement.pose.point.y.value);
}

}
