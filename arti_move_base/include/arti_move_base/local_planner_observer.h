/*
Created by R.Hoheneder on 2022-02-07.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_LOCAL_PLANNER_OBSERVER_H
#define ARTI_MOVE_BASE_LOCAL_PLANNER_OBSERVER_H

#include <arti_async_utils/periodic_pipeline_stage.h>
#include <arti_move_base/limit_utils.h>
#include <arti_move_base/LocalPlannerObserverConfig.h>
#include <arti_move_base/plugin_helper.h>
#include <arti_move_base/utils.h>
#include <arti_nav_core/transformer.h>

#include <arti_costmap_collision_checker/costmap_collision_check.h>

#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <string>
#include <vector>
namespace arti_move_base

{
class LocalPlannerObserver
  : public arti_async_utils::PeriodicPipelineStage<arti_nav_core_msgs::Trajectory2DWithLimits, bool,
    arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum>
{
public:

  LocalPlannerObserver(
    const ros::NodeHandle& node_handle, const InputBufferPtr& input, const OutputBufferPtr& output,
    const std::vector<ErrorCallback>& error_cbs, const std::vector<SuccessCallback>& success_cbs,
    const std::vector<SuccessCallback>& close_to_success_cbs,
    std::shared_ptr<arti_nav_core::Transformer> transformer,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap);

  void cancel() override;

protected:
  static constexpr const char* const LOGGER_NAME = "local_planner_observer";

  void reconfigure(const LocalPlannerObserverConfig& config);

  boost::optional<bool>
        performTask(const arti_nav_core_msgs::Trajectory2DWithLimits& input, bool new_input) override;
  double getDistance(const geometry_msgs::PoseStamped& pose, const arti_nav_core_msgs::Movement2DWithLimits& movement);

  ros::NodeHandle node_handle_;
  std::shared_ptr<arti_nav_core::Transformer> transformer_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
  dynamic_reconfigure::Server<LocalPlannerObserverConfig> config_server_;
  LocalPlannerObserverConfig config_;
  boost::optional<LimitUtils> limit_utils_;

  std::unique_ptr<arti_costmap_collision_checker::CostmapCollisionCheck> costmap_collision_checker_;

  boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> current_trajectory_;

  size_t current_input_index_ = 0;
};

}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_LOCAL_PLANNER_OBSERVER_H
