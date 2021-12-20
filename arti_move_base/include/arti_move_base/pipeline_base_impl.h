/*
Created by clemens on 6/6/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_PIPELINE_BASE_IMPL_H
#define ARTI_MOVE_BASE_PIPELINE_BASE_IMPL_H

#include <arti_move_base/pipeline_base.h>
#include <mutex>

namespace arti_move_base
{
template<class O>
class PipelineBaseImpl : public PipelineBase
{
public:
  PipelineBaseImpl(const ros::NodeHandle& node_handle, const std::string& output_topic)
    : node_handle_(node_handle)
  {
    output_publisher_ = node_handle_.advertise<O>(output_topic, 1);
  }

  void createPipeline(PipelineBuilder& builder) override
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    planner_pipeline_ = builder.getPipeline<O>();
    planner_pipeline_.getOutputStage()->setOutputChangedCB(
      std::bind(&PipelineBaseImpl::processPipelineOutput, this, std::placeholders::_1));
    planner_pipeline_.start();
  }

  void cancel() override
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    planner_pipeline_.cancel();
    output_publisher_.publish(O());
    start_time_ = {};
  }

  void setInput(const arti_nav_core_msgs::Movement2DGoalWithConstraints& goal) override
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    start_time_ = ros::Time::now();
    planner_pipeline_.template skipTill<arti_nav_core_msgs::Movement2DGoalWithConstraints>()->set(goal);
  }

  void setInput(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) override
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    start_time_ = ros::Time::now();
    planner_pipeline_.getInput()->set(goal);
  }

  void setInput(const arti_nav_core_msgs::Trajectory2DWithLimits& goal) override
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    start_time_ = ros::Time::now();
    planner_pipeline_.template skipTill<arti_nav_core_msgs::Trajectory2DWithLimits>()->set(goal);
  }

private:
  void processPipelineOutput(const O& output)
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    const ros::Time now = ros::Time::now();

    output_publisher_.publish(output);

    if (!start_time_.isZero())
    {
      const double time_difference = (now - start_time_).toSec();
      if (time_difference > 0.2)
      {
        ROS_WARN_STREAM("time difference in command sending too big (" << time_difference << "s)");
      }
    }

    start_time_ = now;
  }

  std::mutex mutex_;
  arti_async_utils::Pipeline<arti_nav_core_msgs::Pose2DStampedWithLimits, O> planner_pipeline_;

  ros::NodeHandle node_handle_;

  ros::Publisher output_publisher_;
  ros::Time start_time_;
};
}

#endif //ARTI_MOVE_BASE_PIPELINE_BASE_IMPL_H
