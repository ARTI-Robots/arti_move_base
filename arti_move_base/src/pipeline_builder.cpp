/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_async_utils/pipeline_stage.h>
#include <arti_move_base/pipeline_builder.h>
#include <stdexcept>

namespace arti_move_base
{
void PipelineBuilder::addNetworkPlanner(
  const ros::NodeHandle& node_handle, const NetworkPlanner::ErrorCallback& error_cb,
  const NetworkPlanner::SuccessCallback& success_cb,
  const std::shared_ptr<arti_nav_core::Transformer>& transformer
)
{
  pipeline_stages_.emplace_back(
    std::make_shared<NetworkPlanner>(node_handle, getInput<NetworkPlanner::Input>(),
                                     getOutput<NetworkPlanner::Output>(), makeVector({error_cb}),
                                     makeVector({success_cb}), makeVector<NetworkPlanner::SuccessCallback>({}),
                                     transformer));
}

void PipelineBuilder::addGlobalPlanner(
  const ros::NodeHandle& node_handle, const GlobalPlanner::ErrorCallback& error_cb,
  const GlobalPlanner::SuccessCallback& success_cb,
  const std::shared_ptr<arti_nav_core::Transformer>& transformer,
  const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
  pipeline_stages_.emplace_back(std::make_shared<GlobalPlanner>(
    node_handle, getInput<GlobalPlanner::Input>(), getOutput<GlobalPlanner::Output>(),
    makeVector({error_cb, getErrorCallback<GlobalPlanner::Error, GlobalPlanner::Input>()}),
    makeVector({success_cb, getSuccessCallback<GlobalPlanner::Input>()}),
    makeVector({getCloseToSuccessCallback<GlobalPlanner::Input>()}), transformer,
    costmap));
}

void PipelineBuilder::addLocalPlanner(
  const ros::NodeHandle& node_handle, const LocalPlanner::ErrorCallback& error_cb,
  const LocalPlanner::SuccessCallback& success_cb, const std::shared_ptr<arti_nav_core::Transformer>& transformer,
  const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
  pipeline_stages_.emplace_back(std::make_shared<LocalPlanner>(
    node_handle, getInput<LocalPlanner::Input>(), getOutput<LocalPlanner::Output>(),
    makeVector({error_cb, getErrorCallback<LocalPlanner::Error, LocalPlanner::Input>()}),
    makeVector({success_cb, getSuccessCallback<LocalPlanner::Input>()}),
    makeVector({getCloseToSuccessCallback<LocalPlanner::Input>()}),
    transformer, costmap));
}

template<class O>
void PipelineBuilder::addPathFollower(
  const ros::NodeHandle& node_handle, const typename PathFollower<O>::ErrorCallback& error_cb,
  const typename PathFollower<O>::SuccessCallback& success_cb,
  const std::shared_ptr<arti_nav_core::Transformer>& transformer,
  const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
  pipeline_stages_.emplace_back(std::make_shared<PathFollower<O>>(
    node_handle, getInput<typename PathFollower<O>::Input>(), getOutput<typename PathFollower<O>::Output>(),
    makeVector({error_cb, getErrorCallback<typename PathFollower<O>::Error, typename PathFollower<O>::Input>()}),
    makeVector({success_cb, getSuccessCallback<typename PathFollower<O>::Input>()}),
    makeVector({getCloseToSuccessCallback<typename PathFollower<O>::Input>()}), transformer, costmap));
}

template<class O>
arti_async_utils::Pipeline<arti_nav_core_msgs::Pose2DStampedWithLimits, O> PipelineBuilder::getPipeline()
{
  return doGetPipeline<arti_nav_core_msgs::Pose2DStampedWithLimits, O>();
}

template<class I>
std::shared_ptr<arti_async_utils::BufferedResource<I>> PipelineBuilder::getInput()
{
  if (pipeline_stages_.empty())
  {
    return std::make_shared<arti_async_utils::BufferedResource<I>>();
  }

  const auto prev_stage = std::dynamic_pointer_cast<arti_async_utils::PipelineOutputStage<I>>(
    pipeline_stages_.back());

  if (!prev_stage)
  {
    throw std::runtime_error("Previous stage has invalid output type");
  }

  return prev_stage->getOutput();
}

template<class O>
std::shared_ptr<arti_async_utils::BufferedResource<O>> PipelineBuilder::getOutput()
{
  return std::make_shared<arti_async_utils::BufferedResource<O>>();
}

template<class E, class I>
std::function<void(const E&, const boost::optional<I>&)> PipelineBuilder::getErrorCallback()
{
  if (pipeline_stages_.empty())
  {
    throw std::runtime_error("Previous stage is missing, so no callback could be found");
  }

  const auto error_handler = std::dynamic_pointer_cast<ErrorHandler<I, E>>(pipeline_stages_.back());

  if (!error_handler)
  {
    throw std::runtime_error("Previous stage does not handle the requested error type");
  }

  return std::bind(&ErrorHandler<I, E>::errorCB, error_handler, std::placeholders::_1, std::placeholders::_2);
}

template<class I>
std::function<void(const boost::optional<I>&)> PipelineBuilder::getSuccessCallback()
{
  if (pipeline_stages_.empty())
  {
    throw std::runtime_error("Previous stage is missing, so no callback could be found");
  }

  const auto error_receptor = std::dynamic_pointer_cast<ErrorReceptorBase<I>>(pipeline_stages_.back());

  if (!error_receptor)
  {
    throw std::runtime_error("Previous stage does not handle success");
  }

  return std::bind(&ErrorReceptorBase<I>::successCB, error_receptor, std::placeholders::_1);
}

template<class I>
std::function<void(const boost::optional<I>&)> PipelineBuilder::getCloseToSuccessCallback()
{
  if (pipeline_stages_.empty())
  {
    throw std::runtime_error("Previous stage is missing, so no callback could be found");
  }

  const auto error_receptor = std::dynamic_pointer_cast<ErrorReceptorBase<I>>(pipeline_stages_.back());

  if (!error_receptor)
  {
    throw std::runtime_error("Previous stage does not handle close-to-success");
  }

  return std::bind(&ErrorReceptorBase<I>::closeToSuccessCB, error_receptor, std::placeholders::_1);
}

template<class I, class O>
arti_async_utils::Pipeline<I, O> PipelineBuilder::doGetPipeline()
{
  if (pipeline_stages_.empty())
  {
    throw std::runtime_error("Cannot create empty pipeline");
  }

  const auto front_stage = std::dynamic_pointer_cast<arti_async_utils::PipelineInputStage<I>>(pipeline_stages_.front());
  const auto back_stage = std::dynamic_pointer_cast<arti_async_utils::PipelineOutputStage<O>>(pipeline_stages_.back());

  if (!front_stage)
  {
    throw std::runtime_error("Front stage has invalid input type");
  }

  if (!back_stage)
  {
    throw std::runtime_error("Back stage has invalid output type");
  }

  std::vector<std::shared_ptr<arti_async_utils::PipelineStageBase>> stage_set = pipeline_stages_;
  pipeline_stages_.clear();

  return arti_async_utils::Pipeline<I, O>(
    front_stage->getInput(), back_stage->getOutput(), std::move(stage_set));
}

template<typename T>
std::vector<T> PipelineBuilder::makeVector(std::initializer_list<T> elements)
{
  std::vector<T> vector;
  vector.reserve(elements.size());
  for (auto& element : elements)
  {
    if (element)
    {
      vector.template emplace_back(std::move(element));
    }
  }
  return vector;
}

// Explicit instantiation definitions (they make sure that these template methods are compiled for the given template
// types):
template void PipelineBuilder::addPathFollower<geometry_msgs::Twist>(
  const ros::NodeHandle& node_handle, const typename PathFollower<geometry_msgs::Twist>::ErrorCallback& error_cb,
  const typename PathFollower<geometry_msgs::Twist>::SuccessCallback& success_cb,
  const std::shared_ptr<arti_nav_core::Transformer>& transformer,
  const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap);

template void PipelineBuilder::addPathFollower<ackermann_msgs::AckermannDrive>(
  const ros::NodeHandle& node_handle,
  const typename PathFollower<ackermann_msgs::AckermannDrive>::ErrorCallback& error_cb,
  const typename PathFollower<ackermann_msgs::AckermannDrive>::SuccessCallback& success_cb,
  const std::shared_ptr<arti_nav_core::Transformer>& transformer,
  const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap);

template arti_async_utils::Pipeline<arti_nav_core_msgs::Pose2DStampedWithLimits, geometry_msgs::Twist>
PipelineBuilder::getPipeline<geometry_msgs::Twist>();

template arti_async_utils::Pipeline<arti_nav_core_msgs::Pose2DStampedWithLimits, ackermann_msgs::AckermannDrive>
PipelineBuilder::getPipeline<ackermann_msgs::AckermannDrive>();

}  // namespace arti_move_base
