/*
Created by clemens on 19.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_EVENT_PIPELINE_STAGE_H
#define ARTI_ASYNC_UTILS_EVENT_PIPELINE_STAGE_H

#include <arti_async_utils/pipeline_stage.h>
#include <boost/optional.hpp>
#include <chrono>
#include <ros/console.h>

namespace arti_async_utils
{
template<class I, class O, typename E>
class EventPipelineStage : public PipelineStage<I, O, E>
{
public:
  using typename PipelineStage<I, O, E>::InputBufferPtr;
  using typename PipelineStage<I, O, E>::OutputBufferPtr;
  using typename PipelineStage<I, O, E>::ErrorCallback;
  using typename PipelineStage<I, O, E>::SuccessCallback;

protected:
  typedef std::chrono::system_clock Clock;

  EventPipelineStage(const InputBufferPtr& input, const OutputBufferPtr& output, const ErrorCallback& error_cb,
                     const SuccessCallback& success_cb, const SuccessCallback& close_to_success_cb)
    : PipelineStage<I, O, E>(input, output, error_cb, success_cb, close_to_success_cb)
  {
  }

  EventPipelineStage(const InputBufferPtr& input, const OutputBufferPtr& output,
                     const std::vector<ErrorCallback> &error_cbs,
                     const std::vector<SuccessCallback>& success_cbs,
                     const std::vector<SuccessCallback>& close_to_success_cbs)
    : PipelineStage<I, O, E>(input, output, error_cbs, success_cbs, close_to_success_cbs)
  {
  }

  ~EventPipelineStage() override = default;

  virtual boost::optional<O> performTask(const I& input) = 0;

  void performExecution() final
  {
    last_input_ = this->input_->get();
    this->signalInputChanged(last_input_);

    const Clock::time_point start_time = Clock::now();
    const boost::optional<O> task_output = performTask(last_input_);
    const Clock::time_point end_time = Clock::now();

    const Clock::duration task_duration = end_time - start_time;
    if (task_duration > std::chrono::milliseconds(100))
    {
      ROS_ERROR_STREAM("performTask in event pipeline stage took too long ("
                         << std::chrono::duration<double>(task_duration).count() << "s)");
    }

    if (task_output)
    {
      this->output_->set(task_output.value());
      this->signalOutputChanged(task_output.value());
    }
  }

  boost::optional<I> getLastInput() override
  {
    return last_input_;
  }

private:
  I last_input_;
};
}  // namespace arti_async_utils

#endif  // ARTI_ASYNC_UTILS_EVENT_PIPELINE_STAGE_H
