/*
Created by clemens on 19.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_PIPELINE_STAGE_H
#define ARTI_ASYNC_UTILS_PIPELINE_STAGE_H

#include <arti_async_utils/buffered_resource.h>
#include <arti_async_utils/pipeline_stage_base.h>
#include <functional>
#include <memory>
#include <ros/console.h>
#include <type_traits>
#include <vector>

namespace arti_async_utils
{
namespace detail
{
template<typename Signature, typename... Args>
void callCallback(const std::function<Signature>& callback, Args... args)
{
  if (callback)
  {
    try
    {
      callback(std::forward<Args>(args)...);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("failed to call callback: " << ex.what());
    }
  }
}
}

template<class I>
class PipelineInputStage
{
public:
  typedef I Input;
  typedef std::shared_ptr<arti_async_utils::BufferedResource<I>> InputBufferPtr;
  typedef std::function<void(const I& input)> InputChangedCallback;

  InputBufferPtr getInput()
  {
    return input_;
  }

  void setInputChangedCB(const InputChangedCallback& input_changed_cb)
  {
    input_changed_cb_ = input_changed_cb;
  }

protected:
  explicit PipelineInputStage(const InputBufferPtr& input)
    : input_(input)
  {
  }

  virtual ~PipelineInputStage() = default;

  void signalInputChanged(const I& input)
  {
    detail::callCallback(input_changed_cb_, input);
  }

  InputBufferPtr input_;

private:
  InputChangedCallback input_changed_cb_;
};

template<class O>
class PipelineOutputStage
{
public:
  typedef O Output;
  typedef std::shared_ptr<arti_async_utils::BufferedResource<O>> OutputBufferPtr;
  typedef std::function<void(const O& output)> OutputChangedCallback;

  OutputBufferPtr getOutput()
  {
    return output_;
  }

  void setOutputChangedCB(const OutputChangedCallback& output_changed_cb)
  {
    output_changed_cb_ = output_changed_cb;
  }

protected:
  explicit PipelineOutputStage(const OutputBufferPtr& output)
    : output_(output)
  {
  }

  virtual ~PipelineOutputStage() = default;

  void signalOutputChanged(const O& output)
  {
    detail::callCallback(output_changed_cb_, output);
  }

  OutputBufferPtr output_;

private:
  OutputChangedCallback output_changed_cb_;
};

template<class I, class O, typename E>
class PipelineStage : public PipelineStageBase, public PipelineInputStage<I>, public PipelineOutputStage<O>
{
public:
  using typename PipelineInputStage<I>::Input;
  using typename PipelineInputStage<I>::InputBufferPtr;
  using typename PipelineOutputStage<O>::Output;
  using typename PipelineOutputStage<O>::OutputBufferPtr;

  typedef E Error;
  typedef std::function<void(const E& error, const boost::optional<I>& input)> ErrorCallback;
  typedef std::function<void(const boost::optional<I>& input)> SuccessCallback;

  ~PipelineStage() override = default;

  void cancel() override
  {
    this->input_->clear();
    this->output_->clear();
  }

protected:
  PipelineStage(
    const InputBufferPtr& input, const OutputBufferPtr& output, const ErrorCallback& error_cb,
    const SuccessCallback& success_cb, const SuccessCallback& close_to_success_cb)
    : PipelineStage<I, O, E>(input, output, createVectorIfValid(error_cb), createVectorIfValid(success_cb),
                             createVectorIfValid(close_to_success_cb))
  {
  }

  PipelineStage(
    const InputBufferPtr& input, const OutputBufferPtr& output, const std::vector<ErrorCallback>& error_cbs,
    const std::vector<SuccessCallback>& success_cbs, const std::vector<SuccessCallback>& close_to_success_cbs)
    : PipelineInputStage<I>(input), PipelineOutputStage<O>(output), error_cbs_(error_cbs), success_cbs_(success_cbs),
      close_to_success_cbs_(close_to_success_cbs)
  {
  }

  void interrupt() override
  {
    this->input_->interrupt();
    this->output_->interrupt();
  }

  void callSuccessCB()
  {
    const boost::optional<I> input = this->getLastInput();
    this->input_->clear();
    this->output_->clear();
    for (const auto& cb : success_cbs_)
    {
      detail::callCallback(cb, input);
    }
  }

  void callCloseToSuccessCB()
  {
    const boost::optional<I> input = this->getLastInput();
    for (const auto& cb : close_to_success_cbs_)
    {
      detail::callCallback(cb, input);
    }
  }

  void callErrorCB(const E& error)
  {
    const boost::optional<I> input = this->getLastInput();
    this->input_->clear();
    this->output_->clear();
    for (const auto& cb : error_cbs_)
    {
      detail::callCallback(cb, error, input);
    }
  }

  template<class T>
  bool fitsType() const
  {
    return this->input_->template fitsType<T>();
  }

  virtual boost::optional<I> getLastInput() = 0;

private:
  static_assert(std::is_enum<E>::value || std::is_class<E>::value, "Template argument E is neither an enum or a class");

  template<typename T>
  static std::vector<T> createVectorIfValid(const T& value)
  {
    if (value)
    {
      return {{value}};
    }
    return {};
  }

  std::vector<ErrorCallback> error_cbs_;
  std::vector<SuccessCallback> success_cbs_;
  std::vector<SuccessCallback> close_to_success_cbs_;
};
}  // namespace arti_async_utils

#endif  // ARTI_ASYNC_UTILS_PIPELINE_STAGE_H
