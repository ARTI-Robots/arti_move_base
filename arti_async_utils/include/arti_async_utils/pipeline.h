/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_PIPELINE_H
#define ARTI_ASYNC_UTILS_PIPELINE_H

#include <arti_async_utils/buffered_resource.h>
#include <arti_async_utils/pipeline_stage_base.h>
#include <arti_async_utils/pipeline_stage.h>
#include <memory>
#include <set>

namespace arti_async_utils
{

template<class I, class O>
class Pipeline
{
public:
  typedef std::shared_ptr<arti_async_utils::BufferedResource<I>> InputBufferPtr;
  typedef std::shared_ptr<arti_async_utils::BufferedResource<O>> OutputBufferPtr;

  Pipeline() = default;
  Pipeline(const InputBufferPtr &input, const OutputBufferPtr &output,
           std::vector<std::shared_ptr<PipelineStageBase>> &&pipeline_stages)
    : input_(input), output_(output), pipeline_stages_(pipeline_stages)
  {
  }

  virtual ~Pipeline() = default;

  InputBufferPtr getInput()
  {
    return input_;
  }

  std::shared_ptr<PipelineInputStage<I>> getInputStage()
  {
    if (pipeline_stages_.empty())
    {
      throw std::logic_error("pipeline is empty");
    }

    const auto input_stage = std::dynamic_pointer_cast<PipelineInputStage<I>>(pipeline_stages_.front());
    if (!input_stage)
    {
      throw std::logic_error("pipeline input stage does not match input type");
    }

    return input_stage;
  }

  OutputBufferPtr getOutput()
  {
    return output_;
  }

  std::shared_ptr<PipelineOutputStage<O>> getOutputStage()
  {
    if (pipeline_stages_.empty())
    {
      throw std::logic_error("pipeline is empty");
    }

    const auto output_stage = std::dynamic_pointer_cast<PipelineOutputStage<O>>(pipeline_stages_.back());
    if (!output_stage)
    {
      throw std::logic_error("pipeline output stage does not match output type");
    }

    return output_stage;
  }

  void start()
  {
    for (const auto &stage : pipeline_stages_)
    {
      stage->start();
    }
  }

  void stop()
  {
    for (const auto &stage : pipeline_stages_)
    {
      stage->stop();
    }
  }

  void cancel()
  {
    for (const auto &stage : pipeline_stages_)
    {
      stage->cancel();
    }
  }

  template <class T>
  std::shared_ptr<arti_async_utils::BufferedResource<T>> skipTill()
  {
    for (const auto &stage : pipeline_stages_)
    {
      if (std::dynamic_pointer_cast<PipelineInputStage<T>>(stage))
      {
        return std::dynamic_pointer_cast<PipelineInputStage<T>>(stage)->getInput();
      }
    }

    return std::shared_ptr<arti_async_utils::BufferedResource<T>>();
  }

private:
  InputBufferPtr input_;
  OutputBufferPtr output_;
  std::vector<std::shared_ptr<PipelineStageBase>> pipeline_stages_;
};
}  // namespace arti_async_utils

#endif  // ARTI_ASYNC_UTILS_PIPELINE_H
