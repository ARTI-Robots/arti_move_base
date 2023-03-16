//
// Created by abuchegger on 28.05.18.
//
#include <arti_async_utils/pipeline_stage_base.h>
#include <arti_async_utils/interrupt_exception.h>
#include <functional>

namespace arti_async_utils
{
PipelineStageBase::PipelineStageBase(const std::string& pipeline_stage_name)
  : pipeline_stage_name_(pipeline_stage_name)
{
}

PipelineStageBase::~PipelineStageBase()
{
  stop();
}

void PipelineStageBase::start()
{
  std::unique_lock<std::mutex> execution_lock(execution_mutex_);
  if (!should_execute_)
  {
    should_execute_ = true;
    execution_thread_ = std::thread(std::bind(&PipelineStageBase::execute, this));
  }
}

void PipelineStageBase::stop()
{
  std::unique_lock<std::mutex> execution_lock(execution_mutex_);
  should_execute_ = false;
  execution_lock.unlock();

  interrupt();
  execution_thread_.join();
}

void PipelineStageBase::execute()
{
  while (!shouldExecute())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  while (shouldExecute())
  {
    try
    {
      std::unique_lock<std::mutex> execution_lock(execution_mutex_);
      performExecution();
    }
    catch (const InterruptException& exception)
    {
      // nothing to do just stop execution
    }
  }
}

bool PipelineStageBase::shouldExecute()
{
  std::unique_lock<std::mutex> execution_lock(execution_mutex_);
  return should_execute_;
}
}
