/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_PIPELINE_STAGE_BASE_H
#define ARTI_ASYNC_UTILS_PIPELINE_STAGE_BASE_H

#include <mutex>
#include <thread>

namespace arti_async_utils
{
class PipelineStageBase
{
public:
  PipelineStageBase() = default;
  explicit PipelineStageBase(const PipelineStageBase&) = delete;
  explicit PipelineStageBase(PipelineStageBase&&) = delete;
  virtual ~PipelineStageBase();
  PipelineStageBase& operator=(const PipelineStageBase&) = delete;
  PipelineStageBase& operator=(PipelineStageBase&&) = delete;

  void start();
  void stop();
  virtual void cancel() = 0;

protected:
  virtual void performExecution() = 0;
  virtual void interrupt() = 0;

private:
  void execute();
  bool shouldExecute();

  std::mutex execution_mutex_;
  bool should_execute_ = false;
  std::thread execution_thread_;
};
}  // namespace arti_async_utils


#endif  // ARTI_ASYNC_UTILS_PIPELINE_STAGE_BASE_H
