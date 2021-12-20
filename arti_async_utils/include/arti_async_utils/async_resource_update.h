/*
Created by clemens on 02.01.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_ASYNC_RESOURCE_UPDATE_H
#define ARTI_ASYNC_UTILS_ASYNC_RESOURCE_UPDATE_H

#include <arti_async_utils/buffered_resource.h>
#include <arti_async_utils/lockable_resource.h>
#include <thread>

namespace arti_async_utils
{
template<class R, class O>
class AsyncResourceUpdate
{
protected:
  AsyncResourceUpdate()
    : execution_thread_(&AsyncResourceUpdate::run, this)
  {
  }

  virtual R createResource(const O& original_resource) = 0;

  LockableResourceHandle<R> getResourceHandle()
  {
    return resource_.getResourceHandle();
  }

public:
  void update(const O& original_resource)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!update_pending_)
    {
      resource_update_ = original_resource;
    }
  }

  virtual ~AsyncResourceUpdate()
  {
    execute_ = false;
    resource_update_.interrupt();
    execution_thread_.join();
  }

private:
  std::mutex mutex_;
  bool update_pending_ = false;
  bool execute_ = true;
  LockableResource<R> resource_;
  BufferedResource<O> resource_update_;
  std::thread execution_thread_;

  void run()
  {
    while (execute_)
    {
      O original_resource = resource_update_.get();

      if (execute_)
      {
        setUpdatePending(true);
        resource_ = createResource(original_resource);
        setUpdatePending(false);
      }
    }
  }

  void setUpdatePending(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    update_pending_ = value;
  }
};
}  // namespace arti_async_utils

#endif  // ARTI_ASYNC_UTILS_ASYNC_RESOURCE_UPDATE_H
