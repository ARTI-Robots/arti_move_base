/*
Created by clemens on 02.01.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_LOCKABLE_RESOURCE_H
#define ARTI_ASYNC_UTILS_LOCKABLE_RESOURCE_H

#include <boost/optional.hpp>
#include <mutex>

namespace arti_async_utils
{
template<class T>
class LockableResourceHandle;

template<class T>
class LockableResource
{
public:
  LockableResource() = default;
  virtual ~LockableResource() = default;

  LockableResource(const LockableResource<T>& other) = delete;
  LockableResource& operator=(const LockableResource<T>& other) = delete;

  LockableResource& operator=(const T& resource)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resource_.emplace(resource);
    return *this;
  }

  LockableResourceHandle<T> getResourceHandle()
  {
    return LockableResourceHandle<T>(this);
  }

private:
  std::mutex mutex_;
  boost::optional<T> resource_;

  friend class LockableResourceHandle<T>;
};

template<class T>
class LockableResourceHandle
{
public:
  explicit LockableResourceHandle(LockableResource<T>* lockable_resource)
    : lockable_resource_(lockable_resource), lock_(lockable_resource_->mutex_)
  {
  }

  LockableResourceHandle(LockableResourceHandle<T>& other)
  {
    lock_.swap(other.lock_);
    lockable_resource_ = other.lockable_resource_;
  }

  LockableResourceHandle(LockableResourceHandle<T>&& other) noexcept
  {
    lock_.swap(other.lock_);
    lockable_resource_ = std::move(other.lockable_resource_);
  }

  LockableResourceHandle& operator=(const LockableResourceHandle<T>& other) = delete;

  T* operator->()
  {
    return &lockable_resource_->resource_.value();
  }

  bool hasResource()
  {
    return lockable_resource_->resource_.is_initialized();
  }

private:
  LockableResource<T>* lockable_resource_;
  std::unique_lock<std::mutex> lock_;
};
}  // namespace arti_async_utils

#endif  // ARTI_ASYNC_UTILS_LOCKABLE_RESOURCE_H
