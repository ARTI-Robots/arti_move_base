/*
Created by clemens on 02.01.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_ASYNC_UTILS_BUFFERED_RESOURCE_H
#define ARTI_ASYNC_UTILS_BUFFERED_RESOURCE_H

#include <arti_async_utils/interrupt_exception.h>
#include <boost/optional.hpp>
#include <condition_variable>
#include <mutex>
#include <utility>

namespace arti_async_utils
{
template<class T>
class BufferedResource
{
public:
  BufferedResource() = default;
  virtual ~BufferedResource() = default;

  BufferedResource(const BufferedResource<T>& other) = delete;
  BufferedResource(BufferedResource<T>&& other) = delete;
  BufferedResource& operator=(const BufferedResource<T>& other) = delete;
  BufferedResource& operator=(BufferedResource<T>&& other) = delete;

  BufferedResource& operator=(const T& new_resource)
  {
    set(new_resource);
    return *this;
  }

  /**
   * Returns stored resource without clearing buffer.
   *
   * May block if there is currently no resource in the buffer.
   *
   * @return a pair where the first element is the resource, and the second element is a boolean that is true if the
   *         returned resource is new (is different from the last call to peek).
   */
  std::pair<T, bool> peek()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    waitForResource(lock);
    std::pair<T, bool> result = std::make_pair(resource_.value(), new_resource_);
    new_resource_ = false;
    return result;
  }

  /**
   * Returns stored resource and clears buffer.
   *
   * May block if there is currently no resource in the buffer.
   *
   * @return the stored resource.
   */
  T get()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    waitForResource(lock);
    T result = resource_.value();
    resource_.reset();
    new_resource_ = false;
    return result;
  }

  /**
   * Returns stored resource and clears buffer.
   *
   * Returns nothing if there is currently no resource in the buffer.
   *
   * @return the stored resource.
   */
  boost::optional<T> tryGet()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (resource_)
    {
      T result = resource_.value();
      resource_.reset();
      new_resource_ = false;
      return result;
    }
    return boost::none;
  }

  /**
   * Puts a new resource into the buffer.
   *
   * If the buffer already contains a resource, it is replaced.
   *
   * @param new_resource the new resource to put into the buffer.
   */
  void set(const T& new_resource)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resource_.emplace(new_resource);
    new_resource_ = true;
    condition_variable_.notify_all();
  }

  /**
   * Interrupts all threads that are currently waiting for a resource.
   */
  void interrupt()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interrupted_ = true;
    condition_variable_.notify_all();
  }

  /**
   * Clears interrupt flag.
   */
  void clearInterrupt()
  {
    std::lock_guard<std::mutex> set_lock_guard(mutex_);
    interrupted_ = false;
  }

  /**
   * Returns whether this buffer is empty.
   */
  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !resource_;
  }

  /**
   * Clears buffer.
   */
  void clear()
  {
    std::lock_guard<std::mutex> set_lock_guard(mutex_);
    resource_.reset();
    new_resource_ = false;
  }

  template <class O>
  bool fitsType() const
  {
    return std::is_convertible<O,T>::value;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
  boost::optional<T> resource_;
  bool new_resource_ = false;
  bool interrupted_ = false;

  void waitForResource(std::unique_lock<std::mutex>& lock)
  {
    while (!resource_ && !interrupted_)
    {
      condition_variable_.wait(lock);
    }

    if (interrupted_)
    {
      throw InterruptException();
    }
  }
};
}  // namespace arti_async_utils

#endif  // ARTI_ASYNC_UTILS_BUFFERED_RESOURCE_H
