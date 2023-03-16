/*
Created by clemens on 27.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_MOVE_BASE_PLUGIN_HELPER_H
#define ARTI_MOVE_BASE_PLUGIN_HELPER_H

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#endif

#include <pluginlib/class_loader.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <stdexcept>
#include <string>
#include <utility>

namespace arti_move_base
{

template<class P>
class PluginHelper
{
public:
  PluginHelper(const std::string& package, const std::string& base_class)
  {
    plugin_loader_ = std::make_shared<pluginlib::ClassLoader<P>>(package, base_class);
  }

  P& operator*() const noexcept
  {
    return *(*plugin_);
  }

  P* operator->() const noexcept
  {
    return plugin_->get();
  }

  explicit operator bool() const noexcept
  {
    return static_cast<bool>(plugin_) && static_cast<bool>(*plugin_);
  }

  void reset() noexcept
  {
    plugin_->reset();
    plugin_.reset();
  }

  template<typename... Args>
  void loadAndInitialize(const std::string& plugin_type, Args&& ... args)
  {
    try
    {
      plugin_ = std::make_shared<pluginlib::UniquePtr<P>>(std::move(plugin_loader_->createUniqueInstance(plugin_type)));
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      throw std::invalid_argument("failed to load the '" + plugin_type + "' plugin: " + ex.what());
    }

    if (!plugin_ || !(*plugin_))
    {
      throw std::logic_error("got nullptr when loading the '" + plugin_type + "' plugin");
    }

    try
    {
      (*plugin_)->initialize(std::forward<Args>(args)...);
    }
    catch (const std::exception& ex)
    {
      reset();
      throw std::invalid_argument("failed to initialize the '" + plugin_type + "' plugin: " + ex.what());
    }
  }

protected:
  std::shared_ptr<pluginlib::ClassLoader<P>> plugin_loader_;
  std::shared_ptr<pluginlib::UniquePtr<P>> plugin_;
};

}  // namespace arti_move_base

#endif //ARTI_MOVE_BASE_PLUGIN_HELPER_H
