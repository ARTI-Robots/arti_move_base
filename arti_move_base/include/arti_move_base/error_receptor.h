/*
Created by clemens on 20.03.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_ERROR_RECEPTOR_H
#define ARTI_MOVE_BASE_ERROR_RECEPTOR_H

#include <boost/optional.hpp>
#include <cstddef>

namespace arti_move_base
{
template<class I>
class ErrorHandlerBase;

template<class I>
class ErrorReceptorBase
{
public:
  void successCB(const boost::optional<I>& input)
  {
    error_count_ = 0;
    handleSuccess(input);
  }

  void closeToSuccessCB(const boost::optional<I>& input)
  {
    error_count_ = 0;
    handleCloseToSuccess(input);
  }

protected:
  explicit ErrorReceptorBase(size_t error_propagation_count)
    : error_count_(0), error_propagation_count_(error_propagation_count)
  {
  }

  virtual ~ErrorReceptorBase() = default;

  void setErrorPropagationCount(const size_t error_propagation_count)
  {
    error_propagation_count_ = error_propagation_count;
  }

  virtual void handleSuccess(const boost::optional<I>& input) = 0;

  virtual void handleCloseToSuccess(const boost::optional<I>& /*input*/)
  {
  }

  virtual void propagationError() = 0;

  bool increaseErrorCountAndPropagateIfMaximum()
  {
    error_count_++;
    if (error_count_ >= error_propagation_count_)
    {
      propagationError();
      
      error_count_ = 0;
      //error_count_ = error_propagation_count_;
      return true;
    }
    return false;
  }

  size_t error_count_;
  size_t error_propagation_count_;

  friend class ErrorHandlerBase<I>;
};

template<class I>
class ErrorHandlerBase
{
protected:
  explicit ErrorHandlerBase(ErrorReceptorBase<I>* error_receptor)
    : error_receptor_(error_receptor)
  {
  }

  virtual ~ErrorHandlerBase() = default;

  bool increaseErrorCountAndPropagateIfMaximum()
  {
    return error_receptor_->increaseErrorCountAndPropagateIfMaximum();
  }

private:
  ErrorReceptorBase<I>* error_receptor_;
};

template<class I, class E>
class ErrorHandler : public ErrorHandlerBase<I>
{
public:
  void errorCB(const E& error, const boost::optional<I>& input)
  {
    if (!this->increaseErrorCountAndPropagateIfMaximum())
    {
      handleError(error, input);
    }
  }

protected:
  explicit ErrorHandler(ErrorReceptorBase<I>* error_receptor)
    : ErrorHandlerBase<I>(error_receptor)
  {
  }

  virtual void handleError(const E& error, const boost::optional<I>& input) = 0;
};

template<class... Es>
class ErrorReceptor;

template<class I>
class ErrorReceptor<I> : public ErrorReceptorBase<I>
{
protected:
  explicit ErrorReceptor(size_t error_propagation_count)
    : ErrorReceptorBase<I>(error_propagation_count)
  {}
};

template<class I, class E, class... Es>
class ErrorReceptor<I, E, Es...> : public ErrorReceptor<I, Es...>, public ErrorHandler<I, E>
{
protected:
  explicit ErrorReceptor(size_t error_propagation_count)
    : ErrorReceptor<I, Es...>(error_propagation_count), ErrorHandler<I, E>(this)
  {
  }
};
}  // namespace arti_move_base

#endif  // ARTI_MOVE_BASE_ERROR_RECEPTOR_H
