#ifndef __VDS_CORE_ASYNC_TASK_H_
#define __VDS_CORE_ASYNC_TASK_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include <experimental/coroutine>
#include <future>
#include <chrono>
#include "vds_debug.h"
#include "expected.h"
#include "func_utils.h"

namespace vds {
  template <typename result_type>
  class async_task;

  template <typename result_type>
  class async_result;

  class service_provider;

  class imt_service
  {
  public:
	  static imt_service * get_current();
	  static void async(const service_provider * sp, lambda_holder_t<void> handler);

	  void do_async(lambda_holder_t<void> handler);
  };
  
  template <typename result_type>
  class _async_task_value {
  public:
    virtual ~_async_task_value(){}
    virtual result_type && get() = 0;
  };

  template <>
  class _async_task_value<void> {
  public:
    virtual ~_async_task_value() {}
    virtual void get() = 0;
  };

  template <typename result_type>
  class _async_task_return_value : public _async_task_value<result_type>{
  public:

    template<typename init_type>
    _async_task_return_value(init_type && v)
    : value_(std::forward<init_type>(v)) {
    }

    result_type && get() override {
      return std::move(this->value_);
    }

  private:
    result_type value_;
  };

  template <>
  class _async_task_return_value<void> : public _async_task_value<void> {
  public:

    void get() override {
    }
  };

  template <typename result_type>
  class _async_task_throw_exception : public _async_task_value<result_type> {
  public:
    _async_task_throw_exception(std::exception_ptr ex)
      : ex_(ex) {
    }

    result_type && get() override {
      std::rethrow_exception(this->ex_);
    }

  private:
    std::exception_ptr ex_;
  };

  template <>
  class _async_task_throw_exception<void> : public _async_task_value<void> {
  public:
    _async_task_throw_exception(std::exception_ptr ex)
      : ex_(ex) {
    }

    void get() override {
      std::rethrow_exception(this->ex_);
    }

  private:
    std::exception_ptr ex_;
  };

  template <typename result_type>
  class _async_task_state {
  public:
    _async_task_state() {
    }

    template<class _Rep, class _Period>
    std::future_status wait_for(std::chrono::duration<_Rep, _Period> timeout) {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      if (this->value_) {
        return std::future_status::ready;
      }

      this->value_cond_.wait_for(lock, timeout);
      return (!this->value_) ? std::future_status::timeout : std::future_status::ready;
    }

    bool is_ready() {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      return (!this->value_) ? false : true;
    }


    result_type && get() {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      while (!this->value_) {
        this->value_cond_.wait(lock);
      }
      return std::move(this->value_->get());
    }

    void then(lambda_holder_t<void> f) {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      if (!this->value_) {
        this->then_function_ = std::move(f);
      }
      else {
        lock.unlock();

        f();
      }
    }

    void set_value(_async_task_value<result_type> * v) {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      vds_assert(!this->value_);
      this->value_.reset(v);
      this->value_cond_.notify_all();
	  if (this->then_function_) {
		  lock.unlock();

		  auto mt = imt_service::get_current();
		  if (nullptr != mt) {
			  mt->do_async([f = std::move(this->then_function_)]() {
						f();
			  });
		  }
		  else {
			  this->then_function_();
		  }
      }
    }
  private:
    std::mutex value_mutex_;
    std::condition_variable value_cond_;
    std::unique_ptr<_async_task_value<result_type>> value_;
    lambda_holder_t<void> then_function_;
  };

  template <>
  class _async_task_state<void> {
  public:
    template<class _Rep, class _Period>
    std::future_status wait_for(std::chrono::duration<_Rep, _Period> timeout) {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      if (this->value_) {
        return std::future_status::ready;
      }

      this->value_cond_.wait_for(lock, timeout);
      return (!this->value_) ? std::future_status::timeout : std::future_status::ready;
    }

    bool is_ready() {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      return (!this->value_) ? false : true;
    }


    void get() {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      while (!this->value_) {
        this->value_cond_.wait(lock);
      }
      this->value_->get();
    }

    void then(lambda_holder_t<void> f) {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      if (!this->value_) {
        this->then_function_ = std::move(f);
      }
      else {
        lock.unlock();

        f();
      }
    }

    void set_value(_async_task_value<void> * v) {
      std::unique_lock<std::mutex> lock(this->value_mutex_);
      vds_assert(!this->value_);
      this->value_.reset(v);
      this->value_cond_.notify_all();
      if (this->then_function_) {
        lock.unlock();

		auto mt = imt_service::get_current();
		if (nullptr != mt) {
      mt->do_async([f = std::move(this->then_function_)]() {
          f();
      });
    }
		else {
			this->then_function_();
		}
	  }
    }

  private:
    std::mutex value_mutex_;
    std::condition_variable value_cond_;
    std::unique_ptr<_async_task_value<void>> value_;
    lambda_holder_t<void> then_function_;
  };

  template <typename result_type>
  class [[nodiscard]] async_task {
  public:
    async_task() = default;
    async_task(const async_task &) = delete;
    async_task(async_task&& original) = default;

    async_task(result_type && result)
      : state_(std::make_shared<_async_task_state<result_type>>()) {
      this->state_->set_value(new _async_task_return_value<result_type>(std::move(result)));
    }
    async_task(unexpected && error)
    : state_(std::make_shared<_async_task_state<result_type>>()){
      this->state_->set_value(new _async_task_return_value<result_type>(std::move(error)));
    }

    async_task & operator = (const async_task &) = delete;
    async_task& operator = (async_task&& original) = default;

    async_task(const std::shared_ptr<_async_task_state<result_type>> & state)
    : state_(state) {
    }

    ~async_task() {
    }

	bool has_state() const {
		return (nullptr != this->state_.get());
	}

    template<class _Rep, class _Period>
    std::future_status wait_for(std::chrono::duration<_Rep, _Period> timeout) const {
      return this->state_->wait_for(timeout);
    }

    [[nodiscard]]
    auto && get() {
      return std::move(this->state_->get());
    }

    bool await_ready() const noexcept {
      return this->state_->is_ready();
    }

    [[nodiscard]]
    auto && await_resume() const {
      return std::move(this->state_->get());
    }

    void await_suspend(std::experimental::coroutine_handle<> _ResumeCb)
    {
      this->then([_ResumeCb]() mutable {
        _ResumeCb();
      });
    }

    void then(lambda_holder_t<void> && f) {
      this->state_->then([f_ = std::move(f), s = this->state_]() {
        f_();
      });
    }
    
    void then(lambda_holder_t<void, result_type> && f) {
      this->state_->then([f_ = std::move(f), s = std::move(this->state_)]() {
        f_(std::move(s->get()));
      });
    }

    //void detach() {
    //  auto s = std::move(this->state_);
    //  s->then([s]() {
    //    try {
    //      s->get();
    //    }
    //    catch(...) {          
    //    }
    //  });
    //}
  private:
    std::shared_ptr<_async_task_state<result_type>> state_;
  };  

  template <>
  class async_task<void> {
  public:
    async_task() = delete;
    async_task(const async_task &) = delete;
    async_task(async_task &&) = default;

    async_task & operator = (const async_task &) = delete;
    async_task & operator = (async_task &&) = default;

    async_task(const std::shared_ptr<_async_task_state<void>> & state)
      : state_(state) {
    }


    template<class _Rep, class _Period>
    std::future_status wait_for(std::chrono::duration<_Rep, _Period> timeout) const {
      return this->state_->wait_for(timeout);
    }

    void get() {
      this->state_->get();
    }

    bool await_ready() const noexcept {
      return this->state_->is_ready();
    }

    void await_resume() const {
      this->state_->get();
    }

    void await_suspend(std::experimental::coroutine_handle<> _ResumeCb)
    {
      this->then([_ResumeCb]() mutable {
        _ResumeCb();
      });
    }

    void then(lambda_holder_t<void> f) {
      this->state_->then(std::move(f));
    }

  private:
    std::shared_ptr<_async_task_state<void>> state_;
  };

  template <typename result_type>
  class async_result {
  public:
    async_result()
      : state_(new _async_task_state<result_type>()) {
    }

    async_task<result_type> get_future() {
      return async_task<result_type>(this->state_);
    }

    template<typename init_type>
    void set_value(init_type && v) {
      this->state_->set_value(new _async_task_return_value<result_type>(std::forward<init_type>(v)));
    }

    void set_exception(std::exception_ptr ex) {
      this->state_->set_value(new _async_task_throw_exception<result_type>(ex));
    }

    void unhandled_exception() {
      this->set_exception(std::current_exception());
    }

  private:
    std::shared_ptr<_async_task_state<result_type>> state_;
  };

  template <>
  class async_result<void> {
  public:
    async_result()
    : state_(new _async_task_state<void>()){
    }

    async_task<void> get_future() {
      return async_task<void>(this->state_);
    }

    void set_value() {
      this->state_->set_value(new _async_task_return_value<void>());
    }

    void set_exception(std::exception_ptr ex) {
      this->state_->set_value(new _async_task_throw_exception<void>(ex));
    }

  private:
    std::shared_ptr<_async_task_state<void>> state_;
  };


#ifndef _WIN32
  template<typename T>
  struct awaiter {
    vds::async_task<T> _future;
  public:
    explicit awaiter(vds::async_task<T> &&f) noexcept : _future(std::move(f)) {
    }

    bool await_ready() const noexcept {
      return _future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    template<typename U>
    void await_suspend(std::experimental::coroutine_handle<U> hndl) noexcept {
      this->_future.then([hndl]() mutable {
        hndl();
      });
    }

    T && await_resume() {
      return std::move(_future.get());
    }
  };

  template<>
  struct awaiter<void> {
    vds::async_task<void> _future;
  public:
    explicit awaiter(vds::async_task<void> &&f) noexcept
        : _future(std::move(f)) {}

    bool await_ready() const noexcept {
      return _future.await_ready();
    }

    template<typename U>
    void await_suspend(std::experimental::coroutine_handle<U> hndl) noexcept {
      this->_future.then([hndl]() mutable {
        hndl();
      });
    }

    void await_resume() { _future.get(); }
  };

  template<typename T>
  inline auto operator co_await(async_task<T> f) noexcept {
    return awaiter<T>(std::move(f));
  }

  inline auto operator co_await(async_task<void> f) noexcept {
    return awaiter<void>(std::move(f));
  }

#endif//_WIN32
}

namespace std {
  namespace experimental {
    template<typename R, typename... Args>
    struct coroutine_traits<vds::async_task<R>, Args...> {
      struct promise_type {
        vds::async_result<R> p;

        auto get_return_object() {
          return p.get_future();
        }

        std::experimental::suspend_never initial_suspend() {
          vds::thread_protect::check();
          return {};
        }

        std::experimental::suspend_never final_suspend() {
          return {};
        }

        //void set_exception(std::exception_ptr e) {
        //  p.set_exception(std::move(e));
        //}

        template<typename U>
        void return_value(U &&u) {
          p.set_value(std::forward<U>(u));
        }

        void unhandled_exception() {
          p.set_exception(std::current_exception());
        }
      };
    };
    template<typename... Args>
    struct coroutine_traits<vds::async_task<void>, Args...> {
      struct promise_type {
        vds::async_result<void> p;

        auto get_return_object() {
          return p.get_future();
        }

        std::experimental::suspend_never initial_suspend() {
          vds::thread_protect::check();
          return {};
        }

        std::experimental::suspend_never final_suspend() {
          return {};
        }

        //void set_exception(std::exception_ptr e) {
        //  p.set_exception(std::move(e));
        //}

        void return_void() {
          p.set_value();
        }
        void unhandled_exception() {
          p.set_exception(std::current_exception());
        }
      };
    };
  };
}


#ifdef _WIN32
namespace std {
  template <typename  T>
  inline bool await_ready(vds::async_task<T> & _future)
  {
    return _future.await_ready();
  }

  template <typename  T>
  inline void await_suspend(vds::async_task<T> & _future,
    std::experimental::coroutine_handle<> _ResumeCb)
  {
    _future.then([_ResumeCb]() {
      _ResumeCb();
    });
  }

  template<typename T>
  [[nodiscard]]
  inline T && await_resume(vds::async_task<T> & _future)
  {
    return std::move(_future.get());
  }

  inline void await_resume(vds::async_task<void> & _future)
  {
    _future.get();
  }
}

#endif

#endif // __VDS_CORE_ASYNC_TASK_H_
 
