#ifndef __VDS_CORE_THREAD_APARTMENT_H_
#define __VDS_CORE_THREAD_APARTMENT_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <queue>
#include <functional>
#include <mutex>
#include "mt_service.h"
#include "vds_debug.h"
#include "logger.h"

namespace vds {

  class thread_apartment : public std::enable_shared_from_this<thread_apartment> {
  public:
    thread_apartment(const service_provider * sp)
      : sp_(sp), is_stopping_(false){
    }

    ~thread_apartment() {
      //vds_assert(this->task_queue_.empty());
    }

    void schedule(lambda_holder_t<expected<void>> callback) {
      std::unique_lock<std::mutex> lock(this->task_queue_mutex_);
      vds_assert(!this->is_stopping_);

      const auto need_start = this->task_queue_.empty();
      this->task_queue_.push(std::move(callback));
      lock.unlock();

      if(need_start) {
        mt_service::async(this->sp_, [pthis = this->shared_from_this()]() {
          for (;;) {
            std::unique_lock<std::mutex> lock(pthis->task_queue_mutex_);
            auto & f = pthis->task_queue_.front();
            lock.unlock();

            auto callback_result = f();
            if(callback_result.has_error()) {
              pthis->sp_->get<logger>()->warning("Core", "%s at process callback", callback_result.error()->what());
            }

            lock.lock();
            pthis->task_queue_.pop();
            if (pthis->task_queue_.empty()) {
              if(pthis->empty_query_) {
                auto r = std::move(pthis->empty_query_);
                r->set_value(expected<void>());
              }
              break;
            }
            lock.unlock();
          }
        });
      }
    }

    vds::async_task<vds::expected<void>> prepare_to_stop() {
      std::unique_lock<std::mutex> lock(this->task_queue_mutex_);
      vds_assert(!this->is_stopping_);
      this->is_stopping_ = true;
      if(task_queue_.empty()) {
        auto r = vds::async_result<vds::expected<void>>();
        r.set_value(expected<void>());
        return r.get_future();
      }

      this->empty_query_ = std::make_unique<vds::async_result<vds::expected<void>>>();
      return this->empty_query_->get_future();
    }

    bool is_ready_to_stop() const {
      std::unique_lock<std::mutex> lock(this->task_queue_mutex_);
      return task_queue_.empty();
    }

    size_t size() const {
      return this->task_queue_.size();
    }

  private:
    const service_provider * sp_;
    bool is_stopping_;
    mutable std::mutex task_queue_mutex_;
    std::queue<lambda_holder_t<expected<void>>> task_queue_;
    std::unique_ptr<async_result<expected<void>>> empty_query_;
  };
}

#endif //__VDS_CORE_THREAD_APARTMENT_H_
