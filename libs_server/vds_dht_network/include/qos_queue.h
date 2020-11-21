#ifndef __VDS_SERVER_DHT_NETWORK_QOS_QUEUE_H__
#define __VDS_SERVER_DHT_NETWORK_QOS_QUEUE_H__

#include <queue>
#include <memory>

#include "shutdown_exception.h"
#include "async_task.h"
#include "service_provider.h"

namespace vds {
  namespace dht {
    namespace network {
      template<typename result_type>
      class quos_queue : public std::enable_shared_from_this<quos_queue<result_type>> {
      public:
        quos_queue(const service_provider* sp)
          : sp_(sp), is_stopping_(false) {
        }

        ~quos_queue() {
        }

        async_task<expected<result_type>> invoke(bool high_priority, lambda_holder_t<async_task<expected<result_type>>> callback) {
          std::unique_lock<std::mutex> lock(this->task_queue_mutex_);
          if (this->is_stopping_) {
            return make_unexpected<shutdown_exception>();
          }

          async_result<expected<result_type>> * result;
          const auto need_start = this->task_queue_.empty() && this->high_task_queue_.empty();
          if (high_priority) {
            this->high_task_queue_.emplace(async_result<expected<result_type>>(), std::move(callback));
            result = &this->high_task_queue_.back().first;
          } else {
            this->task_queue_.emplace(async_result<expected<result_type>>(), std::move(callback));
            result = &this->task_queue_.back().first;
          }

          auto task = result->get_future();
          lock.unlock();


          if (need_start) {
            this->start_queue();
          }

          return std::move(task);
        }

        vds::async_task<vds::expected<void>> prepare_to_stop() {
          std::unique_lock<std::mutex> lock(this->task_queue_mutex_);
          vds_assert(!this->is_stopping_);
          this->is_stopping_ = true;
          if (this->task_queue_.empty() && task_queue_.empty()) {
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
        const service_provider* sp_;
        bool is_stopping_;
        mutable std::mutex task_queue_mutex_;
        std::queue<std::pair<async_result<expected<result_type>>, lambda_holder_t<async_task<expected<result_type>>>>> high_task_queue_;
        std::queue<std::pair<async_result<expected<result_type>>, lambda_holder_t<async_task<expected<result_type>>>>> task_queue_;
        std::unique_ptr<async_result<expected<void>>> empty_query_;

        void start_queue() {
          mt_service::async(this->sp_, [pthis = this->shared_from_this()]() -> void {
              std::unique_lock<std::mutex> lock(pthis->task_queue_mutex_);
              if (!pthis->high_task_queue_.empty()) {
                auto& f = pthis->high_task_queue_.front();
                lock.unlock();

                f.second().then([&f, pthis](expected<result_type> callback_result) -> void {
                  if (callback_result.has_error()) {
                    pthis->sp_->get<logger>()->warning("Core", "%s at process callback", callback_result.error()->what());
                  }
                  f.first.set_value(std::move(callback_result));

                  std::unique_lock<std::mutex> lock(pthis->task_queue_mutex_);
                  pthis->high_task_queue_.pop();
                  if (pthis->high_task_queue_.empty() && pthis->task_queue_.empty()) {
                    if (pthis->empty_query_) {
                      auto r = std::move(pthis->empty_query_);
                      r->set_value(expected<void>());
                    }
                  }
                  else {
                    pthis->start_queue();
                  }
                  });
                return;
              }
              if (!pthis->task_queue_.empty()) {
                auto& f = pthis->task_queue_.front();
                lock.unlock();

                f.second().then([&f, pthis](expected<result_type> callback_result) -> void {
                  if (callback_result.has_error()) {
                    pthis->sp_->get<logger>()->warning("Core", "%s at process callback", callback_result.error()->what());
                  }
                  f.first.set_value(std::move(callback_result));

                  std::unique_lock<std::mutex> lock(pthis->task_queue_mutex_);
                  pthis->task_queue_.pop();
                  if (pthis->high_task_queue_.empty() && pthis->task_queue_.empty()) {
                    if (pthis->empty_query_) {
                      auto r = std::move(pthis->empty_query_);
                      r->set_value(expected<void>());
                    }
                  }
                  else {
                    pthis->start_queue();
                  }
                  });
              }
            });
          }
      };
    }//namespace network 
  }//namespace dht
}//namespace vds

#endif//__VDS_SERVER_DHT_NETWORK_QOS_QUEUE_H__
