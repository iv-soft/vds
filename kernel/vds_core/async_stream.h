#ifndef __VDS_CORE_ASYNC_STREAM_H_
#define __VDS_CORE_ASYNC_STREAM_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "dataflow.h"
#include "async_task.h"

namespace vds {

  class async_stream
  {
  public:
    async_stream()
      : second_(0), front_(0), back_(0)
    {
    }

    async_task<size_t /*written*/> write_async(const void * data, size_t data_size)
    {
      return create_async_task(
        [this, data, data_size](
          const std::function<void(const service_provider & sp, size_t written)> & done,
          const error_handler & on_error,
          const service_provider & sp) {

        this->continue_write(sp, done, data, data_size);
      });
    }

    async_task<size_t /*readed*/> read_async(const service_provider & sp, void * buffer, size_t buffer_size)
    {
      return create_async_task(
        [this, buffer, buffer_size](
          const std::function<void(const service_provider & sp, size_t readed)> & done,
          const error_handler & on_error,
          const service_provider & sp) {

        this->continue_read(sp, done, buffer, buffer_size);
      });
    }

  private:
    std::mutex buffer_mutex_;
    uint8_t buffer_[4096];
    uint32_t second_;
    uint32_t front_;
    uint32_t back_;

    //            0    second   front    back   buffer_size
    // to read    [...2...]       [...1...]
    // to write            [..2..]         [...1...]

    std::function<void(void)> continue_write_;
    std::function<void(void)> continue_read_;


    void continue_write(
      const service_provider & sp,
      const std::function<void(const service_provider & sp, size_t written)> & done,
      const void * data,
      size_t data_size)
    {
      std::unique_lock<std::mutex> lock(this->buffer_mutex_);
      if (this->back_ < sizeof(this->buffer_)) {
        auto len = sizeof(this->buffer_) - this->back_;
        if (len < data_size) {
          len = data_size;
        }
        memcpy(this->buffer_ + this->front_, data, len);
        done(sp, len);
      }
      else if (this->second_ < this->front_) {
        auto len = this->front_ - this->second_;
        if (len < data_size) {
          len = data_size;
        }
        memcpy(this->buffer_ + this->second_, data, len);
        done(sp, len);
      }
      else {
        this->continue_write_ = std::bind(this, &async_stream::continue_write, sp, done, data, data_size);

        if (this->continue_read_) {
          imt_service::async(sp, this->continue_read_);
        }
      }
    }

    void continue_read(
      const service_provider & sp,
      const std::function<void(const service_provider & sp, size_t readed)> & done,
      void * buffer,
      size_t buffer_size)
    {
      std::unique_lock<std::mutex> lock(this->buffer_mutex_);

      if (this->front_ < this->back_) {
        auto len = this->back_ - this->front_;
        if (len < buffer_size) {
          len = buffer_size;
        }
        memcpy(this->buffer_ + this->front_, buffer, len);
        done(sp, len);
      }
      else {
        this->continue_read_ = std::bind(this, &async_stream::continue_read, sp, done, buffer, buffer_size);

        if (this->continue_write_) {
          imt_service::async(sp, this->continue_write_);
        }
      }
    }

  };
}

#endif // __VDS_CORE_ASYNC_STREAM_H_
 