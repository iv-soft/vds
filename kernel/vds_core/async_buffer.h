//#ifndef __VDS_CORE_ASYNC_STREAM_H_
//#define __VDS_CORE_ASYNC_STREAM_H_
//
///*
//Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
//All rights reserved
//*/
//#include <mutex>
//#include <memory>
//
//
//#include "mt_service.h"
//#include "not_mutex.h"
//#include "stream.h"
//#include "vds_debug.h"
//
//namespace vds {
//  
//  template <typename item_t> 
//  class continuous_buffer : public std::enable_shared_from_this<continuous_buffer<item_t>>
//  {
//  public:
//    using item_type = item_t;
//
//    continuous_buffer(const service_provider * sp)
//      : sp_(sp), second_(0), front_(0), back_(0), eof_(false), eof_readed_(false)
//    {
//    }
//
//    ~continuous_buffer()
//    {
////#ifdef _DEBUG
////#pragma warning(disable: 4297)
////      if (0 != this->second_ || 0 != this->front_ || 0 != this->back_ || this->continue_read_ || this->continue_write_
////        || !this->eof_ || !this->eof_readed_) {
////        if (!std::current_exception()) {
////          throw std::runtime_error("continuous_buffer::~continuous_buffer logic error");
////        }
////      }
////#pragma warning(default: 4297)
////#endif//_DEBUG
//    }
//
//    vds::async_task<vds::expected<void>> write_async(      
//      const item_type * data,
//      size_t data_size)
//    {
//      if (0 == data_size && this->eof_) {
//        co_return vds::make_unexpected<std::runtime_error>("Logic error");
//      }
//
//      this->in_mutex_.lock();
//
//      if (0 == data_size) {
//        if (this->eof_) {
//          this->in_mutex_.unlock();
//          co_return vds::make_unexpected<std::runtime_error>("continuous_buffer::write_all_async logic error");
//        }
//        this->eof_ = true;
//
//        if (this->continue_read_) {
//          std::function<void(void)> f;
//          this->continue_read_.swap(f);
//#ifdef _DEBUG
//          mt_service::async(this->sp_,
//            [f]() {
//            f();
//          });
//#else
//          mt_service::async(this->sp_, f);
//#endif
//        }
//
//        this->in_mutex_.unlock();
//      }
//      else {
//        this->in_mutex_.unlock();
//
//        CHECK_EXPECTED_ASYNC(co_await this->write_all(data, data_size));
//      }
//
//      co_return expected<void>();
//    }
//
//    vds::async_task<vds::expected<size_t /*readed*/>> read_async( item_type * buffer, size_t buffer_size)
//    {
//      vds_assert(0 != buffer_size);
//
//      if (this->continue_read_) {
//        co_return vds::make_unexpected<std::runtime_error>("Logic error 29");
//      }
//
//      this->out_mutex_.lock();
//
//      size_t readed;
//      GET_EXPECTED_VALUE_ASYNC(readed, co_await this->continue_read(buffer, buffer_size));
//
//      this->out_mutex_.unlock();
//
//      co_return readed;
//    }
//
//    vds::async_task<vds::expected<const_data_buffer>> read_all()
//    {
//      auto buffer = std::make_shared<std::tuple<resizable_data_buffer, item_type[1024]>>();
//      co_return co_await this->read_all(buffer);
//    }
//
//  private:
//    const service_provider * sp_;
//    not_mutex in_mutex_;
//    not_mutex out_mutex_;
//    std::mutex buffer_mutex_;
//    item_type buffer_[4096];
//    uint32_t second_;
//    uint32_t front_;
//    uint32_t back_;
//    bool eof_;
//    bool eof_readed_;
//    std::string eof_stack_;
//    //            0    second   front    back   buffer_size
//    // to read    [...2...]       [...1...]
//    // to write            [..2..]         [...1...]
//
//
//    std::function<void(void)> continue_write_;
//    std::function<void(void)> continue_read_;
//
//    vds::async_task<vds::expected<size_t /*len*/>> continue_write(
//      
//      const item_type * data,
//      size_t data_size)
//    {
//      this->buffer_mutex_.lock();
//      if (this->back_ < sizeof(this->buffer_) / sizeof(this->buffer_[0])) {
//        size_t len = sizeof(this->buffer_) / sizeof(this->buffer_[0]) - this->back_;
//        if (len > data_size) {
//          len = data_size;
//        }
//
//        std::copy(data, data + len, this->buffer_ + this->back_);
//        this->back_ += len;
//
//        if (this->continue_read_) {
//          std::function<void(void)> f;
//          this->continue_read_.swap(f);
//          mt_service::async(this->sp_, f);
//        }
//
//        this->buffer_mutex_.unlock();
//
//        co_return len;
//      }
//      else if (this->second_ < this->front_) {
//        auto len = this->front_ - this->second_;
//        if (len > data_size) {
//          len = data_size;
//        }
//        std::copy(data, data + len, this->buffer_ + this->second_);
//        this->second_ += len;
//
//        if (this->continue_read_) {
//          std::function<void(void)> f;
//          this->continue_read_.swap(f);
//          mt_service::async(this->sp_, f);
//        }
//
//        this->buffer_mutex_.unlock();
//
//        co_return len;
//      }
//
//      auto result = std::make_shared<vds::async_result<vds::expected<size_t>>>();
//      continue_write_ = [pthis = this->shared_from_this(), result, data, data_size](){
//        auto size = pthis->continue_write(data, data_size).get();
//        result->set_value(std::move(size));
//      };
//      this->buffer_mutex_.unlock();
//
//      co_return co_await result->get_future();
//    }
//
//    vds::async_task<vds::expected<void>> write_all(
//      
//      const item_type * data,
//      size_t data_size)
//    {
//      for (;;) {
//        size_t len;
//        GET_EXPECTED_VALUE_ASYNC(len, co_await this->continue_write(data, data_size));
//
//        if (len == data_size) {
//          co_return expected<void>();
//        }
//        else {
//          data += len;
//          data_size -= len;
//        }
//      }
//    }
//
//    vds::async_task<vds::expected<size_t /*readed*/>> continue_read(
//      
//      item_type * buffer,
//      size_t buffer_size)
//    {
//      vds_assert(0 != buffer_size);
//      this->buffer_mutex_.lock();
//
//      if (this->front_ < this->back_) {
//        size_t len = this->back_ - this->front_;
//        if (len > buffer_size) {
//          len = buffer_size;
//        }
//        std::copy(this->buffer_ + this->front_, this->buffer_ + this->front_ + len, buffer);
//        this->front_ += len;
//
//        if (this->front_ == this->back_) {
//          this->front_ = 0;
//          this->back_ = this->second_;
//          this->second_ = 0;
//        }
//
//        if (this->continue_write_) {
//          std::function<void(void)> f;
//          this->continue_write_.swap(f);
//          mt_service::async(this->sp_, f);
//        }
//        vds_assert(0 != len);
//        this->buffer_mutex_.unlock();
//
//        co_return len;
//      }
//      else if (this->eof_) {
//        vds_assert(!this->eof_readed_);
//        this->eof_readed_ = true;
//        this->buffer_mutex_.unlock();
//
//        co_return 0;
//      }
//
//      auto result = std::make_shared<vds::async_result<vds::expected<size_t>>>();
//      this->continue_read_ = [pthis = this->shared_from_this(), result, buffer, buffer_size](){
//        auto size = pthis->continue_read(buffer, buffer_size).get();
//        result->set_value(std::move(size));
//      };
//      this->buffer_mutex_.unlock();
//
//      co_return co_await result->get_future();
//    }
//    vds::async_task<vds::expected<const_data_buffer>> read_all(
//      
//      const std::shared_ptr<std::tuple<resizable_data_buffer, item_type[1024]>> & buffer)
//    {
//      for (;;) {
//        size_t readed;
//        GET_EXPECTED_VALUE_ASYNC(readed, co_await this->read_async(std::get<1>(*buffer), 1024));
//
//        if (0 == readed) {
//          co_return std::get<0>(*buffer).move_data();
//        }
//        else {
//          std::get<0>(*buffer).add(std::get<1>(*buffer), readed);
//        }
//      }
//    }
//
//    expected<void> reset()
//    {
//      if (!this->eof_ || 0 != this->second_ || 0 != this->front_ || 0 != this->back_ || this->continue_read_ || this->continue_write_) {
//        return vds::make_unexpected<std::runtime_error>("continuous_buffer::reset logic error");
//      }
//
//      this->eof_ = false;
//      this->eof_readed_ = false;
//      return expected<void>();
//    }
//  };
//  
//  template <typename item_t>
//  class continuous_stream_input_async : public stream_input_async<item_t> {
//  public:
//    continuous_stream_input_async(const std::shared_ptr<continuous_buffer<item_t>> & buffer)
//    : buffer_(buffer) {      
//    }
//
//    vds::async_task<vds::expected<size_t>> read_async(
//      uint8_t * buffer,
//      size_t len) override {
//      return this->buffer_->read_async(buffer, len);
//    }
//
//
//  private:
//    std::shared_ptr<continuous_buffer<item_t>> buffer_;
//  };
//  
//  template <typename item_t>
//  class continuous_stream_output_async : public stream_output_async<item_t> {
//  public:
//    continuous_stream_output_async(const std::shared_ptr<continuous_buffer<item_t>> & buffer)
//      : buffer_(buffer) {
//    }
//
//    vds::async_task<vds::expected<void>> write_async(
//      
//      const uint8_t *data,
//      size_t len) override {
//      return this->buffer_->write_async(data, len);
//    }
//  private:
//    std::shared_ptr<continuous_buffer<item_t>> buffer_;
//  };
//}
//
//#endif // __VDS_CORE_ASYNC_STREAM_H_
// 
