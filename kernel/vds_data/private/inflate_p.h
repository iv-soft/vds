#ifndef __VDS_DATA_INFLATE_P_H_
#define __VDS_DATA_INFLATE_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "zlib.h"
#include "service_provider.h"

namespace vds {

  //Decompress stream
  class _inflate_handler
  {
  public:
    expected<void> create(
      const std::shared_ptr<stream_output_async<uint8_t>> & target)
    {
      this->target_ = target;

      memset(&this->strm_, 0, sizeof(z_stream));
      if (Z_OK != inflateInit(&this->strm_)) {
        return make_unexpected<std::runtime_error>("inflateInit failed");
      }

      return expected<void>();
    }

    vds::async_task<expected<void>> write_async( const uint8_t * input_data, size_t input_size)
    {
      if(0 == input_size){
        inflateEnd(&this->strm_);
        CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(input_data, input_size));
        co_return expected<void>();
      }
      
      this->strm_.next_in = (Bytef *)input_data;
      this->strm_.avail_in = (uInt)input_size;

      uint8_t buffer[1024];
      do{
        this->strm_.next_out = (Bytef *)buffer;
        this->strm_.avail_out = sizeof(buffer);
        auto error = ::inflate(&this->strm_, Z_NO_FLUSH);

        if (Z_STREAM_ERROR == error || Z_NEED_DICT == error || Z_DATA_ERROR == error || Z_MEM_ERROR == error) {
          co_return make_unexpected<std::runtime_error>("inflate failed");
        }

        auto written = sizeof(buffer) - this->strm_.avail_out;
        if (0 != written) {
          CHECK_EXPECTED_ASYNC(co_await this->target_->write_async(buffer, written));
        }
      } while(0 == this->strm_.avail_out);

      co_return expected<void>();
    }

  private:
    std::shared_ptr<stream_output_async<uint8_t>> target_;
    z_stream strm_;
  };
}

#endif // __VDS_DATA_INFLATE_H_
