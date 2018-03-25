#ifndef __VDS_DATA_DEFLATE_P_H_
#define __VDS_DATA_DEFLATE_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <cstring>
#include <stdexcept>

#include "zlib.h"
#include "deflate.h"
#include "service_provider.h"

namespace vds {

	class _deflate_handler : public _stream<uint8_t>
	{
	public:
		_deflate_handler(
        const stream<uint8_t> & target,
        int compression_level)
			: target_(target)
		{
			memset(&this->strm_, 0, sizeof(z_stream));
			if (Z_OK != deflateInit(&this->strm_, compression_level)) {
				throw std::runtime_error("deflateInit failed");
			}
		}

		void write(
      const uint8_t * input_data,
      size_t input_size) override
		{
			if (0 == input_size) {
        this->strm_.next_in = (Bytef *)input_data;
        this->strm_.avail_in = (uInt)input_size;

        uint8_t buffer[1024];
        do {
          this->strm_.next_out = (Bytef *)buffer;
          this->strm_.avail_out = sizeof(buffer);
          auto error = ::deflate(&this->strm_, Z_FINISH);

          if (Z_STREAM_ERROR == error) {
            throw std::runtime_error("deflate failed");
          }

          auto written = sizeof(buffer) - this->strm_.avail_out;
          this->target_.write(buffer, written);
        } while (0 == this->strm_.avail_out);

				deflateEnd(&this->strm_);
				this->target_.write(input_data, input_size);
				return;
			}

			this->strm_.next_in = (Bytef *)input_data;
			this->strm_.avail_in = (uInt)input_size;

			uint8_t buffer[1024];
			do {
				this->strm_.next_out = (Bytef *)buffer;
				this->strm_.avail_out = sizeof(buffer);
				auto error = ::deflate(&this->strm_, Z_NO_FLUSH);

				if (Z_STREAM_ERROR == error) {
					throw std::runtime_error("deflate failed");
				}

				auto written = sizeof(buffer) - this->strm_.avail_out;
				this->target_.write(buffer, written);
			} while (0 == this->strm_.avail_out);
		}

	private:
		stream<uint8_t> target_;
		z_stream strm_;
	};

	class _deflate_async_handler : public _stream_async<uint8_t>
  {
  public:
    _deflate_async_handler(
        const stream_async<uint8_t> & target,
        int compression_level)
    : target_(target)
    {
      memset(&this->strm_, 0, sizeof(z_stream));
      if (Z_OK != deflateInit(&this->strm_, compression_level)) {
        throw std::runtime_error("deflateInit failed");
      }
    }

    async_task<> write_async(
      const uint8_t * input_data,
      size_t input_size)
    {
      if(0 == input_size){
        deflateEnd(&this->strm_);
        return this->target_.write_async(input_data, input_size);
      }
      
      this->strm_.next_in = (Bytef *)input_data;
      this->strm_.avail_in = (uInt)input_size;

      return [pthis = this->shared_from_this()](const async_result<> & result){
        static_cast<_deflate_async_handler *>(pthis.get())->continue_write(result);
      };
    }
    
  private:
    stream_async<uint8_t> target_;
    z_stream strm_;
    uint8_t buffer_[1024];
    
    void continue_write(const async_result<> & result)
    {
      if(0 != this->strm_.avail_out){
        result.done();
      }
      else {
        this->strm_.next_out = (Bytef *)this->buffer_;
        this->strm_.avail_out = sizeof(this->buffer_);
        auto error = ::deflate(&this->strm_, Z_FINISH);

        if(Z_STREAM_ERROR == error){
          result.error(std::make_shared<std::runtime_error>("deflate failed"));
          return;
        }

        auto written = sizeof(buffer_) - this->strm_.avail_out;
        this->target_.write_async(buffer_, written)
        .execute(
          [pthis = this->shared_from_this(), result](const std::shared_ptr<std::exception> & ex){
            if(!ex){
              static_cast<_deflate_async_handler *>(pthis.get())->continue_write(result);
            } else {
              result.error(ex); 
            };
          });
      }
    }
  };
}

#endif // __VDS_DATA_DEFLATE_P_H_
