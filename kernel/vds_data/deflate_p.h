#ifndef __VDS_DATA_DEFLATE_P_H_
#define __VDS_DATA_DEFLATE_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "zlib.h"
#include "deflate.h"

namespace vds {

  class _deflate_handler
  {
  public:
    _deflate_handler(int compression_level)
    : state_(StateEnum::STATE_NORMAL)
    {
      memset(&this->strm_, 0, sizeof(z_stream));
      if (Z_OK != deflateInit(&this->strm_, compression_level)) {
        throw std::runtime_error("deflateInit failed");
      }
    }

    void update_data(
      const void * input_data,
      size_t input_size,
      void * output_data,
      size_t output_size,
      size_t & readed,
      size_t & written)
    {
      if(this->state_ == StateEnum::STATE_NORMAL){
        this->strm_.next_in = (Bytef *)input_data;
        this->strm_.avail_in = (uInt)input_size;
        this->state_ == StateEnum::STATE_PUSH;
      }
      
      this->strm_.next_out = (Bytef *)output_data;
      this->strm_.avail_out = output_size;
      auto error = ::deflate(&this->strm_, (0 == input_size) ? Z_FINISH : Z_NO_FLUSH);
      
      if(Z_STREAM_ERROR == error){
        throw std::runtime_error("zlib error");
      }

      if (0 == this->strm_.avail_out) {
        readed = 0;
        written = output_size - this->strm_.avail_out;
      }
      else {
        if(this->strm_.avail_in != 0){
          throw std::runtime_error("zlib error");
        }
        
        if(0 == input_size){
          deflateEnd(&this->strm_);
        }
        else {
          this->state_ = StateEnum::STATE_NORMAL;
        }
        
        readed = input_size;
        written = 0;
      }
    }

  private:
    z_stream strm_;
    enum class StateEnum
    {
      STATE_NORMAL,
      STATE_PUSH
    };
    StateEnum state_;
  };
}

#endif // __VDS_DATA_DEFLATE_P_H_
