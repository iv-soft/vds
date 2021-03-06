#ifndef __VDS_HTTP_HTTP_INCOMING_STREAM_H_
#define __VDS_HTTP_HTTP_INCOMING_STREAM_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "http_response.h"

namespace vds {
  
  class http_incoming_stream
  {
  public:
    http_incoming_stream()
    : handler_(nullptr)
    {
    }

    vds::expected<void> push_data(      
      const void * data,
      size_t len)
    {
      if (nullptr == this->handler_) {
        if (0 < len) {
          return vds::make_unexpected<std::logic_error>("Read handler for http_incoming_stream have not set");
        }
      }
      else {
        return this->handler_->push_data(
          data,
          len
        );
      }

      return expected<void>();
    }    
   
    class read_handler
    {
    public:
      virtual ~read_handler()
      {
      }
      
      virtual expected<void> push_data(
        const void * data,
        size_t len
      ) = 0;
    };

    void handler(read_handler * value)
    {
      this->handler_ = value;
    }
    
  private:
    
    read_handler * handler_;
  };
}

#endif // __VDS_HTTP_HTTP_INCOMING_STREAM_H_
