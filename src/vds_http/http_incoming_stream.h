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
    : 
    size_limit_((size_t)-1),
    readed_(0),
    handler_(nullptr)
    {
    }
    
    ~http_incoming_stream()
    {
        std::cout << "http_incoming_stream::~http_incoming_stream\n";
    }
    
    size_t push_data(
      const void * data,
      size_t len)
    {
      size_t l = std::min(
        this->size_limit_ -this->readed_,
        len);
      
      if(0 == l){
        return l;
      }
      
      this->readed_ += l;
      this->handler_->push_data(
        data,
        l
      );
    }

   
    class read_handler
    {
    public:
      virtual ~read_handler()
      {
      }
      
      virtual void push_data(
        const void * data,
        size_t len
      ) = 0;
    };

    void handler(read_handler * value)
    {
      this->handler_ = value;
    }
    
    void limit(size_t size_limit)
    {
      this->size_limit_ = size_limit;
    }
    
  private:
    size_t size_limit_;
    size_t readed_;
    
    read_handler * handler_;
  };
}

#endif // __VDS_HTTP_HTTP_INCOMING_STREAM_H_
