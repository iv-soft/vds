#ifndef __VDS_NETWORK_OUTPUT_NETWORK_STREAM_H_
#define __VDS_NETWORK_OUTPUT_NETWORK_STREAM_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "write_socket_task.h"

namespace vds {
  class output_network_stream
  {
  public:
    output_network_stream(const network_socket & s)
      : s_(s)
    {
    }
    
    template <typename context_type>
    class handler : public sequence_step<context_type, void(void)>
    {
      using base_class = sequence_step<context_type, void(void)>;
    public:
      handler(
        const context_type & context,
        const output_network_stream & args)
      : base_class(context),
        s_(args.s_.handle()),
        task_(this->prev, this->error)        
      {
      }
      
      void operator()(
        const void * data,
        size_t len
      ) {
        if(0 == len) {
          this->next();
        }
        else {
          this->task_.set_data(data, len);
          this->task_.schedule(this->s_);
        }          
      }
      
    private:
      network_socket::SOCKET_HANDLE s_;
      write_socket_task<
        typename base_class::prev_step_t,
        typename base_class::error_method_t> task_;
    };
  private:
    const network_socket & s_;
  };
}

#endif // __VDS_NETWORK_OUTPUT_NETWORK_STREAM_H_