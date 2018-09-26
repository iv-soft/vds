#ifndef __VDS_NETWORK_NETWORK_SOCKET_H_
#define __VDS_NETWORK_NETWORK_SOCKET_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <memory>

#include "async_buffer.h"
#include "network_address.h"

namespace vds {

  class tcp_network_socket : public stream_output_async<uint8_t>
  {
  public:
    tcp_network_socket();

    static tcp_network_socket connect(
      const service_provider & sp,
      const network_address & address);

    std::shared_ptr<vds::stream_input_async<uint8_t>> start(
        const service_provider & sp) const;

    void close();

    class _tcp_network_socket * operator ->() const;

    std::future<void> write_async(
        const service_provider &sp,
        const item_type *data,
        size_t len) override;

  private:
    friend class _tcp_network_socket;
    tcp_network_socket(class _tcp_network_socket * impl);

    _tcp_network_socket * impl_;
  };
}

#endif//__VDS_NETWORK_NETWORK_SOCKET_H_
