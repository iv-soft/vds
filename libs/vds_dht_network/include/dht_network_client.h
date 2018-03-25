#ifndef __VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_H_
#define __VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "service_provider.h"
#include "const_data_buffer.h"

namespace vds {
  class database_transaction;

  namespace dht {
    namespace network {
      enum class message_type_t;
      class _client;

      class client {
      public:
        void start(const service_provider & sp, const const_data_buffer &this_node_id, uint16_t port);
        void stop(const service_provider & sp);

        void save(
          const service_provider & sp,
          database_transaction & t,
          const const_data_buffer & value);

        _client *operator ->() const {
          return this->impl_.get();
        }
      private:
        std::shared_ptr<_client> impl_;
      };
    }
  }
}

#endif //__VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_H_
