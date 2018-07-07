#ifndef __VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_H_
#define __VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "service_provider.h"
#include "const_data_buffer.h"
#include "route_statistic.h"
#include "session_statistic.h"

namespace vds {
  class database_transaction;

  namespace dht {
    namespace network {
      enum class message_type_t;
      class _client;

      class client {
      public:
        void start(
            const service_provider & sp,
            const const_data_buffer & this_node_id,
            uint16_t port);
        void stop(const service_provider & sp);

        struct chunk_info {
          const_data_buffer id;
          const_data_buffer key;
          std::vector<const_data_buffer> object_ids;
        };

        chunk_info save(
          const service_provider & sp,
          database_transaction & t,
          const const_data_buffer & value);

        void save(
          const service_provider & sp,
          database_transaction & t,
          const std::string & key,
          const const_data_buffer & value);

        async_task<const_data_buffer> restore(
            const service_provider & sp,
            const chunk_info & block_id);

        async_task<const_data_buffer> restore(
          const service_provider & sp,
          const std::string & key);

        async_task<uint8_t, const_data_buffer> restore_async(
          const service_provider & sp,
          const std::string & key);

        const const_data_buffer & current_node_id() const;

        void get_route_statistics(route_statistic & result);
        void get_session_statistics(session_statistic & session_statistic);

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
