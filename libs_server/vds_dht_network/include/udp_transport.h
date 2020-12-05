#ifndef __VDS_DHT_NETWORK_UDP_TRANSPORT_H_
#define __VDS_DHT_NETWORK_UDP_TRANSPORT_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "service_provider.h"
#include "udp_socket.h"
#include "legacy.h"
#include "debug_mutex.h"
#include "iudp_transport.h"
#include "thread_apartment.h"
#include "network_address.h"

namespace vds {
  struct session_statistic;
}

namespace vds {
  namespace dht {
    namespace network {
      class udp_transport : public iudp_transport {
      public:
        static constexpr uint8_t PROTOCOL_VERSION = 0;

        udp_transport();
        udp_transport(const udp_transport&) = delete;
        udp_transport(udp_transport&&) = delete;
        ~udp_transport();

        expected<void> start(
          const service_provider * sp,
          const std::shared_ptr<asymmetric_public_key> & node_public_key,
          const std::shared_ptr<asymmetric_private_key> & node_key,
          const network_address & bind_interface,
          bool dev_network);

        void stop() override;

        vds::async_task<vds::expected<void>> write_async(udp_datagram && datagram) override;
        vds::async_task<vds::expected<void>> try_handshake( const std::string& address) override;
        expected<void> broadcast_handshake() override;

        async_task<expected<void>> on_timer() override;

        const const_data_buffer& this_node_id() const {
          return this->this_node_id_;
        }

        void get_session_statistics(session_statistic& session_statistic);

      private:
        const service_provider * sp_;
        const_data_buffer this_node_id_;
        std::shared_ptr<asymmetric_public_key> node_public_key_;
        std::shared_ptr<asymmetric_private_key> node_key_;
        udp_server server_;

        std::shared_ptr<thread_apartment> send_thread_;

        struct quota_state_t {
          uint32_t sent_data_;
          uint32_t send_quota_;
        };

        std::map<network_address, quota_state_t> quota_states_;
        std::list<std::pair<udp_datagram, std::shared_ptr<vds::async_result<vds::expected<void>>>>> send_queue_;
        uint32_t total_sent_;
        uint32_t prev_sent_;
        std::mutex send_mutex_;


#ifdef _DEBUG
#ifndef _WIN32
        pid_t owner_id_;
#else
        DWORD owner_id_;
#endif//_WIN32
#endif//_DEBUG

        uint32_t MAGIC_LABEL;


        struct session_state {
          std::mutex session_mutex_;

          bool blocked_;
          std::chrono::steady_clock::time_point update_time_;
          std::shared_ptr<class dht_session> session_;
          const_data_buffer session_key_;

          session_state()
          : blocked_(false) {
          }
        };
        mutable std::shared_mutex sessions_mutex_;
        std::map<network_address, session_state> sessions_;

        vds::async_task<vds::expected<bool>> read_handler(expected<udp_datagram> result);
      };
    }
  }
}


#endif //__VDS_DHT_NETWORK_UDP_TRANSPORT_H_
