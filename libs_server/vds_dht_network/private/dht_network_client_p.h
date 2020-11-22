#ifndef __VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_P_H_
#define __VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "service_provider.h"
#include "const_data_buffer.h"
#include "dht_session.h"
#include "dht_route.h"
#include "chunk.h"
#include "sync_process.h"
#include "udp_transport.h"
#include "imessage_map.h"

class mock_server;

namespace vds {
  namespace dht {
    namespace messages {
      //class sync_replica_operations_response;
      //class sync_replica_operations_request;
      //class sync_leader_broadcast_response;
      //class sync_leader_broadcast_request;
      //class sync_add_message_request;
      class sync_replica_data;
      class sync_replica_request;
      class high_priority_replica_data;
      class high_priority_replica_request;
      class dht_pong;
      class dht_ping;
      class transaction_log_record;
      class transaction_log_request;
      class dht_find_node_response;
      class dht_find_node;
    }
  }

  class database_transaction;

  namespace dht {
    namespace network {
      class client_save_stream;

      class _client : public std::enable_shared_from_this<_client> {
      public:

        _client(
          const service_provider * sp,
          const const_data_buffer & this_node_id);

        static expected<std::shared_ptr<_client>> create(
          const service_provider * sp,
          const std::shared_ptr<asymmetric_public_key> & node_public_key,
          const std::shared_ptr<asymmetric_private_key>& node_key,
          uint16_t port,
          bool dev_network);

        void add_transport(std::shared_ptr<iudp_transport> transport);

        expected<void> start();
        void stop();
        void get_neighbors(
          
          std::list<std::shared_ptr<dht_route::node>>& result);
        
        expected<void> on_new_session(
          database_read_transaction& t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks,
          const const_data_buffer& partner_id);

        static expected<bool> save_replica_data(
          const service_provider * sp,
          database_transaction& t,
          const const_data_buffer& data_hash,
          const const_data_buffer& data,
          const const_data_buffer& owner);

        expected<void> save_data(
          const service_provider* sp,
          database_transaction & t,
          const const_data_buffer& data,
          const const_data_buffer& owner);

        expected<std::vector<vds::const_data_buffer>> save_temp(
          database_transaction& t,
          const const_data_buffer& value_id,
          const const_data_buffer& value,
          uint32_t * replica_size);

        //expected<std::shared_ptr<client_save_stream>> create_save_stream();

        const const_data_buffer& current_node_id() const {
          return this->route_.current_node_id();
        }

        async_task<expected<bool>> apply_message(
          const messages::dht_find_node& message,
          const imessage_map::message_info_t& message_info);

        async_task<expected<bool>> apply_message(
          
          const messages::dht_find_node_response& message,
          const imessage_map::message_info_t& message_info);

        async_task<expected<bool>> apply_message(
          
          const messages::dht_ping& message,
          const imessage_map::message_info_t& message_info);

        async_task<expected<bool>> apply_message(
          
          const messages::dht_pong& message,
          const imessage_map::message_info_t& message_info);

        //Sync messages
        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_new_election_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_new_election_response& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_add_message_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_leader_broadcast_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_leader_broadcast_response& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_replica_operations_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_replica_operations_response& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_looking_storage_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_looking_storage_response& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_snapshot_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_snapshot_response& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_offer_send_replica_operation_request& message,
        //  const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_offer_remove_replica_operation_request& message,
        //  const imessage_map::message_info_t& message_info);

        expected<bool> apply_message(
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks,
          const messages::sync_replica_request& message,
          const imessage_map::message_info_t& message_info);

        expected<bool> apply_message(
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks,
          const messages::sync_replica_data& message,
          const imessage_map::message_info_t& message_info);

        expected<bool> apply_message(
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>>& final_tasks,
          const messages::high_priority_replica_request& message,
          const imessage_map::message_info_t& message_info);

        expected<bool> apply_message(
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>>& final_tasks,
          const messages::high_priority_replica_data& message,
          const imessage_map::message_info_t& message_info);

        //expected<bool> apply_message(
        //  database_transaction& t,
        //  std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //  const messages::sync_replica_query_operations_request & message,
        //  const imessage_map::message_info_t& message_info);

        //
        template <typename message_type>
        async_task<expected<void>> send(
          const const_data_buffer& node_id,
          expected<message_type> && message) {
          CHECK_EXPECTED_ERROR(message);
          return this->send(node_id, message_type::message_id, message_serialize(message.value()));
        }

        async_task<expected<void>> send(
          const const_data_buffer& node_id,
          message_type_t message_id,
          expected<const_data_buffer> && message);

        template <typename message_type>
        async_task<expected<void>> redirect(
          const_data_buffer node_id,
          std::vector<const_data_buffer> hops,
          expected<message_type> message) {
          CHECK_EXPECTED_ERROR(message);
          return this->redirect(std::move(node_id), std::move(hops), message_type::message_id, message_serialize(message.value()));
        }

        async_task<expected<void>> redirect(
          const_data_buffer node_id,
          std::vector<const_data_buffer> hops,
          message_type_t message_id,
          expected<const_data_buffer> message);


        async_task<expected<void>> find_nodes(
            const const_data_buffer& node_id,
            size_t radius);

        template <typename message_type>
        async_task<expected<void>> send_near(
          const const_data_buffer& node_id,
          size_t radius,
          expected<message_type> && message,
          const std::function<expected<bool>(const dht_route::node& node)>& filter) {
          GET_EXPECTED(message_data, message_serialize(message.value()));
          return this->send_near(node_id, radius, message_type::message_id, message_data, filter);
        }

        async_task<expected<void>> for_near(
          const const_data_buffer& target_node_id,
          size_t max_count,
          const std::function<expected<bool>(const dht_route::node & node)>& filter,
          lambda_holder_t<vds::async_task<vds::expected<bool>>, const std::shared_ptr<dht_route::node>&> callback);

        expected<void> for_near_sync(
          const const_data_buffer& target_node_id,
          size_t max_count,
          const std::function<expected<bool>(const dht_route::node & node)>& filter,
          lambda_holder_t<vds::expected<bool>, const std::shared_ptr<dht_route::node>&> callback);

        template <typename message_type>
        vds::async_task<vds::expected<void>> send_neighbors(
          expected<message_type> && message) {

          CHECK_EXPECTED_ERROR(message);

          return this->send_neighbors(message_type::message_id, message_serialize(message.value()));
        }

        async_task<expected<void>> add_session(
          const std::shared_ptr<dht_session>& session,
          uint8_t hops);

        async_task<expected<uint8_t>> prepare_restore(
          std::vector<const_data_buffer> replicas_hashes,
          bool high_priority);

        async_task<expected<const_data_buffer>> restore(
          std::vector<const_data_buffer> replicas_hashes,
          bool high_priority);

        expected<client::block_info_t> prepare_restore(
          database_read_transaction & t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks,
          const std::vector<const_data_buffer>& replicas_hashes,
          bool high_priority);

        async_task<vds::expected<uint8_t>> restore_async(
          const std::vector<const_data_buffer>& replicas_hashes,
          bool high_priority,
          std::shared_ptr<const_data_buffer> result = std::shared_ptr<const_data_buffer>());

        void get_route_statistics(route_statistic& result);
        void get_session_statistics(session_statistic& session_statistic);

        void add_route(
          const std::vector<const_data_buffer>& hops,
          const std::shared_ptr<dht_session>& session);

        void add_route(
          const const_data_buffer& source_node,
          uint16_t hops,
          const std::shared_ptr<dht_session>& session);

        void remove_session(
          
          const std::shared_ptr<dht_session>& session);

        //expected<void> add_sync_entry(
        //    database_transaction& t,
        //    std::list<std::function<async_task<expected<void>>()>> & final_tasks,
        //    const const_data_buffer& object_id,
        //    uint32_t object_size) {
        //  return this->sync_process_.add_sync_entry(t, final_tasks, object_id, object_size);
        //}

        void update_wellknown_connection_enabled(bool value) {
          this->update_wellknown_connection_enabled_ = value;
        }

      private:
        friend class sync_process;
        friend class dht_session;
        friend class mock_server;
        friend class client_save_stream;

        const service_provider * sp_;
        std::list<std::shared_ptr<iudp_transport>> udp_transports_;
        dht_route route_;
        std::map<uint16_t, std::unique_ptr<chunk_generator<uint16_t>>> generators_;
        sync_process sync_process_;

        timer update_timer_;
        uint32_t update_route_table_counter_;
        bool update_wellknown_connection_enabled_;

        vds::async_task<vds::expected<void>> update_route_table();
        vds::expected<void> process_update(
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks);


        vds::async_task<vds::expected<void>> proxy_message(
            const const_data_buffer &node_id,
            message_type_t message_id,
            const const_data_buffer &message,
            const std::vector<const_data_buffer> & hops);

        vds::async_task<vds::expected<void>> send_neighbors(
          message_type_t message_id,
          expected<const_data_buffer> && message);

        static expected<const_data_buffer> replica_id(
          const std::string& key,
          uint16_t replica);

        async_task<expected<void>> send_near(
          const const_data_buffer& target_node_id,
          size_t max_count,
          const message_type_t message_id,
          expected<const_data_buffer>&& message,
          const std::function<expected<bool>(const dht_route::node & node)>& filter);

        expected<void> update_wellknown_connection(          
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks);

        static expected<const_data_buffer> read_data(
          const const_data_buffer& data_hash,
          const filename& data_path);

        static expected<void> delete_data(
          const const_data_buffer& replica_hash,
          const filename& filename);

        expected<void> restore_async(
          database_transaction& t,
          std::list<std::function<async_task<expected<void>>()>> & final_tasks,
          const std::vector<const_data_buffer>& object_ids,
          const std::shared_ptr<const_data_buffer>& result,
          const std::shared_ptr<uint8_t> & result_progress,
          bool high_priority);
      };
    }
  }
}

#endif //__VDS_DHT_NETWORK_DTH_NETWORK_CLIENT_H_
