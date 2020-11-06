#ifndef __VDS_DHT_NETWORK_SYNC_MESSAGES_H__
#define __VDS_DHT_NETWORK_SYNC_MESSAGES_H__

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

namespace vds {
  namespace dht {
    namespace messages {

      /**
     * \brief Replica data request from outside object's consensus
     */
      class sync_replica_request {
      public:
        static const network::message_type_t message_id = network::message_type_t::sync_replica_request;

        const_data_buffer object_id;

        template <typename visitor_type>
        auto & visit(visitor_type & v) {
          return v(
            this->object_id
          );
        }
      };

  //    /**
  //     * \brief Base class to message from leader
  //     */
  //    class sync_base_message_request {
  //    public:
  //      const_data_buffer object_id;
  //      uint64_t generation;
  //      uint64_t current_term;
  //      uint64_t commit_index;
  //      uint64_t last_applied;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->generation,
  //          this->current_term,
  //          this->commit_index,
  //          this->last_applied
  //        );
  //      }
  //    };

  //    /**
  //     * \brief Base class to responce message to leader
  //     */
  //    class sync_base_message_response {
  //    public:
  //      const_data_buffer object_id;
  //      uint64_t generation;
  //      uint64_t current_term;
  //      uint64_t commit_index;
  //      uint64_t last_applied;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->generation,
  //          this->current_term,
  //          this->commit_index,
  //          this->last_applied
  //        );
  //      }
  //    };

  //    /**
  //     * \brief Request new operation from leader
  //     */
  //    class sync_replica_operations_request : public sync_base_message_request {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_replica_operations_request;

  //      orm::sync_message_dbo::message_type_t message_type;
  //      const_data_buffer member_node;
  //      uint16_t replica;
  //      const_data_buffer message_source_node;
  //      uint64_t message_source_index;
  //      
  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return sync_base_message_request::visit<visitor_type>(v)
  //        (
  //          this->message_type,
  //          this->member_node,
  //          this->replica,
  //          this->message_source_node,
  //          this->message_source_index
  //        );
  //      }
  //    };

  //    /**
  //     * \brief Response about new operation to leader
  //     */
  //    class sync_replica_operations_response : public sync_base_message_response {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_replica_operations_response;

  //    };

  //    /**
  //     * \brief new operation from leader
  //     */
  //    class sync_add_message_request {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_add_message_request;

  //      const_data_buffer object_id;
  //      const_data_buffer leader_node;
  //      const_data_buffer source_node;
  //      uint64_t local_index;
  //      orm::sync_message_dbo::message_type_t message_type;
  //      const_data_buffer member_node;
  //      uint16_t replica;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->leader_node,
  //          this->source_node,
  //          this->local_index,
  //          this->message_type,
  //          this->member_node,
  //          this->replica
  //        );
  //      }

  //    };

  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //    class sync_new_election_request {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_new_election_request;

  //      const_data_buffer object_id;
  //      uint64_t generation;
  //      uint64_t current_term;
  //      const_data_buffer source_node;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->generation,
  //          this->current_term,
  //          this->source_node
  //        );
  //      }
  //    };

  //    class sync_new_election_response {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_new_election_response;

  //      const_data_buffer object_id;
  //      uint64_t generation;
  //      uint64_t current_term;
  //      const_data_buffer source_node;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->generation,
  //          this->current_term,
  //          this->source_node
  //        );
  //      }
  //    };
  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //    class sync_replica_query_operations_request : public sync_base_message_response {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_replica_query_operations_request;
  //    };

  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //    class sync_looking_storage_request : public sync_base_message_request {
  //      using base_class = sync_base_message_request;
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_looking_storage_request;

  //      uint32_t object_size;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return base_class::visit(v)(
  //          this->object_size
  //          );
  //      }
  //    };

  //    class sync_looking_storage_response {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_looking_storage_response;

  //      const_data_buffer object_id;
  //      std::set<uint16_t> replicas;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->replicas
  //          );
  //      }
  //    };

  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //    class sync_snapshot_request {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_snapshot_request;

  //      const_data_buffer object_id;
  //      const_data_buffer source_node;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return v(
  //          this->object_id,
  //          this->source_node
  //        );
  //      }
  //    };

  //    class sync_snapshot_response : public sync_base_message_request {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_snapshot_response;

  //      struct member_state {
  //        const_data_buffer voted_for;
  //        const_data_buffer public_key;
  //        const_data_buffer sign;
  //      };

  //      uint32_t object_size;
  //      const_data_buffer target_node;
  //      const_data_buffer leader_node;
  //      std::map<const_data_buffer, std::set<uint16_t>> replica_map;
  //      std::map<const_data_buffer, member_state> members;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return sync_base_message_request::visit(v)(
  //          this->object_size,
  //          this->target_node,
  //          this->leader_node,
  //          this->replica_map,
  //          this->members
  //        );
  //      }
  //    };
  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //    class sync_leader_broadcast_request : public sync_base_message_request {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_leader_broadcast_request;

  //    };

  //    class sync_leader_broadcast_response : public sync_base_message_response {
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_leader_broadcast_response;

  //    };

  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //    class sync_offer_send_replica_operation_request : public sync_base_message_request {
  //      using base_class = sync_base_message_request;
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_offer_send_replica_operation_request;

  //      uint16_t replica;
  //      const_data_buffer target_node;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return base_class::visit(v)(
  //          this->replica,
  //          this->target_node
  //        );
  //      }
  //    };

  //    class sync_offer_remove_replica_operation_request : public sync_base_message_request {
  //      using base_class = sync_base_message_request;
  //    public:
  //      static const network::message_type_t message_id = network::message_type_t::sync_offer_remove_replica_operation_request;

  //      const_data_buffer member_node;
  //      uint16_t replica;

  //      template <typename visitor_type>
  //      auto & visit(visitor_type & v) {
  //        return base_class::visit(v)(
  //          this->member_node,
  //          this->replica
  //          );
  //      }
  //    };
  //    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      class sync_replica_data {
      public:
        static const network::message_type_t message_id = network::message_type_t::sync_replica_data;

        const_data_buffer object_id;
        const_data_buffer data;
        const_data_buffer owner;

        template <typename visitor_type>
        auto & visit(visitor_type & v) {
          return v(
            object_id,
            data,
            owner);
        }
      };
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
  }

  //inline expected<void> operator <<(
  //  binary_serializer& s,
  //  const orm::sync_message_dbo::message_type_t & f) {
  //  return (s << (uint8_t)f);
  //}

  //inline expected<void> operator >>(
  //  binary_deserializer& s,
  //  orm::sync_message_dbo::message_type_t & f) {
  //  uint8_t v;
  //  CHECK_EXPECTED(s >> v);
  //  f = static_cast<orm::sync_message_dbo::message_type_t>(v);
  //  return expected<void>();
  //}

  //inline expected<void> operator <<(
  //  binary_serializer& s,
  //  const dht::messages::sync_snapshot_response::member_state & f) {
  //  CHECK_EXPECTED(s << f.voted_for);
  //  CHECK_EXPECTED(s << f.public_key);
  //  CHECK_EXPECTED(s << f.sign);
  //  return expected<void>();
  //}

  //inline expected<void> operator >>(
  //  binary_deserializer& s,
  //  dht::messages::sync_snapshot_response::member_state & f) {
  //  CHECK_EXPECTED(s >> f.voted_for);
  //  CHECK_EXPECTED(s >> f.public_key);
  //  CHECK_EXPECTED(s >> f.sign);
  //  return expected<void>();
  //}
}

#endif//__VDS_DHT_NETWORK_SYNC_MESSAGES_H__
