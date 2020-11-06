#ifndef __VDS_TRANSACTIONS_CHANNEL_MESSAGES_WALKER_H_
#define __VDS_TRANSACTIONS_CHANNEL_MESSAGES_WALKER_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "binary_serialize.h"
#include "payment_transaction.h"
#include "channel_message_id.h"
#include "channel_add_reader_transaction.h"
#include "channel_add_writer_transaction.h"
#include "channel_create_transaction.h"
#include "user_message_transaction.h"
#include "func_utils.h"
#include "control_message_transaction.h"
#include "logger.h"

namespace vds {
  namespace transactions {
    struct message_environment_t {
      std::chrono::system_clock::time_point time_point_;
      std::string user_name_;
    };

    typedef std::tuple<
      channel_add_reader_transaction,
      channel_add_writer_transaction,
      channel_create_transaction,
      user_message_transaction,
      control_message_transaction
    > message_types;

    template<size_t index>
    class channel_messages_walker_base;
    
    template<>
    class channel_messages_walker_base<0> {
    protected:
      expected<bool> visit(channel_message_id message_id, binary_deserializer & s, const message_environment_t& message_environment) {
        return vds::make_unexpected<std::runtime_error>("Invalid channel message " + std::to_string((uint8_t)message_id));
      }

      expected<void> set_handler(channel_message_id message_id, lambda_holder_t<expected<bool>, binary_deserializer& /*s*/, const message_environment_t& /*message_environment*/> && handler) {
        return vds::make_unexpected<std::runtime_error>("Invalid channel message " + std::to_string((uint8_t)message_id));
      }
    };

    template<size_t index>
    class channel_messages_walker_base : public channel_messages_walker_base<index - 1> {
    protected:
      using message_type = std::tuple_element_t<index - 1, message_types>;

      expected<bool> visit(channel_message_id message_id, binary_deserializer& s, const message_environment_t& message_environment) {
        if (message_id == message_type::message_id) {
          if (!this->handler_) {
            GET_EXPECTED(message, message_deserialize<message_type>(s));
            return expected<bool>(true);
          }
          return this->handler_(s, message_environment);
        }

        return channel_messages_walker_base<index - 1>::visit(message_id, s, message_environment);
      }

      expected<void> set_handler(channel_message_id message_id, lambda_holder_t<expected<bool>, binary_deserializer& /*s*/, const message_environment_t& /*message_environment*/>&& handler) {
        if (message_id == message_type::message_id) {
          this->handler_ = std::move(handler);
          return expected<void>();
        }

        return channel_messages_walker_base<index - 1>::set_handler(message_id, s, std::move(handler));
      }

    private:
      lambda_holder_t<expected<bool>, binary_deserializer& /*s*/, const message_environment_t&> handler_;
    };

    class channel_messages_walker : public channel_messages_walker_base<std::tuple_size<message_types>::value>{
    public:
      expected<bool> process(
              const service_provider * sp,
              const const_data_buffer & message_data,
              const message_environment_t & message_environment) {
        binary_deserializer s(message_data);

        while(0 < s.size()) {
          uint8_t message_id;
          CHECK_EXPECTED(s >> message_id);

          GET_EXPECTED(result, this->visit((channel_message_id)message_id, s, message_environment));
          if (!result) {
            return false;
          }
        }

        return true;
      }
    };

    template <typename... handler_types>
    class channel_messages_walker_lambdas;

    template <>
    class channel_messages_walker_lambdas<>
      : public channel_messages_walker
    {
    public:
      channel_messages_walker_lambdas() {
      }
    };

    template <typename first_handler_type, typename... handler_types>
    class channel_messages_walker_lambdas<first_handler_type, handler_types...>
        : public channel_messages_walker_lambdas<handler_types...>
    {
      using base_class = channel_messages_walker_lambdas<handler_types...>;
    public:
      channel_messages_walker_lambdas(
        first_handler_type&& first_handler,
        handler_types && ... handler)
        : base_class(std::forward<handler_types>(handler)...) {
        this->set_handler(std::tuple_element_t<0, typename functor_info<first_handler_type>::arguments_tuple>::message_id, [handler = std::move(first_handler)](
          binary_deserializer& s,
          const message_environment_t& message_environment) {
            GET_EXPECTED(message, message_deserialize<typename std::tuple_element_t<0, typename functor_info<first_handler_type>::arguments_tuple>>(s));
            return handler(message, message_environment);
          });
      }
    };
  }
}

#endif //__VDS_TRANSACTIONS_CHANNEL_MESSAGES_WALKER_H_

