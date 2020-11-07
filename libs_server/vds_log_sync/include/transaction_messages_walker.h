#ifndef __VDS_TRANSACTIONS_TRANSACTION_MESSAGES_WALKER_H_
#define __VDS_TRANSACTIONS_TRANSACTION_MESSAGES_WALKER_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "binary_serialize.h"
#include "payment_transaction.h"
#include "channel_message.h"
#include "create_user_transaction.h"
#include "node_manager_transactions.h"
#include "store_block_transaction.h"
#include "host_block_transaction.h"

namespace vds {
  namespace transactions {

    typedef std::tuple<
      payment_transaction,
      payment_request_transaction,
      asset_issue_transaction,
      channel_message,
      create_user_transaction,
      node_add_transaction,
      create_wallet_transaction,
      store_block_transaction,

      host_block_transaction,
      host_delete_block_transaction
    > transaction_types;

    template<size_t index>
    class transaction_messages_walker_base;

    template<>
    class transaction_messages_walker_base<0> {
    protected:
      expected<bool> visit(transaction_id message_id, binary_deserializer& s) {
        return vds::make_unexpected<std::runtime_error>("Invalid transaction " + std::to_string((uint8_t)message_id));
      }

      void set_handler(transaction_id message_id, lambda_holder_t<expected<bool>, binary_deserializer& /*s*/>&& handler) {
        assert(false);
        //return vds::make_unexpected<std::runtime_error>("Invalid transaction " + std::to_string((uint8_t)message_id));
      }
    };

    template<size_t index>
    class transaction_messages_walker_base : public transaction_messages_walker_base<index - 1> {
    protected:
      using message_type = typename std::remove_const<typename std::remove_reference<std::tuple_element_t<index - 1, transaction_types>>::type>::type;

      expected<bool> visit(transaction_id message_id, binary_deserializer& s) {
        if (message_id == message_type::message_id) {
          if (!this->handler_) {
            GET_EXPECTED(message, message_deserialize<message_type>(s));
            return expected<bool>(true);
          }
          return this->handler_(s);
        }

        return transaction_messages_walker_base<index - 1>::visit(message_id, s);
      }

      void set_handler(transaction_id message_id, lambda_holder_t<expected<bool>, binary_deserializer& /*s*/>&& handler) {
        if (message_id == message_type::message_id) {
          this->handler_ = std::move(handler);
          return;
        }

        transaction_messages_walker_base<index - 1>::set_handler(message_id, std::move(handler));
      }

    private:
      lambda_holder_t<expected<bool>, binary_deserializer& /*s*/> handler_;
    };

    class transaction_messages_walker : public transaction_messages_walker_base<std::tuple_size<message_types>::value> {
    public:
      expected<bool> process(const const_data_buffer & message_data) {
        binary_deserializer s(message_data);

        while (0 < s.size()) {
          uint8_t message_id;
          CHECK_EXPECTED(s >> message_id);

          GET_EXPECTED(result, this->visit((transaction_id)message_id, s));
          if (!result) {
            return false;
          }
        }

        return true;
      }
    };

    template <typename... handler_types>
    class transaction_messages_walker_lambdas;

    template <>
    class transaction_messages_walker_lambdas<>
      : public transaction_messages_walker
    {
    public:
      transaction_messages_walker_lambdas() {
      }
    };

    template <typename first_handler_type, typename... handler_types>
    class transaction_messages_walker_lambdas<first_handler_type, handler_types...>
        : public transaction_messages_walker_lambdas<handler_types...>
    {
      using base_class = transaction_messages_walker_lambdas<handler_types...>;
      using message_type = typename std::remove_const<typename std::remove_reference<std::tuple_element_t<0, typename functor_info<first_handler_type>::arguments_tuple>>::type>::type;
    public:
      transaction_messages_walker_lambdas(
          first_handler_type && first_handler,
          handler_types && ... handler)
      : base_class(std::forward<handler_types>(handler)...) {
        this->set_handler(message_type::message_id, [handler = std::forward<first_handler_type>(first_handler)](
          binary_deserializer& s) -> expected<bool> {
          GET_EXPECTED(message, message_deserialize<message_type>(s));
          return handler(message);
        });
      }
    };
  }
}

#endif //__VDS_TRANSACTIONS_TRANSACTION_MESSAGES_WALKER_H_

