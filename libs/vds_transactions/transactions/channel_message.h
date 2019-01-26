#ifndef __VDS_TRANSACTIONS_CHANNEL_MESSAGE_H_
#define __VDS_TRANSACTIONS_CHANNEL_MESSAGE_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "const_data_buffer.h"
#include "asymmetriccrypto.h"
#include "transaction_id.h"
#include "symmetriccrypto.h"
#include "channel_messages_walker.h"

namespace vds {
  namespace transactions {
    class channel_message {
    public:
      static const transaction_id message_id = transaction_id::channel_message;

      static expected<channel_message> create(
        const const_data_buffer &channel_id,
        const std::string &channel_read_cert_subject,
        const std::string &write_cert_subject,
        const const_data_buffer &crypted_key,
        const const_data_buffer &crypted_data,
        const asymmetric_private_key & write_key) {

        binary_serializer s;
        CHECK_EXPECTED(vds::serialize(s, (uint8_t)message_id));
        CHECK_EXPECTED(vds::serialize(s, channel_id));
        CHECK_EXPECTED(vds::serialize(s, channel_read_cert_subject));
        CHECK_EXPECTED(vds::serialize(s, write_cert_subject));
        CHECK_EXPECTED(vds::serialize(s, crypted_key));
        CHECK_EXPECTED(vds::serialize(s, crypted_data));

        GET_EXPECTED(signature, asymmetric_sign::signature(
          hash::sha256(),
          write_key,
          s.get_buffer(),
          s.size()));

        return message_create<channel_message>(
          channel_id,
          channel_read_cert_subject,
          write_cert_subject,
          crypted_key,
          crypted_data,
          signature);
      }

      template <typename  visitor_type>
      void visit(visitor_type & v) {
        v(
          this->channel_id_,
          this->channel_read_cert_subject_,
          this->write_cert_subject_,
          this->crypted_key_,
          this->crypted_data_,
          this->signature_);
      }

      const const_data_buffer & channel_id() const {
        return this->channel_id_;
      }

      const std::string & channel_read_cert_subject() const {
        return this->channel_read_cert_subject_;
      }

      const std::string & write_cert_subject() const {
        return this->write_cert_subject_;
      }

      const const_data_buffer & crypted_key() const {
        return this->crypted_key_;
      }

      const const_data_buffer & crypted_data() const {
        return this->crypted_data_;
      }

      const const_data_buffer & signature() const {
        return this->signature_;
      }


      template <typename... handler_types>
      expected<void> walk_messages(
        const service_provider * sp,
        const asymmetric_private_key & channel_read_key,
        const message_environment_t & message_environment,
        handler_types && ... handlers) const {

        GET_EXPECTED(decrypted_data, channel_read_key.decrypt(this->crypted_key_));
        GET_EXPECTED(key, symmetric_key::deserialize(
          symmetric_crypto::aes_256_cbc(),
          decrypted_data));
        GET_EXPECTED(data, symmetric_decrypt::decrypt(key, this->crypted_data_));

        channel_messages_walker_lambdas<handler_types...> walker(
          std::forward<handler_types>(handlers)...);

        CHECK_EXPECTED(walker.process(sp, data, message_environment));

        return expected<void>();
      }

    private:
      const_data_buffer channel_id_;
      std::string channel_read_cert_subject_;
      std::string write_cert_subject_;
      const_data_buffer crypted_key_;
      const_data_buffer crypted_data_;
      const_data_buffer signature_;
    };
  }
}

#endif //__VDS_TRANSACTIONS_CHANNEL_MESSAGE_H_
