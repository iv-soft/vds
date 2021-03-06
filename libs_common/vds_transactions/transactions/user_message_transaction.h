#ifndef __VDS_FILE_MANAGER_FILE_ADD_TRANSACTION_H_
#define __VDS_FILE_MANAGER_FILE_ADD_TRANSACTION_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <unordered_map>
#include "binary_serialize.h"
#include "channel_message_id.h"
#include "json_object.h"
#include "json_parser.h"

namespace vds {
	namespace transactions {

		class user_message_transaction {
		public:
      static const channel_message_id message_id = channel_message_id::user_message_transaction;

			struct file_block_t {
				const_data_buffer block_id;
				const_data_buffer block_key;
        uint64_t block_size;
			};

      struct file_info_t {
        std::string name;
        std::string mime_type;
        uint64_t size;
        const_data_buffer file_id;
        std::list<file_block_t> file_blocks;
      };

      std::shared_ptr<json_value> message;
      std::list<file_info_t> files;

      template <typename  visitor_type>
      auto & visit(visitor_type & v) {
        return v(
          message,
          files
        );
      }

    };
	}
    inline vds::expected<void> operator << (
            vds::binary_serializer & s,
            const vds::transactions::user_message_transaction::file_block_t & data) {
        CHECK_EXPECTED(s << data.block_id);
        CHECK_EXPECTED(s << data.block_key);
        CHECK_EXPECTED(s << data.block_size);
        return expected<void>();
    }

    inline vds::expected<void> operator >> (
            vds::binary_deserializer & s,
            vds::transactions::user_message_transaction::file_block_t & data) {
        CHECK_EXPECTED(s >> data.block_id);
        CHECK_EXPECTED(s >> data.block_key);
        CHECK_EXPECTED(s >> data.block_size);
        return expected<void>();
    }

    inline vds::expected<void> operator <<(
            vds::binary_serializer & s,
            const vds::transactions::user_message_transaction::file_info_t & data) {
        CHECK_EXPECTED(s << data.name);
        CHECK_EXPECTED(s << data.mime_type);
        CHECK_EXPECTED(s << data.size);
        CHECK_EXPECTED(s << data.file_id);
        CHECK_EXPECTED(s << data.file_blocks);
        return expected<void>();
    }

    inline vds::expected<void> operator >>(
            vds::binary_deserializer & s,
            vds::transactions::user_message_transaction::file_info_t & data) {
        CHECK_EXPECTED(s >> data.name);
        CHECK_EXPECTED(s >> data.mime_type);
        CHECK_EXPECTED(s >> data.size);
        CHECK_EXPECTED(s >> data.file_id);
        CHECK_EXPECTED(s >> data.file_blocks);
        return expected<void>();
    }

}

#endif //__VDS_FILE_MANAGER_FILE_ADD_TRANSACTION_H_
