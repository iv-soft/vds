#ifndef __VDS_CHUNK_MANAGER_CHUNK_MANAGER_H_
#define __VDS_CHUNK_MANAGER_CHUNK_MANAGER_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include <unordered_map>
#include <memory>
#include <string>
#include "const_data_buffer.h"

namespace vds {
  class chunk_manager {
	public:
		static const uint32_t BLOCK_SIZE = 1024 * 1024;

		struct chunk_info {
			const_data_buffer id;
			const_data_buffer key;
			const_data_buffer data;
		};

		static chunk_info pack_block(
				const const_data_buffer &data);

		static chunk_info save_block(
				class database_transaction & t,
				const const_data_buffer &data,
        size_t & padding,
				std::unordered_map<uint16_t, const_data_buffer> & replica_hashes);

		static const_data_buffer unpack_block(
				const const_data_buffer & block_id,
				const const_data_buffer & block_key,
				const void * block_data,
        size_t block_size);

    static const_data_buffer unpack_block(
        const const_data_buffer & block_id,
        const const_data_buffer & block_key,
        const const_data_buffer & block_data){
      return unpack_block(block_id, block_key, block_data.data(), block_data.size());
    }

	private:
		std::shared_ptr<class _chunk_manager> impl_;
	};
}

#endif //__VDS_CHUNK_MANAGER_CHUNK_MANAGER_H_
