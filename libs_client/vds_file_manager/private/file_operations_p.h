#ifndef __VDS_FILE_MANAGER_FILE_OPERATIONS_P_H_
#define __VDS_FILE_MANAGER_FILE_OPERATIONS_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "file_operations.h"
#include "user_message_transaction.h"
#include "async_buffer.h"
#include "hash.h"
#include "vds_client.h"

namespace vds {
  class user_manager;
}

namespace vds {
  namespace file_manager_private {
    class _file_operations : public std::enable_shared_from_this<_file_operations> {
    public:

      vds::expected<std::shared_ptr<stream_output_async<uint8_t>>> upload_file(
        const std::string & name,
        const std::string & mime_type,
        const const_data_buffer & file_hash,
        lambda_holder_t<
          async_task<expected<void>>,
          transactions::user_message_transaction::file_info_t> final_handler);


	    async_task<expected<file_manager::file_operations::download_result_t>> download_file(
          vds_client& client,
          std::shared_ptr<user_manager> user_mng,
          const_data_buffer channel_id,
          std::string file_name,
          const_data_buffer file_hash);

      async_task<expected<file_manager::file_operations::prepare_download_result_t>> prepare_download_file(
        const std::shared_ptr<user_manager> & user_mng,
        const const_data_buffer & channel_id,
        const std::string& file_name,
        const const_data_buffer & target_file);

      expected<std::shared_ptr<stream_input_async<uint8_t>>> download_stream(
        std::list<transactions::user_message_transaction::file_block_t> file_blocks);


      vds::async_task<vds::expected<void>> create_message(
        
        const std::shared_ptr<user_manager>& user_mng,
        const const_data_buffer& channel_id,
        const std::shared_ptr<json_value>& message,
        const std::list<transactions::user_message_transaction::file_info_t>& files);


      //	    vds::async_task<vds::expected<void>> download_block(
//			
//			database_transaction& t,
//      file_manager::download_file_task::block_info & block_id,
//			const std::shared_ptr<file_manager::download_file_task> & result);
      vds::async_task<vds::expected<void>> prepare_to_stop();

    private:
      friend class vds::file_manager::file_operations;

      struct pack_file_result {
        const_data_buffer total_hash;
        uint64_t total_size;
        std::list<transactions::user_message_transaction::file_block_t> file_blocks;
      };

//			void restore_chunk(
//					
//					database_transaction& t,
//					file_manager::download_file_task::block_info & block,
//					const std::shared_ptr<file_manager::download_file_task> & result);

      expected<std::map<vds::const_data_buffer, file_manager::file_operations::block_info_t>> prepare_download_stream(
        vds_client & client,
        const std::list<vds::transactions::user_message_transaction::file_block_t> &file_blocks_param);

      class download_stream_t : public stream_input_async<uint8_t>
      {
      public:
        download_stream_t(
          const service_provider * sp,
          std::list<transactions::user_message_transaction::file_block_t> file_blocks);

        ~download_stream_t();

        async_task<expected<size_t>> read_async(
          uint8_t * buffer,
          size_t len) override;

      private:
        std::list<transactions::user_message_transaction::file_block_t> file_blocks_;

        const_data_buffer buffer_;
        size_t readed_;
      };

      static expected<void> lookup_file(
        vds_client & client,
        const std::shared_ptr<user_manager> & user_mng,
        const const_data_buffer & channel_id,
        const std::string & file_name,
        const const_data_buffer & file_hash,
        std::shared_ptr<file_manager::file_operations::download_result_t> result,
        std::list<transactions::user_message_transaction::file_block_t> & download_tasks);

		};
  }
}

#endif //__VDS_FILE_MANAGER_FILE_OPERATIONS_P_H_
