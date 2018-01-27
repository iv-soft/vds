#ifndef __VDS_FILE_MANAGER_FILE_OPERATIONS_H_
#define __VDS_FILE_MANAGER_FILE_OPERATIONS_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <string>
#include <guid.h>
#include <transactions/file_add_transaction.h>
#include "const_data_buffer.h"
#include "async_task.h"
#include "filename.h"
#include "download_file_task.h"

namespace vds {
  namespace file_manager_private {
    class _file_operations;
  };

  namespace file_manager {
    class file_operations {
    public:
      static const size_t BLOCK_SIZE = 16 * 1024 * 1024;

      file_operations();

      vds::async_task<> upload_file(
		  const service_provider &sp,
		  const vds::guid &channel_id,
          const std::string &name,
		  const std::string &mimetype,
          const vds::filename &file_path);


	  vds::async_task<> download_file(
		  const service_provider &sp,
			const std::shared_ptr<download_file_task> & task);

    protected:
      std::shared_ptr<file_manager_private::_file_operations> impl_;
    };
  }
}

#endif //__VDS_FILE_MANAGER_FILE_OPERATIONS_H_
