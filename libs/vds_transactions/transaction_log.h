#ifndef __VDS_TRANSACTIONS_TRANSACTION_LOG_H_
#define __VDS_TRANSACTIONS_TRANSACTION_LOG_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "async_task.h"

namespace vds {
  class transaction_log {
  public:
    static const uint8_t user_manager_category_id = 'u';

    static void apply(
        const service_provider &sp,
        class database_transaction &t,
        const class const_data_buffer &chunk);

  private:

  };

}


#endif //__VDS_TRANSACTIONS_TRANSACTION_LOG_H_