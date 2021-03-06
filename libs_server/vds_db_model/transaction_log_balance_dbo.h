#ifndef __VDS_DB_MODEL_TRANSACTION_LOG_BALANCE_DBO_H_
#define __VDS_DB_MODEL_TRANSACTION_LOG_BALANCE_DBO_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "database_orm.h"
#include "const_data_buffer.h"

namespace vds {
  namespace orm {
    class transaction_log_balance_dbo : public database_table {
    public:
      transaction_log_balance_dbo()
          : database_table("transaction_log_balance"),
            id(this, "id"),
            owner(this, "owner"),
            source(this, "source"),
            balance(this, "balance") {
      }

      database_column<const_data_buffer, std::string> id;
      database_column<std::string> owner;
      database_column<const_data_buffer> source;
      database_column<int64_t> balance;
    };
  }
}

#endif //__VDS_DB_MODEL_TRANSACTION_LOG_BALANCE_DBO_H_
