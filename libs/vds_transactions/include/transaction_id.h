#ifndef __VDS_TRANSACTIONS__TRANSACTION_ID_H_
#define __VDS_TRANSACTIONS__TRANSACTION_ID_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
namespace vds {
  namespace transactions {
    enum class transaction_id : uint8_t {
      payment_transaction = 'p',
      asset_issue_transaction = 'i',
      channel_message = 'c',
      create_user_transaction = 'u',
      node_add_transaction = 'm'
    };
  }
}

#endif //__VDS_TRANSACTIONS__TRANSACTION_ID_H_
