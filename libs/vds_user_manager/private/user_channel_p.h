#ifndef __VDS_USER_MANAGER_USER_CHANNEL_P_H_
#define __VDS_USER_MANAGER_USER_CHANNEL_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include <memory>
#include "member_user.h"

namespace vds {

  class _user_channel
  {
  public:
      _user_channel();
      _user_channel(
        const vds::guid &id,
        const vds::certificate &cert);

      ~_user_channel();

  private:
    guid id_;
    certificate cert_;
  };
}

#endif // __VDS_USER_MANAGER_USER_CHANNEL_P_H_