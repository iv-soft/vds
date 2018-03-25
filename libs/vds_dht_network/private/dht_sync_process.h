#ifndef __VDS_DHT_NETWORK_DTH_SYNC_PROCESS_H_
#define __VDS_DHT_NETWORK_DTH_SYNC_PROCESS_H_

#include <messages/got_replica.h>
#include "service_provider.h"
#include "database.h"
#include "messages/offer_move_replica.h"

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

namespace vds {
  namespace dht {
    namespace network {
      class sync_process {
      public:
        void do_sync(
          const service_provider & sp,
          database_transaction & t);

        void apply_message(
            const service_provider & sp,
            database_transaction & t,
            const messages::offer_move_replica & message);

        void apply_message(
            const service_provider & sp,
            database_transaction & t,
            const messages::replica_not_found & message);

        void apply_message(
            const service_provider & sp,
            database_transaction & t,
            const messages::offer_replica & message);

        void apply_message(
            const service_provider & sp,
            database_transaction & t,
            const messages::got_replica & message);

      private:
        void sync_local_channels(
          const service_provider & sp,
          database_transaction & t);

        void sync_replicas(const service_provider &sp, database_transaction &t);
      };
    }
  }
}

#endif //__VDS_DHT_NETWORK_DTH_SYNC_PROCESS_H_
