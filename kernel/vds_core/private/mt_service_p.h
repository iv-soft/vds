#ifndef __VDS_CORE_MT_SERVICE_P_H_
#define __VDS_CORE_MT_SERVICE_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include <list>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>

#include "service_provider.h"

namespace vds {
  class mt_service;
  
  class _mt_service
  {
  public:
    _mt_service(const service_provider * sp);

    void start();
    void stop();
    async_task<expected<void>> prepare_to_stop();

    void do_async(lambda_holder_t<void> handler);

	static void set_instance(const service_provider * sp);

  private:
    const service_provider * sp_;
    std::list<std::thread> work_threads_;
    unsigned int free_threads_;
    bool is_shuting_down_;
    
    std::mutex mutex_;
    std::condition_variable cond_;
    std::queue<lambda_holder_t<void>> queue_;
    
    void work_thread();
  };
}

#endif // __VDS_CORE_MT_SERVICE_P_H_
