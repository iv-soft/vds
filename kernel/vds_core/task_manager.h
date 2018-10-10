#ifndef __VDS_CORE_TASK_MANAGER_H_
#define __VDS_CORE_TASK_MANAGER_H_

#include <chrono>
#include <list>
#include <condition_variable>
#include <thread>

#include "service_provider.h"
#include "state_machine.h"

namespace vds {
  
  class timer
  {
  public:
    timer(const char * name);
    
    void start(
      const service_provider * sp,
      const std::chrono::steady_clock::duration & period,
      const std::function<bool(void)> & callback);
    
    void stop();
    
  private:
    const service_provider * sp_;
    std::string name_;

    friend class task_manager;
    std::chrono::steady_clock::duration period_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    std::function<bool(void)> handler_;

	  enum state_t {
		  bof,
		  scheduled,
		  in_handler,
		  eof
	  };
    std::shared_ptr<state_machine<state_t>> current_state_;
		bool is_shuting_down_;
    void execute();
    void schedule();
  };

  class task_manager : public iservice_factory
  {
  public:
    task_manager();
    ~task_manager();

    void register_services(service_registrator &) override;
    void start(const service_provider * sp) override;
    void stop() override;
    std::future<void> prepare_to_stop() override;

    void disable_timers() {
      this->is_disabled_ = true;
    }

  private:
    friend class timer;
    const service_provider * sp_;
    std::list<timer *> scheduled_;
    std::condition_variable scheduled_changed_;
    std::mutex scheduled_mutex_;
    std::thread work_thread_;
    bool is_shuting_down_;
    bool is_disabled_;
    void work_thread();
  };
}

#endif // __VDS_CORE_TASK_MANAGER_H_
