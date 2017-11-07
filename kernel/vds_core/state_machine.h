#ifndef __VDS_CORE_STATE_MACHINE_H_
#define __VDS_CORE_STATE_MACHINE_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <mutex>
#include <condition_variable>

#include "types.h"

namespace vds {
  
  template <typename state_enum_type>
  class state_machine
  {
  public:
    state_machine(state_enum_type start_state, state_enum_type failed_state)
    : state_(start_state), failed_state_(failed_state)
    {
    }
    
    bool change_state(state_enum_type expected_state, state_enum_type new_state, error_logic err_logic)
    {
      std::unique_lock<std::mutex> lock(this->state_mutex_);
      for(;;){
        if(expected_state == this->state_){
          this->state_ = new_state;
          this->state_cond_.notify_one();
          return true;
        }
        else if(this->failed_state_ == this->state_) {
          if(error_logic::throw_exception == err_logic){
            throw *this->error_;
          }
          
          return false;
        }
        
        this->state_cond_.wait(lock);
      }
    }
    
    bool wait(state_enum_type expected_state, error_logic err_logic)
    {
      std::unique_lock<std::mutex> lock(this->state_mutex_);
      for(;;){
        if(expected_state == this->state_){
          return true;
        }
        else if(this->failed_state_ == this->state_) {
          if(error_logic::throw_exception == err_logic){
            throw *this->error_;
          }
          
          return false;
        }
        
        this->state_cond_.wait(lock);
      }
    }
    
    bool fail(const std::shared_ptr<std::exception> & ex)
    {
      std::unique_lock<std::mutex> lock(this->state_mutex_);
      if(this->failed_state_ != this->state_){
        this->error_ = ex;
        this->state_ = this->failed_state_;
        this->state_cond_.notify_one();
        return true;
      }
      else {
        return false;
      }
    }
    
    const std::shared_ptr<std::exception> & get_error() const
    {
      return this->error_;
    }
    
  private:
    state_enum_type state_;
    const state_enum_type failed_state_;
    std::mutex state_mutex_;
    std::condition_variable state_cond_;
    std::shared_ptr<std::exception> error_;
  };
  
};

#endif//__VDS_CORE_STATE_MACHINE_H_