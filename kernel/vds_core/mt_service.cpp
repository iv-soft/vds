/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "mt_service.h"
#include "shutdown_event.h"
#include "barrier.h"

#include "private/mt_service_p.h"

#include <thread>
#include "logger.h"

#ifndef _WIN32
#include <sys/syscall.h>
#include <sys/types.h>
#endif//_WIN32

thread_local vds::imt_service * current_instance;

vds::mt_service::mt_service()
{
}

vds::mt_service::~mt_service()
{
}

vds::expected<void> vds::mt_service::register_services(vds::service_registrator& registrator)
{
  registrator.add_service<imt_service>(this);
  return expected<void>();
}

vds::expected<void> vds::mt_service::start(const service_provider * sp)
{
  this->impl_.reset(new _mt_service(sp));
  this->impl_->start();

  return expected<void>();
}

vds::expected<void> vds::mt_service::stop()
{
	if (this->impl_) {
		this->impl_->stop();
	}

  return expected<void>();
}

vds::async_task<vds::expected<void>> vds::mt_service::prepare_to_stop() {
  return this->impl_->prepare_to_stop();
}

vds::imt_service * vds::imt_service::get_current()
{
	return current_instance;
}

void vds::imt_service::async(const service_provider * sp, const std::function<void(void)>& handler)
{
	sp->get<imt_service>()->do_async(handler);
}

void vds::imt_service::async(const service_provider * sp, std::function<void(void)>&& handler)
{
	sp->get<imt_service>()->do_async(std::move(handler));
}

void vds::imt_service::do_async(const std::function<void(void)>& handler)
{
  ((mt_service *)this)->impl_->do_async(handler);
}

void vds::imt_service::do_async( std::function<void(void)> && handler)
{
  ((mt_service *)this)->impl_->do_async(std::move(handler));
}

vds::_mt_service::_mt_service(const service_provider * sp)
: sp_(sp), is_shuting_down_(false), free_threads_(0)
{
}

void vds::_mt_service::start()
{
  //unsigned int count = std::thread::hardware_concurrency();
  //if(count < 1){
  //  count = 1;
  //}
  //else if(count > 1024 * 1024) {
  //  count = 1024 * 1024;
  //}
  //
  //for(unsigned int i = 0; i < count; ++i){
  //  this->work_threads_.push_back(std::thread(std::bind(&_mt_service::work_thread, this)));
  //}
}

void vds::_mt_service::stop()
{
  for(auto & t : this->work_threads_){
    this->cond_.notify_all();
    t.join();
  }
}

vds::async_task<vds::expected<void>> vds::_mt_service::prepare_to_stop() {
  this->is_shuting_down_ = true;
  for (auto & t : this->work_threads_) {
	  this->cond_.notify_all();
  }
  co_return expected<void>();
}

void vds::_mt_service::do_async( const std::function<void(void)> & handler)
{
  std::unique_lock<std::mutex> lock(this->mutex_);
#if defined(DEBUG)
  this->queue_.push([sp = this->sp_, handler, thread_id =
#ifndef _WIN32
    syscall(SYS_gettid)
#else
    GetCurrentThreadId()
#endif
  ]() {
    sp->get<logger>()->trace("Async", "Anync from %d", thread_id);
    handler();
  });
#else//defined(DEBUG)
  this->queue_.push(handler);
#endif//defined(DEBUG)
  this->cond_.notify_all();
}

void vds::_mt_service::do_async( std::function<void(void)> && handler)
{
  std::unique_lock<std::mutex> lock(this->mutex_);
  this->queue_.push(std::move(handler));
  if(0 == this->free_threads_) {
    this->work_threads_.emplace_back(std::bind(&_mt_service::work_thread, this));
  }
  else {
    this->cond_.notify_one();
  }
}

void vds::_mt_service::set_instance(const service_provider * sp)
{
	current_instance = sp->get<imt_service>();
}

void vds::_mt_service::work_thread()
{
  set_instance(this->sp_);
  while(!this->is_shuting_down_){
    std::unique_lock<std::mutex> lock(this->mutex_);
    if (this->queue_.empty()) {
      if(8 < ++this->free_threads_) {
        --this->free_threads_;
        break;
      }
      do {
        this->cond_.wait(lock);
      } while (this->queue_.empty() && !this->is_shuting_down_);
      --this->free_threads_;

      if (this->is_shuting_down_) {
        break;
      }
    }
    
    auto handler = this->queue_.front();
    this->queue_.pop();

    lock.unlock();
    
    handler();
  }
  current_instance = nullptr;
}

