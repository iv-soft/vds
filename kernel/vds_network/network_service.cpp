/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "network_service.h"
#include "private/network_service_p.h"
#include "tcp_network_socket.h"
#include "udp_socket.h"
#include "service_provider.h"
#include "logger.h"
#include "private/socket_task_p.h"
#include "private/mt_service_p.h"

vds::network_service::network_service()
: impl_(new _network_service())
{
}

vds::network_service::~network_service()
{
}

vds::expected<void> vds::network_service::register_services(service_registrator & registator)
{
    registator.add_service<network_service>(this);
    return expected<void>();
}

vds::expected<void> vds::network_service::start(const service_provider * sp)
{
  return this->impl_->start(sp);
}

vds::expected<void> vds::network_service::stop()
{
  return this->impl_->stop();
}

vds::async_task<vds::expected<void>> vds::network_service::prepare_to_stop()
{
  return this->impl_->prepare_to_stop();
}


std::string vds::network_service::to_string(const sockaddr & from, size_t from_len)
{
  char hbuf[NI_MAXHOST], sbuf[NI_MAXSERV];
  
  getnameinfo(&from, (socklen_t)from_len,
    hbuf, sizeof hbuf,
    sbuf, sizeof sbuf,
    NI_NUMERICHOST | NI_NUMERICSERV);
  
  return std::string(hbuf) + ":" + std::string(sbuf);
}

std::string vds::network_service::to_string(const sockaddr& from, size_t from_len, uint16_t port)
{
  char hbuf[NI_MAXHOST], sbuf[NI_MAXSERV];

  getnameinfo(&from, (socklen_t)from_len,
    hbuf, sizeof hbuf,
    sbuf, sizeof sbuf,
    NI_NUMERICHOST | NI_NUMERICSERV);

  return std::string(hbuf) + ":" + std::to_string(port);
}

std::string vds::network_service::to_string(const sockaddr_in & from)
{
  return get_ip_address_string(from) + ":" + std::to_string(ntohs(from.sin_port));
}

std::string vds::network_service::get_ip_address_string(const sockaddr_in & from)
{
  char buffer[20];
  int len = sizeof(buffer);

  inet_ntop(from.sin_family, &(from.sin_addr), buffer, len);

  return buffer;
}

vds::expected<std::list<vds::network_address>> vds::network_service::all_network_addresses()
{
#ifdef _WIN32
  ULONG outBufLen = 16 * 1024;
  std::unique_ptr<IP_ADAPTER_ADDRESSES> pAddresses;
  for (;;) {
    pAddresses.reset(reinterpret_cast<IP_ADAPTER_ADDRESSES*>(malloc(outBufLen)));
    if (!pAddresses) {
      return make_unexpected<std::bad_alloc>();//Memory allocation failed for IP_ADAPTER_ADDRESSES struct
    }
    const auto dwRetVal = GetAdaptersAddresses(AF_UNSPEC, 0, NULL, pAddresses.get(), &outBufLen);
    if (ERROR_BUFFER_OVERFLOW == dwRetVal) {
      continue;
    }
    if (NO_ERROR == dwRetVal) {
      break;
    }
    
    return make_unexpected<std::system_error>(dwRetVal, std::system_category(), "GetAdaptersAddresses");
  }

  std::list<network_address> result;
  for (auto current = pAddresses.get(); current != nullptr; current = current->Next) {
    if (IfOperStatusUp == current->OperStatus) {
      for (auto address = current->FirstUnicastAddress; address != NULL; address = address->Next) {
        result.push_back(
          network_address(address->Address.lpSockaddr, address->Address.iSockaddrLength)
        );
      }
    }
  }

  return result;
#else//_WIN32
  struct ifaddrs* ifaddr, * ifa;
  if (0 > getifaddrs(&ifaddr)) {
    auto error = errno;
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "getifaddrs");
  }

  std::list<network_address> result;
  for (auto ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == NULL) {
      continue;
    }

    if ((IFF_UP | IFF_BROADCAST) == (ifa->ifa_flags & (IFF_UP | IFF_BROADCAST)) && (ifa->ifa_addr->sa_family == AF_INET || ifa->ifa_addr->sa_family == AF_INET6)) {
      result.push_back(
        network_address(ifa->ifa_addr, (ifa->ifa_addr->sa_family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6))
      );
    }
  }

  freeifaddrs(ifaddr);
  return result;
#endif//_WIN32
}
/////////////////////////////////////////////////////////////////////////////
#define NETWORK_EXIT 0xA1F8

vds::_network_service::_network_service()
#ifdef _WIN32
  : handle_(NULL)
#endif
{
}


vds::_network_service::~_network_service()
{
}

vds::expected<void> vds::_network_service::start(const service_provider * sp)
{
  this->sp_ = sp;

#ifdef _WIN32
    //Initialize Winsock
    WSADATA wsaData;
    if (NO_ERROR != WSAStartup(MAKEWORD(2, 2), &wsaData)) {
        auto error = WSAGetLastError();
        return vds::make_unexpected<std::system_error>(error, std::system_category(), "Initiates Winsock");
    }

    this->handle_ = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 0);

    if (NULL == this->handle_) {
        auto error = WSAGetLastError();
        return vds::make_unexpected<std::system_error>(error, std::system_category(), "Create I/O completion port");
    }

    //Create worker threads
    for (unsigned int i = 0; i < 2 * std::thread::hardware_concurrency(); ++i) {
        this->work_threads_.push_back(new std::thread([this] { this->thread_loop(); }));
    }

#else
    this->epoll_set_ = epoll_create(100);
    if(0 > this->epoll_set_){
      return vds::make_unexpected<std::runtime_error>("Out of memory for epoll_create");
    }
  this->epoll_thread_ = std::thread(
    [this, sp] {
      _mt_service::set_instance(sp);
      for(;;){
        std::unique_lock<std::mutex> lock(this->tasks_mutex_);
        if(this->tasks_.empty()){
          if(sp->get_shutdown_event().is_shuting_down()){
            break;
          }
          
          this->tasks_cond_.wait(lock);
          continue;
        }
        lock.unlock();
        
        struct epoll_event events[64];
        
        auto result = epoll_wait(this->epoll_set_, events, sizeof(events) / sizeof(events[0]), 1000);
        if(0 > result){
          auto error = errno;
          if(EINTR == error){
            continue;
          }
          
          //return vds::make_unexpected<std::system_error>(error, std::system_category(), "epoll_wait");
          return;
        }
        else if(0 < result){
          for(int i = 0; i < result; ++i){
            lock.lock();
            auto p = this->tasks_.find(events[i].data.fd);
            if(this->tasks_.end() != p){
              auto handler = p->second;
              lock.unlock();

                (void)handler->process(events[i].events);
            }
            else {
              lock.unlock();
            }
          }
        }          
      }
  });
#endif
  return expected<void>();
}

vds::expected<void> vds::_network_service::stop()
{
  this->sp_->get<logger>()->trace("network", "Stopping network service");

#ifndef _WIN32
  for(;;) {
      std::list<std::shared_ptr<socket_base>> tasks;
      std::unique_lock<std::mutex> lock(this->tasks_mutex_);
      for (auto &task : this->tasks_) {
          tasks.push_back(task.second);
      }
      lock.unlock();
      if(tasks.empty()){
          break;
      }
      for (auto &task : tasks) {
          task->stop();
      }
  }

    this->tasks_cond_.notify_one();
  if (this->epoll_thread_.joinable()) {
    this->epoll_thread_.join();
  }
#else
  for (size_t i = 0; i < this->work_threads_.size(); ++i) {
    PostQueuedCompletionStatus(this->handle_, 0, NETWORK_EXIT, NULL);
  }
  for (auto p : this->work_threads_) {
    p->join();
    delete p;
  }
#endif

#ifdef _WIN32

  if (NULL != this->handle_) {
    CloseHandle(this->handle_);
  }

  WSACleanup();
#endif
  return expected<void>();
}

vds::async_task<vds::expected<void>> vds::_network_service::prepare_to_stop()
{

#ifndef _WIN32
    for (;;) {
        std::list<std::shared_ptr<socket_base>> tasks;
        std::unique_lock<std::mutex> lock(this->tasks_mutex_);
        for (auto& task : this->tasks_) {
            tasks.push_back(task.second);
        }
        lock.unlock();
        if (tasks.empty()) {
            break;
        }
        for (auto& task : tasks) {
            task->stop();
        }
    }

    this->tasks_cond_.notify_one();
    if (this->epoll_thread_.joinable()) {
        this->epoll_thread_.join();
    }
#else
    for (size_t i = 0; i < this->work_threads_.size(); ++i) {
        PostQueuedCompletionStatus(this->handle_, 0, NETWORK_EXIT, NULL);
    }
    for (auto p : this->work_threads_) {
        p->join();
        delete p;
    }
    this->work_threads_.clear();
#endif

#ifdef _WIN32
    if (NULL != this->handle_) {
        CloseHandle(this->handle_);
        this->handle_ = NULL;
    }

    WSACleanup();
#endif
    co_return expected<void>();
}

#ifdef _WIN32

vds::expected<void> vds::_network_service::associate(SOCKET_HANDLE s)
{
    if (this->sp_->get_shutdown_event().is_shuting_down()) {
        return vds::make_unexpected<vds_exceptions::shooting_down_exception>();
    }


  if (NULL == CreateIoCompletionPort((HANDLE)s, this->handle_, NULL, 0)) {
    auto error = GetLastError();
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "Associate with input/output completion port");
  }
  return expected<void>();
}

void vds::_network_service::thread_loop()
{
	_mt_service::set_instance(this->sp_);
  while (!this->sp_->get_shutdown_event().is_shuting_down()) {
    DWORD dwBytesTransfered = 0;
    ULONG_PTR lpContext;
    OVERLAPPED * pOverlapped = NULL;

    if (!GetQueuedCompletionStatus(
      this->handle_,
      &dwBytesTransfered,
      &lpContext,
      &pOverlapped,
      INFINITE)) {
      auto errorCode = GetLastError();
      if (errorCode == WAIT_TIMEOUT) {
        continue;
      }

      if (pOverlapped != NULL) {
        this->sp_->get<logger>()->error("network", "GetQueuedCompletionStatus %d error %s", errorCode, std::system_error(errorCode, std::system_category(), "GetQueuedCompletionStatus").what());
        _socket_task::from_overlapped(pOverlapped)->error(errorCode);
        continue;
      }
      else {
        this->sp_->get<logger>()->error("network", "GetQueuedCompletionStatus %d error %s", errorCode, std::system_error(errorCode, std::system_category(), "GetQueuedCompletionStatus").what());
        return;
      }
    }

    if (lpContext == NETWORK_EXIT) {
      return;
    }

    _socket_task::from_overlapped(pOverlapped)->process(dwBytesTransfered);
  }
}
#else

vds::expected<void> vds::_network_service::associate(
  SOCKET_HANDLE s,
  const std::shared_ptr<socket_base> & handler,
  uint32_t event_mask)
{
  if (this->sp_->get_shutdown_event().is_shuting_down()) {
    return vds::make_unexpected<vds_exceptions::shooting_down_exception>();
  }

  struct epoll_event event_data;
  memset(&event_data, 0, sizeof(event_data));
  event_data.events = event_mask;
  event_data.data.fd = s;
  
  int result = epoll_ctl(this->epoll_set_, EPOLL_CTL_ADD, s, &event_data);
  if(0 > result) {
    auto error = errno;
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "epoll_ctl(EPOLL_CTL_ADD)");
  }
  
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  if(this->tasks_.empty()){
    this->tasks_cond_.notify_one();
  }
  
  this->tasks_[s] = handler;
  return expected<void>();
}

vds::expected<void> vds::_network_service::set_events(
  SOCKET_HANDLE s,
  uint32_t event_mask)
{
  struct epoll_event event_data;
  memset(&event_data, 0, sizeof(event_data));
  event_data.events = event_mask;
  event_data.data.fd = s;
  
  int result = epoll_ctl(this->epoll_set_, EPOLL_CTL_MOD, s, &event_data);
  if(0 > result) {
    const auto error = errno;
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "epoll_ctl(EPOLL_CTL_MOD)");
  }

  return expected<void>();
}

vds::expected<void> vds::_network_service::remove_association(
  SOCKET_HANDLE s)
{
  struct epoll_event event_data;
  memset(&event_data, 0, sizeof(event_data));
  event_data.events = 0;
  
  int result = epoll_ctl(this->epoll_set_, EPOLL_CTL_DEL, s, &event_data);
  if(0 > result) {
    const auto error = errno;
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "epoll_ctl(EPOLL_CTL_DEL)");
  }
  
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  this->tasks_.erase(s);
  return expected<void>();
}

#endif//_WIN32

