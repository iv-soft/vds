#ifndef __VDS_NETWORK_SOCKET_TASK_P_H_
#define __VDS_NETWORK_SOCKET_TASK_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "network_types_p.h"

namespace vds {
  class service_provider;
  
  class _socket_task : public std::enable_shared_from_this<_socket_task>
  {
  public:
    _socket_task();
    virtual ~_socket_task();
    
#ifdef _WIN32
    virtual void process(DWORD dwBytesTransfered) = 0;
    virtual void error(DWORD error_code) = 0;

    static _socket_task * from_overlapped(OVERLAPPED * pOverlapped) {
        return reinterpret_cast<_socket_task *>((uint8_t *)pOverlapped - offsetof(_socket_task, overlapped_));
    }
#else
    virtual void process(uint32_t events) = 0;
#endif//_WIN32
    virtual void check_timeout(const service_provider & sp) = 0;
    virtual void prepare_to_stop(const service_provider & sp) = 0;

  protected:
#ifdef _WIN32
    OVERLAPPED overlapped_;
    WSABUF wsa_buf_;
#else

#endif//_WIN32

  };
}

#endif // __VDS_NETWORK_SOCKET_TASK_P_H_