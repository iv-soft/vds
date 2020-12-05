#ifndef __VDS_NETWORK_UDP_SOCKET_H_
#define __VDS_NETWORK_UDP_SOCKET_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "async_buffer.h"
#include "const_data_buffer.h"
#include "network_address.h"
#include "socket_base.h"
#include "network_service.h"

namespace vds {
  class _udp_socket;
  class _udp_datagram;
  class _udp_server;
  class _udp_client;
  
  class udp_datagram
  {
  public:


    udp_datagram();
    udp_datagram(const udp_datagram & other);
    udp_datagram(udp_datagram && other);

    udp_datagram(
      const network_address & address,
      const void * data,
      size_t data_size);

    udp_datagram(
      const network_address & address,
      const const_data_buffer & data);
    
    ~udp_datagram();

    network_address address() const;

    const uint8_t * data() const;
    size_t data_size() const;

    udp_datagram & operator = (const udp_datagram & other);
    udp_datagram & operator = (udp_datagram && other);

    _udp_datagram * operator -> () const { return this->impl_; }

  private:
    friend class _udp_datagram;
    udp_datagram(_udp_datagram * impl);

    _udp_datagram * impl_;
  };

  //class udp_datagram_reader : public std::enable_shared_from_this<udp_datagram_reader> {
  //public:
  //  vds::async_task<vds::expected<udp_datagram>> read_async();
  //};


  class udp_socket
#ifndef _WIN32
    : public socket_base
#else
    : public std::enable_shared_from_this<udp_socket>
#endif//_WIN32
  {
  public:
    udp_socket();
    udp_socket(udp_socket && original);
    udp_socket(const udp_socket & original) = delete;

    ~udp_socket();

    sa_family_t family() const;

    udp_socket &operator = (const udp_socket & original) = delete;
    udp_socket & operator = (udp_socket && original);

    expected<void> start(const service_provider * sp, lambda_holder_t<async_task<expected<bool>>, expected<udp_datagram>> read_handler);

    async_task<expected<void>> write_async(const service_provider* sp, const udp_datagram& message);

#ifndef _WIN32
    void stop() override;
#else
    void stop();
#endif//_WIN32
    _udp_socket * operator -> () const { return this->impl_; }

    static expected<std::shared_ptr<udp_socket>> create(const service_provider * sp, sa_family_t af);

    expected<void> join_membership(sa_family_t af, const std::string & group_address);
    expected<void> broadcast(sa_family_t af, const std::string & group_address, u_short port, const const_data_buffer & message);

#ifndef _WIN32
        expected<void> process(uint32_t events) override;
#endif

  private:
    friend class _udp_socket;

    udp_socket(_udp_socket * impl)
      : impl_(impl)
    {
    }

    _udp_socket * impl_;
  };

  class udp_server
  {
  public:
    udp_server();
    ~udp_server();

    expected<void> start(
      const service_provider * sp,
      const network_address & address,
      lambda_holder_t<async_task<expected<bool>>, expected<udp_datagram>> read_handler);

    void prepare_to_stop();
    void stop();

	  const std::shared_ptr<udp_socket> & socket() const;

    _udp_server *operator ->()const {
      return this->impl_;
    }

    const network_address & address() const;
    async_task<expected<void>> write_async(const service_provider* sp, const udp_datagram& message);

  private:
    _udp_server * impl_;
  };  
}

#endif//__VDS_NETWORK_UDP_SOCKET_H_
