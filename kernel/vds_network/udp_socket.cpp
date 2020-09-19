#include "stdafx.h"
#include "udp_socket.h"
#include "private/udp_socket_p.h"

vds::udp_datagram::udp_datagram()
  : impl_(nullptr)
{
}

vds::udp_datagram::udp_datagram(const udp_datagram& other)
: impl_(new _udp_datagram(*other.impl_)){
}

vds::udp_datagram::udp_datagram(udp_datagram && other)
  : impl_(other.impl_) {
  other.impl_ = nullptr;
}

vds::udp_datagram::udp_datagram(vds::_udp_datagram* impl)
  : impl_(impl)
{
}

vds::udp_datagram::udp_datagram(
  const network_address & address,
  const void* data,
  size_t data_size)
: impl_(new _udp_datagram(address, data, data_size))
{
}

vds::udp_datagram::udp_datagram(
  const network_address & address,
  const const_data_buffer & data)
  : impl_(new _udp_datagram(address, data))
{
}

vds::udp_datagram::~udp_datagram() {
  delete this->impl_;
}

vds::network_address vds::udp_datagram::address() const {
  return this->impl_->address();
}


//void vds::udp_datagram::reset(const std::string & server, uint16_t port, const void * data, size_t data_size, bool check_max_safe_data_size /*= true*/)
//{
//  if (check_max_safe_data_size && max_safe_data_size < data_size) {
//    return vds::make_unexpected<std::runtime_error>("Data too long");
//  }
//
//  this->impl_.reset(new _udp_datagram(server, port, data, data_size));
//}

const uint8_t * vds::udp_datagram::data() const
{
  return this->impl_->data();
}

size_t vds::udp_datagram::data_size() const
{
  return this->impl_ ? this->impl_->data_size() : 0;
}

vds::udp_datagram& vds::udp_datagram::operator=(const udp_datagram& other) {
  delete this->impl_;
  this->impl_ = new _udp_datagram(*other.impl_);
  return *this;
}

vds::udp_datagram& vds::udp_datagram::operator=(udp_datagram&& other) {
  if (this != &other) {
    delete this->impl_;
    this->impl_ = other.impl_;
    other.impl_ = nullptr;
  }
  return *this;

}

vds::async_task<vds::expected<void>> vds::udp_datagram_writer::write_async( const udp_datagram& message) {
  return static_cast<_udp_send *>(this)->write_async(message);
}

vds::udp_socket::udp_socket()
{
}

vds::udp_socket::~udp_socket()
{
  delete this->impl_;
}

sa_family_t vds::udp_socket::family() const
{
  return this->impl_->family();
}

#ifdef _WIN32

vds::expected<std::shared_ptr<vds::udp_datagram_writer>>
vds::udp_socket::start(const service_provider * sp, lambda_holder_t<async_task<expected<bool>>, expected<udp_datagram>> read_handler)
{
  auto reader = new _udp_receive(sp, this->shared_from_this(), std::move(read_handler));
  reader->schedule_read();

  return std::make_shared<_udp_send>(sp, this->shared_from_this());
}

#else//_WIN32
vds::expected<std::shared_ptr<vds::udp_datagram_writer>>
vds::udp_socket::start(const service_provider * sp, lambda_holder_t<async_task<expected<bool>>, expected<udp_datagram>> read_handler)
{
  return this->impl_->start(sp, this->shared_from_this(), std::move(read_handler));
}

vds::expected<std::shared_ptr<vds::udp_datagram_writer>>
vds::_udp_socket::start(
  const vds::service_provider *sp,
  const std::shared_ptr<vds::socket_base> &owner,
  lambda_holder_t<async_task<expected<bool>>, expected<udp_datagram>> read_handler) {

  auto read_task = std::make_shared<_udp_receive>(sp, owner, std::move(read_handler));
  auto write_task = std::make_shared<_udp_send>(sp, owner);

  this->read_task_ = read_task;
  this->write_task_ = write_task;

  CHECK_EXPECTED(read_task->schedule_read());
  return write_task;
}
#endif

void vds::udp_socket::stop()
{
  this->impl_->close();
}

vds::expected<std::shared_ptr<vds::udp_socket>> vds::udp_socket::create(
  const service_provider * sp,
    sa_family_t af)
{
#ifdef _WIN32
  auto s = WSASocket(af, SOCK_DGRAM, IPPROTO_UDP, NULL, 0, WSA_FLAG_OVERLAPPED);
  if (INVALID_SOCKET == s) {
    auto error = WSAGetLastError();
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "create socket");
  }

  /*************************************************************/
  /* Allow socket descriptor to be reuseable                   */
  /*************************************************************/
  int on = 1;
  if (0 > setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*)&on, sizeof(on))) {
    auto error = errno;
    ::close(s);
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "Allow socket descriptor to be reuseable");
  }

  if (af == AF_INET6) {
    int no = 0;
    if (setsockopt(s, IPPROTO_IPV6, IPV6_V6ONLY, (char *)&no, sizeof(no)) < 0) {
      auto error = WSAGetLastError();
      ::close(s);
      return vds::make_unexpected<std::system_error>(error, std::system_category(), "set IPV6_V6ONLY=0");
    }
  }
  CHECK_EXPECTED((*sp->get<network_service>())->associate(s));
#else
  auto s = socket(af, SOCK_DGRAM, IPPROTO_UDP);
  if (0 > s) {
    auto error = errno;
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "create socket");
  }

  /*************************************************************/
  /* Allow socket descriptor to be reuseable                   */
  /*************************************************************/
  int on = 1;
  if (0 > setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on))) {
    auto error = errno;
    ::close(s);
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "Allow socket descriptor to be reuseable");
  }

  /*************************************************************/
  /* Set socket to be nonblocking. All of the sockets for    */
  /* the incoming connections will also be nonblocking since  */
  /* they will inherit that state from the listening socket.   */
  /*************************************************************/
  if (0 > ioctl(s, FIONBIO, (char *)&on)) {
    auto error = errno;
    ::close(s);
    return vds::make_unexpected<std::system_error>(error, std::system_category(), "Set socket to be nonblocking");
  }

  if (af == AF_INET6) {
    int no = 0;
    if (setsockopt(s, IPPROTO_IPV6, IPV6_V6ONLY, (char *)&no, sizeof(no)) < 0) {
      auto error = errno;
      ::close(s);
      return vds::make_unexpected<std::system_error>(error, std::system_category(), "set IPV6_V6ONLY=0");
    }
  }

#endif
  return std::shared_ptr<udp_socket>(new udp_socket(new _udp_socket(sp, s, af)));
}

vds::expected<void> vds::udp_socket::join_membership(sa_family_t af, const std::string & group_address)
{
  if (AF_INET6 == af) {
    struct ipv6_mreq group;
    group.ipv6mr_interface = 0;
    if (0 > inet_pton(AF_INET6, group_address.c_str(), &group.ipv6mr_multiaddr)) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "parse address " + group_address);
    }

    //int ifidx = 0;
    //if (setsockopt((*this)->handle(), IPPROTO_IPV6, IPV6_MULTICAST_IF, (const char*)&ifidx, sizeof(ifidx))) {
    //  int error = errno;
    //  return make_unexpected<std::system_error>(error, std::generic_category(), "set multicast interface");
    //}

    int loopBack = 0;
    if (0 > setsockopt((*this)->handle(), IPPROTO_IPV6, IPV6_MULTICAST_LOOP, (const char*)&loopBack, sizeof(loopBack))) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "set multicast loop");
    }
    
    int mcastTTL = 1;
    if (0 > setsockopt((*this)->handle(), IPPROTO_IPV6, IPV6_MULTICAST_HOPS, (const char*)&mcastTTL, sizeof(mcastTTL))) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "set multicast hops");
    }

    if (0 > setsockopt((*this)->handle(), IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, (const char *)&group, sizeof group)) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "IPV6_ADD_MEMBERSHIP " + group_address);
    }
  } else {
    int broadcast = 1;
    if (setsockopt((*this)->handle(), SOL_SOCKET, SO_BROADCAST, (const char*)&broadcast, sizeof(broadcast)) < 0) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "set broadcast " + group_address);
    }
  }

  return expected<void>();
}

vds::expected<void> vds::udp_socket::broadcast(sa_family_t af, const std::string & group_address, u_short port, const const_data_buffer & message)
{
  if (AF_INET6 == af) {
    struct sockaddr_in6 address = { AF_INET6, htons(port) };
    if (0 > inet_pton(AF_INET6, group_address.c_str(), &address.sin6_addr)) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "parse address " + group_address);
    }

    if (sendto((*this)->handle(), (const char *)message.data(), message.size(), 0, (struct sockaddr*)&address, sizeof(address)) < 0) {
      int error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "sendto");
    }
  }
  else {
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr  = INADDR_BROADCAST;
         
    if(sendto((*this)->handle(), (const char *)message.data(), message.size(), 0, (sockaddr *)&address, sizeof(address)) < 0) {
#ifdef _WIN32
      const auto error = WSAGetLastError();
      return make_unexpected<std::system_error>(error, std::system_category(), "sendto");
#else
      const auto error = errno;
      return make_unexpected<std::system_error>(error, std::generic_category(), "sendto");
#endif
    }
  }

  return expected<void>();
}

#ifndef _WIN32
vds::expected<void> vds::udp_socket::process(uint32_t events) {
  return this->impl_->process(events);
}

vds::expected<void> vds::_udp_socket::process(uint32_t events) {
  if (EPOLLOUT == (EPOLLOUT & events)) {
    if (0 == (this->event_masks_ & EPOLLOUT)) {
      return vds::make_unexpected<std::runtime_error>("Invalid state");
    }

    auto w = this->write_task_.lock();
    if(w) {
      CHECK_EXPECTED(w->process());
    }
  }

  if (EPOLLIN == (EPOLLIN & events)) {
    if (0 == (this->event_masks_ & EPOLLIN)) {
      return vds::make_unexpected<std::runtime_error>("Invalid state");
    }

    CHECK_EXPECTED(this->read_task_->process());
  }
  return expected<void>();
}

#endif//_WIN32

vds::udp_server::udp_server()
  : impl_(nullptr)
{
}

vds::udp_server::~udp_server()
{
  delete this->impl_;
}

vds::expected<std::shared_ptr<vds::udp_datagram_writer>> vds::udp_server::start(
  const service_provider * sp,
  const network_address & address,
  lambda_holder_t<async_task<expected<bool>>, expected<udp_datagram>> read_handler)
{
  vds_assert(nullptr == this->impl_);
  this->impl_ = new _udp_server(address);
  return this->impl_->start(sp, std::move(read_handler));
}

void vds::udp_server::stop()
{
  if (nullptr != this->impl_) {
    this->impl_->stop();
    delete this->impl_;
    this->impl_ = nullptr;
  }
}

const std::shared_ptr<vds::udp_socket> &vds::udp_server::socket() const {
  return this->impl_->socket();
}


const vds::network_address& vds::udp_server::address() const {
  return this->impl_->address();
}

void vds::udp_server::prepare_to_stop() {
  this->impl_->prepare_to_stop();
}

