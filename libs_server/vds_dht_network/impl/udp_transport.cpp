/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/


#include "stdafx.h"
#include "udp_transport.h"
#include "messages/dht_route_messages.h"
#include "dht_session.h"
#include "logger.h"
#include "dht_network_client.h"
#include "dht_network_client_p.h"

vds::dht::network::udp_transport::udp_transport(){
}

vds::dht::network::udp_transport::~udp_transport() {
}

vds::expected<void> vds::dht::network::udp_transport::start(
  const service_provider * sp,
  const std::shared_ptr<asymmetric_public_key> & node_public_key,
  const std::shared_ptr<asymmetric_private_key> & node_key,
  const network_address & bind_interface,
  bool dev_network) {

  this->MAGIC_LABEL = dev_network ? 0x54445331 : 0x56445331;

  this->send_thread_ = std::make_shared<thread_apartment>(sp);
  this->sp_ = sp;
  GET_EXPECTED_VALUE(this->this_node_id_, node_public_key->fingerprint());
  this->node_public_key_ = node_public_key;
  this->node_key_ = node_key;

  GET_EXPECTED_VALUE(this->writer_, this->server_.start(sp, bind_interface,
    [pthis = this->shared_from_this()](expected<udp_datagram> datagram_result) {
      return static_cast<udp_transport *>(pthis.get())->read_handler(std::move(datagram_result));
    }));
  sp->get<logger>()->debug("Binding to %s", bind_interface.to_string().c_str());
  CHECK_EXPECTED(this->server_.socket()->join_membership(bind_interface.family(), "FF02::1"));

  return expected<void>();
}

void vds::dht::network::udp_transport::stop() {
  this->server_.stop();
}

vds::async_task<vds::expected<void>>
vds::dht::network::udp_transport::write_async( const udp_datagram& datagram) {
  auto result = std::make_shared<vds::async_result<vds::expected<void>>>();
  this->send_thread_->schedule([result, this, datagram]() ->expected<void> {
    auto res = std::make_shared<expected<void>>(this->writer_->write_async(datagram).get());
    mt_service::async(this->sp_, [res, result]() {
      result->set_value(std::move(*res));
    });
    return expected<void>();
  });

  return result->get_future();

  //  std::unique_lock<std::debug_mutex> lock(this->write_mutex_);
  //  while(this->write_in_progress_) {
  //    this->write_cond_.wait(*reinterpret_cast<std::unique_lock<std::mutex> *>(&lock));
  //  }
  //  this->write_in_progress_ = true;
  //#ifdef _DEBUG
  //#ifndef _WIN32
  //  this->owner_id_ = syscall(SYS_gettid);
  //#else
  //  this->owner_id_ = GetCurrentThreadId();
  //#endif
  //#endif//_DEBUG
  //co_await this->writer_->write_async(sp, datagram);
}

vds::async_task<vds::expected<void>> vds::dht::network::udp_transport::try_handshake(
                                                                  const std::string& address_str) {

  GET_EXPECTED_ASYNC(address, network_address::parse(this->server_.address().family(), address_str));

  this->sessions_mutex_.lock();
  auto p = this->sessions_.find(address);
  if (this->sessions_.end() == p) {
    this->sessions_mutex_.unlock();
  }
  else {
    auto & session_info = p->second;
    this->sessions_mutex_.unlock();

    session_info.session_mutex_.lock();
    if (session_info.blocked_) {
      if ((std::chrono::steady_clock::now() - session_info.update_time_) <= std::chrono::minutes(1)) {
        session_info.session_mutex_.unlock();
        co_return expected<void>();
      }
    }
    else if(session_info.session_) {
      session_info.session_mutex_.unlock();
      co_return expected<void>();
    }
    session_info.session_mutex_.unlock();
  }

  resizable_data_buffer out_message;
  CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)protocol_message_type_t::HandshakeBroadcast));
  CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL >> 24)));
  CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL >> 16)));
  CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL >> 8)));
  CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL)));
  CHECK_EXPECTED_ASYNC(out_message.add(PROTOCOL_VERSION));

  binary_serializer bs;
  CHECK_EXPECTED_ASYNC(bs << this->node_public_key_->der());

  CHECK_EXPECTED_ASYNC(out_message.add(bs.move_data()));

  co_return co_await this->write_async(udp_datagram(
    address,
    out_message.move_data()));
}

vds::expected<void> vds::dht::network::udp_transport::broadcast_handshake()
{
  resizable_data_buffer out_message;
  CHECK_EXPECTED(out_message.add((uint8_t)protocol_message_type_t::HandshakeBroadcast));
  CHECK_EXPECTED(out_message.add((uint8_t)(MAGIC_LABEL >> 24)));
  CHECK_EXPECTED(out_message.add((uint8_t)(MAGIC_LABEL >> 16)));
  CHECK_EXPECTED(out_message.add((uint8_t)(MAGIC_LABEL >> 8)));
  CHECK_EXPECTED(out_message.add((uint8_t)(MAGIC_LABEL)));
  CHECK_EXPECTED(out_message.add(PROTOCOL_VERSION));

  binary_serializer bs;
  CHECK_EXPECTED(bs << this->node_public_key_->der());

  CHECK_EXPECTED(out_message.add(bs.move_data()));
  
  auto message = out_message.move_data();
  (void)this->server_.socket()->broadcast(this->server_.socket()->family(), "FF02::1", 8050, message);

  return expected<void>();
}

vds::async_task<vds::expected<void>> vds::dht::network::udp_transport::on_timer() {
  std::list<std::shared_ptr<dht_session>> sessions;

  this->sessions_mutex_.lock_shared();
  for(auto & p : this->sessions_) {
    if (p.second.session_) {
      sessions.push_back(p.second.session_);
    }
  }
  this->sessions_mutex_.unlock_shared();

  for(auto & s : sessions) {
    CHECK_EXPECTED_ASYNC(co_await s->on_timer());
  }

  co_return expected<void>();
}

void vds::dht::network::udp_transport::get_session_statistics(session_statistic& session_statistic) {
  session_statistic.send_queue_size_ = this->send_thread_->size();

  std::shared_lock<std::shared_mutex> lock(this->sessions_mutex_);
  for (const auto& p : this->sessions_) {
    const auto & session = p.second;
    if (session.session_) {
      session_statistic.items_.push_back(session.session_->get_statistic());
    }
  }
}


vds::async_task<vds::expected<bool>> vds::dht::network::udp_transport::read_handler(expected<udp_datagram> datagram_result) {
    if(datagram_result.has_error()) {
      if(this->sp_->get_shutdown_event().is_shuting_down()) {
        co_return false;
      }
      co_return true;
    }

    udp_datagram datagram = std::move(datagram_result.value());

    if (this->sp_->get_shutdown_event().is_shuting_down()) {
      co_return false;
    }

    this->sessions_mutex_.lock();
    auto & session_info = this->sessions_[datagram.address()];
    this->sessions_mutex_.unlock();

    session_info.session_mutex_.lock();
    if (session_info.blocked_) {
      if ((std::chrono::steady_clock::now() - session_info.update_time_) > std::chrono::minutes(1)
        && (*datagram.data() == (uint8_t)protocol_message_type_t::Handshake
        || *datagram.data() == (uint8_t)protocol_message_type_t::HandshakeBroadcast
        || *datagram.data() == (uint8_t)protocol_message_type_t::Welcome
        || *datagram.data() == (uint8_t)protocol_message_type_t::Failed)) {
        logger::get(this->sp_)->trace(ThisModule, "Unblock session %s", datagram.address().to_string().c_str());
        session_info.blocked_ = false;
      }
      else {
        session_info.session_mutex_.unlock();
        if (*datagram.data() != (uint8_t)protocol_message_type_t::Failed
          && *datagram.data() != (uint8_t)protocol_message_type_t::Handshake
          && *datagram.data() != (uint8_t)protocol_message_type_t::HandshakeBroadcast
          && *datagram.data() != (uint8_t)protocol_message_type_t::Welcome) {
          uint8_t out_message[] = { (uint8_t)protocol_message_type_t::Failed };
          (void)co_await this->write_async(udp_datagram(datagram.address(),
              const_data_buffer(out_message, sizeof(out_message))));
        }
        co_return true;
      }
    }

    switch ((protocol_message_type_t)datagram.data()[0]) {
    case protocol_message_type_t::HandshakeBroadcast:
    case protocol_message_type_t::Handshake: {

      if (session_info.session_) {
        session_info.session_mutex_.unlock();
        co_return true;
      }

      if (
        (uint8_t)(MAGIC_LABEL >> 24) == datagram.data()[1]
        && (uint8_t)(MAGIC_LABEL >> 16) == datagram.data()[2]
        && (uint8_t)(MAGIC_LABEL >> 8) == datagram.data()[3]
        && (uint8_t)(MAGIC_LABEL) == datagram.data()[4]
        && PROTOCOL_VERSION == datagram.data()[5]) {
        binary_deserializer bd(datagram.data() + 6, datagram.data_size() - 6);
        const_data_buffer partner_node_public_key_der;
        CHECK_EXPECTED_ASYNC(bd >> partner_node_public_key_der);
        GET_EXPECTED_ASYNC(partner_node_public_key, asymmetric_public_key::parse_der(partner_node_public_key_der));
        GET_EXPECTED_ASYNC(partner_node_id, partner_node_public_key.fingerprint());
        if (partner_node_id == this->this_node_id_) {
          session_info.session_mutex_.unlock();
          break;
        }

        bool is_duplicate = false;
        this->sessions_mutex_.lock();
        for (auto p = this->sessions_.begin(); p != this->sessions_.end(); ++p) {
          if (p->second.session_ && p->second.session_->partner_node_id() == partner_node_id) {
            if (p->second.blocked_) {
              this->sessions_.erase(p);
            }
            else {
              is_duplicate = true;
            }
            break;
          }
        }
        this->sessions_mutex_.unlock();

        if (is_duplicate) {
          session_info.session_mutex_.unlock();
          break;
        }

        session_info.update_time_ = std::chrono::steady_clock::now();
        session_info.session_key_.resize(32);
        crypto_service::rand_bytes(session_info.session_key_.data(), session_info.session_key_.size());

        GET_EXPECTED_ASYNC(encrypted_key, partner_node_public_key.encrypt(session_info.session_key_));

        session_info.session_ = std::make_shared<dht_session>(
          this->sp_,
          datagram.address(),
          this->this_node_id_,
          std::move(partner_node_public_key),
          partner_node_id,
          session_info.session_key_,
          this->shared_from_this());

        resizable_data_buffer out_message;
        CHECK_EXPECTED_ASYNC(out_message.add(static_cast<uint8_t>(protocol_message_type_t::Welcome)));
        CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL >> 24)));
        CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL >> 16)));
        CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL >> 8)));
        CHECK_EXPECTED_ASYNC(out_message.add((uint8_t)(MAGIC_LABEL)));

        binary_serializer bs;
        CHECK_EXPECTED_ASYNC(bs << this->node_public_key_->der());
        CHECK_EXPECTED_ASYNC(bs << encrypted_key);

        session_info.session_mutex_.unlock();

        this->sp_->get<logger>()->debug(ThisModule, "Add session %s", datagram.address().to_string().c_str());
        CHECK_EXPECTED_ASYNC(co_await (*this->sp_->get<client>())->add_session(session_info.session_, 0));

        CHECK_EXPECTED_ASYNC(out_message.add(bs.move_data()));
        CHECK_EXPECTED_ASYNC(co_await this->write_async(udp_datagram(datagram.address(), out_message.move_data())));
        CHECK_EXPECTED_ASYNC(co_await this->sp_->get<imessage_map>()->on_new_session(partner_node_id));
      }
      else {
        session_info.session_mutex_.unlock();
        co_return true;
      }
      break;
    }
    case protocol_message_type_t::Welcome: {
      if (datagram.data_size() > 5
        && (uint8_t)(MAGIC_LABEL >> 24) == datagram.data()[1]
        && (uint8_t)(MAGIC_LABEL >> 16) == datagram.data()[2]
        && (uint8_t)(MAGIC_LABEL >> 8) == datagram.data()[3]
        && (uint8_t)(MAGIC_LABEL) == datagram.data()[4]) {

        const_data_buffer public_key_buffer;
        const_data_buffer key_buffer;
        binary_deserializer bd(datagram.data() + 5, datagram.data_size() - 5);
        CHECK_EXPECTED_ASYNC(bd >> public_key_buffer);
        CHECK_EXPECTED_ASYNC(bd >> key_buffer);

        GET_EXPECTED_ASYNC(public_key, asymmetric_public_key::parse_der(public_key_buffer));
        GET_EXPECTED_ASYNC(key, this->node_key_->decrypt(key_buffer));

        GET_EXPECTED_ASYNC(partner_id, public_key.fingerprint());

        auto session = std::make_shared<dht_session>(
          this->sp_,
          datagram.address(),
          this->this_node_id_,
          std::move(public_key),
          partner_id,
          key,
          this->shared_from_this());

        session_info.session_ = session;
        session_info.session_mutex_.unlock();

        const auto from_address = datagram.address().to_string();
        this->sp_->get<logger>()->debug(ThisModule, "Add session %s", from_address.c_str());
        CHECK_EXPECTED_ASYNC(co_await (*this->sp_->get<client>())->add_session(session, 0));
        CHECK_EXPECTED_ASYNC(co_await this->sp_->get<imessage_map>()->on_new_session(partner_id));
      }
      else {
        session_info.session_mutex_.unlock();
        co_return vds::make_unexpected<std::runtime_error>("Invalid protocol");
      }

      break;
    }
    case protocol_message_type_t::Failed: {
      logger::get(this->sp_)->trace(ThisModule, "Block session %s", datagram.address().to_string().c_str());
      if (session_info.session_) {
        (*this->sp_->get<client>())->remove_session(session_info.session_);
        session_info.session_.reset();
      }
      session_info.blocked_ = true;
      session_info.update_time_ = std::chrono::steady_clock::now();
      session_info.session_mutex_.unlock();
      break;
    }
    default: {
      if (session_info.session_) {
        auto session = session_info.session_;
        session_info.session_mutex_.unlock();

        bool failed = false;
        auto result = co_await session->process_datagram(
          const_data_buffer(datagram.data(), datagram.data_size()));
        if (result.has_error()) {
          logger::get(this->sp_)->debug(ThisModule, "%s at process message from %s",
            result.error()->what(),
            datagram.address().to_string().c_str());
          failed = true;
        }

        if (failed) {
          session_info.session_mutex_.lock();
          logger::get(this->sp_)->trace(ThisModule, "Block session %s", datagram.address().to_string().c_str());
          (*this->sp_->get<client>())->remove_session(session_info.session_);
          session_info.blocked_ = true;
          session_info.session_.reset();
          session_info.update_time_ = std::chrono::steady_clock::now();
          session_info.session_mutex_.unlock();

          uint8_t out_message[] = { (uint8_t)protocol_message_type_t::Failed };
          CHECK_EXPECTED_ASYNC(co_await this->write_async(udp_datagram(datagram.address(),
            const_data_buffer(out_message, sizeof(out_message)))));
        }
      }
      else {
        logger::get(this->sp_)->trace(ThisModule, "Block session %s", datagram.address().to_string().c_str());
        if (session_info.session_) {
          (*this->sp_->get<client>())->remove_session(session_info.session_);
        }
        session_info.blocked_ = true;
        session_info.session_.reset();
        session_info.update_time_ = std::chrono::steady_clock::now();
        session_info.session_mutex_.unlock();

        uint8_t out_message[] = { (uint8_t)protocol_message_type_t::Failed };
        (void)co_await this->write_async(udp_datagram(datagram.address(),
            const_data_buffer(out_message, sizeof(out_message))));
      }
      break;
    }
  }

  co_return true;
}

