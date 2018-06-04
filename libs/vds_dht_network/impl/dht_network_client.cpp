/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "dht_network_client.h"
#include "private/dht_network_client_p.h"
#include "chunk_replicas_dbo.h"
#include "messages/dht_find_node.h"
#include "messages/dht_find_node_response.h"
#include "messages/dht_ping.h"
#include "messages/dht_pong.h"
#include "messages/replica_request.h"
#include "deflate.h"
#include "db_model.h"
#include "async_task.h"
#include "inflate.h"
#include "vds_exceptions.h"
#include "local_data_dbo.h"
#include "messages/offer_replica.h"
#include "well_known_node_dbo.h"
#include "url_parser.h"

vds::dht::network::_client::_client(
    const service_provider & sp,
    const const_data_buffer & node_id)
: update_timer_("DHT Network"),
  route_(node_id),
  update_route_table_counter_(0)
{
  for (uint16_t replica = 0; replica < GENERATE_HORCRUX; ++replica) {
    this->generators_[replica].reset(new chunk_generator<uint16_t>(MIN_HORCRUX, replica));
  }
}

std::vector<vds::const_data_buffer> vds::dht::network::_client::save(
    const service_provider &sp,
    database_transaction &t,
    const const_data_buffer & value) {

  std::vector<vds::const_data_buffer> result(GENERATE_HORCRUX);
  for(uint16_t replica = 0; replica < GENERATE_HORCRUX; ++replica) {
    binary_serializer s;
    this->generators_.find(replica)->second->write(s, value.data(), value.size());
    const auto replica_data = s.data();
    const auto replica_id = hash::signature(hash::sha256(), replica_data);

    orm::chunk_replicas_dbo t1;
    t.execute(
        t1.insert(
            t1.id = base64::from_bytes(replica_id),
            t1.replica_data = replica_data,
            t1.last_sync = std::chrono::system_clock::now() - std::chrono::hours(24)
        ));
    result[replica] = replica_id;
  }

  return result;
}

void vds::dht::network::_client::save(
  const service_provider& sp,
  database_transaction& t,
  const std::string& name,
  const const_data_buffer& value) {
  for (uint16_t replica = 0; replica < GENERATE_HORCRUX; ++replica) {
    binary_serializer s;
    this->generators_.find(replica)->second->write(s, value.data(), value.size());
    const auto replica_data = s.data();
    const auto id = base64::from_bytes(replica_id(name, replica));

    sp.get<logger>()->trace(
      ThisModule,
      sp,
      "save replica %s[%d]: %s",
      name.c_str(),
      replica,
      id.c_str());

    orm::chunk_replicas_dbo t1;
    t.execute(
      t1.insert(
        t1.id = id,
        t1.replica_data = replica_data,
        t1.last_sync = std::chrono::system_clock::now() - std::chrono::hours(24)
      ));
  }
}

vds::async_task<> vds::dht::network::_client::apply_message(const service_provider& sp, database_transaction& t,
  const messages::transaction_log_state& message) {
  return this->sync_process_.apply_message(sp, t, message);
}

vds::async_task<> vds::dht::network::_client::apply_message(const service_provider& sp, database_transaction& t,
  const messages::transaction_log_request& message) {
  return this->sync_process_.apply_message(sp, t, message);
}

void vds::dht::network::_client::apply_message(const service_provider& sp, database_transaction& t,
  const messages::transaction_log_record& message) {
  this->sync_process_.apply_message(sp, t, message);
}

void vds::dht::network::_client::apply_message(const service_provider& sp, const messages::dht_find_node& message) {
  std::map<const_data_buffer /*distance*/, std::list<std::shared_ptr<dht_route<std::shared_ptr<dht_session>>::node>>> result_nodes;
  this->route_.search_nodes(sp, message.target_id(), 70, result_nodes);

  std::list<messages::dht_find_node_response::target_node> result;
  for (auto &presult : result_nodes) {
    for (auto & node : presult.second) {
      result.push_back(
        messages::dht_find_node_response::target_node(
          node->node_id_, node->proxy_session_->address().to_string(), node->hops_));
    }
  }

  this->send(sp, message.source_node(), messages::dht_find_node_response(result))
  .execute([sp](const std::shared_ptr<std::exception> & ex) {
    if(ex) {
      sp.get<logger>()->warning(ThisModule, sp, "%s at send dht_find_node_response", ex->what());
    }
  });
}

vds::async_task<>  vds::dht::network::_client::apply_message(
  const service_provider& sp,
  const std::shared_ptr<dht_session> & session,
  const messages::dht_find_node_response& message) {
  auto result = async_task<>::empty();
  for(auto & p : message.nodes()) {
    this->route_.add_node(sp, p.target_id_, session, p.hops_ + 1);
    result = result.then([sp, pthis = this->shared_from_this(), address = p.address_]() {
      pthis->udp_transport_->try_handshake(sp, address)
      .execute([sp, address](const std::shared_ptr<std::exception> & ex) {
        if(ex) {
          sp.get<logger>()->warning(ThisModule, sp, "%s at try handshake %s",
            ex->what(),
            address.c_str());
        }
      });
    });
  }
  return result;
}

void vds::dht::network::_client::apply_message(const service_provider& sp, const std::shared_ptr<dht_session>& session,
  const messages::dht_ping& message) {
  if(message.target_node() == this->current_node_id()) {
    session->send_message(
      sp,
      this->udp_transport_,
      (uint8_t)messages::dht_pong::message_id,
      messages::dht_pong(message.source_node(), this->current_node_id()).serialize()).no_wait();
  }
  else {
    this->route_.for_near(sp, message.target_node(), 1, [this, &message, sp](const std::shared_ptr<dht_route<std::shared_ptr<dht_session>>::node> & candidate)->bool {
      if (dht_object_id::distance(message.target_node(), candidate->node_id_) < dht_object_id::distance(message.target_node(), this->current_node_id())) {
        this->send(sp, message.target_node(), message).no_wait();
      }
      return true;
    });
  }
}

vds::async_task<> vds::dht::network::_client::apply_message(
  const service_provider& sp,
  const std::shared_ptr<dht_session>& session,
  const messages::dht_pong& message) {
  this->route_.mark_pinged(message.source_node(), session->address());

  auto result = async_task<>::empty();
  this->route_.for_near(sp, message.target_node(), 1, [this, &message, &result, sp](const std::shared_ptr<dht_route<std::shared_ptr<dht_session>>::node> & candidate)->bool {
    if (dht_object_id::distance(message.target_node(), candidate->node_id_) < dht_object_id::distance(message.target_node(), this->current_node_id())) {
      result = this->send(sp, message.target_node(), message);
    }
    return true;
  });
  return result;
}

vds::async_task<> vds::dht::network::_client::apply_message(
  const service_provider& sp,
  const std::shared_ptr<dht_session>& session,
  const messages::replica_request& message) {
  auto async_tasks = std::make_shared<async_task<>>(async_task<>::empty());
  return sp.get<db_model>()->async_transaction(
    sp,
    [pthis = this->shared_from_this(), sp, session, message, async_tasks](database_transaction & t){

    orm::chunk_replicas_dbo t1;
    auto st = t.get_reader(
      t1
      .select(t1.replica_data)
      .where(t1.id == base64::from_bytes(message.replica_hash())));

    if (st.execute()) {
      auto data = t1.replica_data.get(st);
      *async_tasks = session->send_message(
        sp,
        pthis->udp_transport_,
        static_cast<uint8_t>(messages::offer_replica::message_id),
        messages::offer_replica(
          message.replica_hash(),
          data,
          pthis->current_node_id()).serialize());
    }
  }).then([async_tasks]() {
    return std::move(*async_tasks);
  });
}

void vds::dht::network::_client::add_session(const service_provider& sp, const std::shared_ptr<dht_session>& session, uint8_t hops) {
  this->route_.add_node(sp, session->partner_node_id(), session, hops);
}

vds::async_task<> vds::dht::network::_client::send(const service_provider& sp, const const_data_buffer& target_node_id,
  const message_type_t message_id, const const_data_buffer& message) {
  auto result = async_task<>::empty();
  this->route_.for_near(
    sp,
    target_node_id,
    1,
    [sp, target_node_id, message_id, message, &result, pthis = this->shared_from_this()](const std::shared_ptr<dht_route<std::shared_ptr<dht_session>>::node> & candidate) {
    result = candidate->send_message(
      sp,
      pthis->udp_transport_,
      message_id,
      message);
    return false;
  });
  return result;
}

void vds::dht::network::_client::send_neighbors(const service_provider& sp,
  const message_type_t message_id, const const_data_buffer& message) {
  this->route_.for_neighbors(
    sp,
    [sp, message_id, message, pthis = this->shared_from_this()](const std::shared_ptr<dht_route<std::shared_ptr<dht_session>>::node> & candidate) {
    candidate->send_message(
      sp,
      pthis->udp_transport_,
      message_id,
      message).no_wait();
    return true;
  });
}

vds::const_data_buffer vds::dht::network::_client::replica_id(const std::string& key, uint16_t replica) {
    auto id = "{" + std::to_string(replica) + "}" + key;
    return hash::signature(hash::sha256(), id.c_str(), id.length());
}


void vds::dht::network::_client::start(const vds::service_provider &sp, uint16_t port) {
  this->udp_transport_ = std::make_shared<udp_transport>();
  this->udp_transport_->start(sp, port, this->current_node_id());

  this->update_timer_.start(sp, std::chrono::seconds(1), [sp, pthis = this->shared_from_this()](){
    std::unique_lock<std::debug_mutex> lock(pthis->update_timer_mutex_);
    if(!pthis->in_update_timer_){
      pthis->in_update_timer_ = true;
      lock.unlock();

      auto async_tasks = std::make_shared<async_task<>>(async_task<>::empty());
      sp.get<db_model>()->async_transaction(sp, [sp, pthis, async_tasks](database_transaction & t){
        *async_tasks = pthis->process_update(sp, t);
        return true;
      })
      .then([async_tasks]() {
        return std::move(*async_tasks);
      })
      .execute([sp, pthis](const std::shared_ptr<std::exception> & ex){
        if(ex){
        }
        std::unique_lock<std::debug_mutex> lock(pthis->update_timer_mutex_);
        pthis->in_update_timer_ = false;
      });

    }

    return !sp.get_shutdown_event().is_shuting_down();
  });
}

void vds::dht::network::_client::stop(const service_provider& sp) {
  this->udp_transport_->stop(sp);
}

vds::async_task<> vds::dht::network::_client::update_route_table(const service_provider& sp) {
  auto result = async_task<>::empty();
  if (0 == this->update_route_table_counter_++ % 100) {
    for (size_t i = 0; i < 8 * this->route_.current_node_id().size(); ++i) {
      auto canditate = dht_object_id::generate_random_id(this->route_.current_node_id(), i);
      result = result.then([pthis = this->shared_from_this(), sp, canditate]() {
        pthis->send(
          sp,
          canditate,
          messages::dht_find_node(canditate, pthis->route_.current_node_id()))
          .execute([sp, canditate](const std::shared_ptr<std::exception> & ex) {
          if (ex) {
            sp.get<logger>()->warning(ThisModule, sp, "%s at update route table %s",
              ex->what(),
              base64::from_bytes(canditate).c_str());
          }
        });
      });
    }
  }
  return result;
}

vds::async_task<> vds::dht::network::_client::process_update(const vds::service_provider &sp, vds::database_transaction &t) {
  return async_series(
    this->route_.on_timer(sp.create_scope("Route update"), this->udp_transport_).then([]() {
      std::cout << "Route update finished\n";
    }),
    this->update_route_table(sp.create_scope("Route table update")).then([]() {
      std::cout << "Route table update finished\n";
    }),
    this->sync_process_.do_sync(sp.create_scope("Sync process"), t).then([]() {
      std::cout << "Sync process finished\n";
    }),
    this->update_wellknown_connection(sp.create_scope("wellknown connection update"), t).then([]() {
      std::cout << "wellknown connection update finished\n";
    }));
}

void vds::dht::network::_client::get_route_statistics(route_statistic& result) {
  this->route_.get_statistics(result);
}

void vds::dht::network::_client::get_session_statistics(session_statistic& session_statistic) {
  this->udp_transport_->get_session_statistics(session_statistic);
}

vds::async_task<>
vds::dht::network::_client::apply_message(
  const vds::service_provider &sp,
  const std::shared_ptr<dht_session> &session,
  const vds::dht::messages::offer_replica &message) {
  return sp.get<db_model>()->async_transaction(
      sp,
      [pthis = this->shared_from_this(), sp, message](database_transaction & t){
        orm::chunk_replicas_dbo t1;
        t.execute(t1.insert_or_ignore(
            t1.id = base64::from_bytes(message.replica_hash()),
            t1.replica_data = message.replica_data(),
            t1.last_sync = std::chrono::system_clock::now()));
      });
}


vds::async_task<> vds::dht::network::_client::restore(
    const vds::service_provider &sp,
    const std::string & name,
    const std::shared_ptr<vds::const_data_buffer> &result,
    const std::chrono::steady_clock::time_point &start) {

  std::vector<vds::const_data_buffer> replica_hashes;
  for (uint16_t replica = 0; replica < GENERATE_HORCRUX; ++replica) {
    replica_hashes.push_back(replica_id(name, replica));
  }

  return this->restore(sp, replica_hashes, result, start);
}

vds::async_task<uint8_t> vds::dht::network::_client::restore_async(
  const vds::service_provider &sp,
  const std::string & name,
  const std::shared_ptr<vds::const_data_buffer> &result) {

  std::vector<vds::const_data_buffer> replica_hashes;
  for (uint16_t replica = 0; replica < GENERATE_HORCRUX; ++replica) {
    replica_hashes.push_back(replica_id(name, replica));
  }

  return this->restore_async(sp, replica_hashes, result);
}

vds::async_task<> vds::dht::network::_client::restore(
    const vds::service_provider &sp,
    const std::vector<vds::const_data_buffer> &replica_hashes,
    const std::shared_ptr<vds::const_data_buffer> &result,
    const std::chrono::steady_clock::time_point &start) {

  return this->restore_async(
    sp,
    replica_hashes,
    result)
      .then([result, pthis = this->shared_from_this(), sp, replica_hashes, start](uint8_t progress) -> async_task<> {
        if (result->size() > 0) {
          return async_task<>::empty();
        }

        if (std::chrono::minutes(10) < (std::chrono::steady_clock::now() - start)) {
          return async_task<>(std::make_shared<vds_exceptions::not_found>());
        }

        return pthis->restore(sp, replica_hashes, result, start);
      });
}

vds::async_task<uint8_t> vds::dht::network::_client::restore_async(
  const vds::service_provider &sp,
  const std::vector<vds::const_data_buffer> &replica_hashes,
  const std::shared_ptr<vds::const_data_buffer> &result) {

  auto result_progress = std::make_shared<uint8_t>();
  auto result_task = std::make_shared<async_task<>>(async_task<>::empty());
  return sp.get<db_model>()->async_read_transaction(
    sp,
    [pthis = this->shared_from_this(), sp, replica_hashes, result, result_task, result_progress](database_transaction &t) -> bool {

    std::vector<uint16_t> replicas;
    std::vector<const_data_buffer> datas;
    std::list<const_data_buffer> unknonw_replicas;

    orm::chunk_replicas_dbo t1;
    for (uint16_t replica = 0; replica < GENERATE_HORCRUX; ++replica) {
      auto st = t.get_reader(
        t1
        .select(t1.replica_data)
        .where(t1.id == base64::from_bytes(replica_hashes[replica])));

      if (st.execute()) {
        replicas.push_back(replica);
        datas.push_back(t1.replica_data.get(st));


        if (replicas.size() >= MIN_HORCRUX) {
          break;
        }
      }
      else {
        unknonw_replicas.push_back(replica_hashes[replica]);
      }
    }

    if (replicas.size() >= MIN_HORCRUX) {
      chunk_restore <uint16_t> restore(MIN_HORCRUX, replicas.data());
      binary_serializer s;
      restore.restore(s, datas);
      *result = s.data();
      *result_progress = 100;
      return true;
    }

    *result_progress = 99 * replicas.size() / MIN_HORCRUX;
    for (const auto &replica : unknonw_replicas) {
      *result_task = result_task->then([pthis, sp, replica]() {
        pthis->send(
          sp,
          replica,
          messages::replica_request(replica, pthis->current_node_id())).no_wait();
      });
    }
    return true;
  })
    .then([result_task]() -> async_task<> {
    return std::move(*result_task);
  })
    .then([pthis = this->shared_from_this(), result_progress]() {
    return *result_progress;
  });
}

vds::async_task<>
vds::dht::network::_client::update_wellknown_connection(
    const vds::service_provider &sp,
    vds::database_transaction &t) {

  auto result = async_task<>::empty();
  orm::well_known_node_dbo t1;
  auto st = t.get_reader(t1.select(t1.addresses));
  while(st.execute()){
    for(const auto & address : split_string(t1.addresses.get(st), ';', true)){
      result = result.then(
        [pthis = this->shared_from_this(), sp, address]() {
        pthis->udp_transport_->try_handshake(sp, address).execute([sp, address](const std::shared_ptr<std::exception> & ex) {
          if (ex) {
            sp.get<logger>()->warning(ThisModule, sp, "%s at send handshake to %s",
              ex->what(), address.c_str());
          }
        });
      });
    }
  }

  return result;
}


void vds::dht::network::client::start(
  const vds::service_provider &sp,
  const vds::const_data_buffer &this_node_id, uint16_t port) {
  this->impl_.reset(new _client(sp, this_node_id));
  this->impl_->start(sp, port);

}

void vds::dht::network::client::stop(const service_provider& sp) {
  if(this->impl_) {
    this->impl_->stop(sp);
  }
}

static const uint8_t pack_block_iv[] = {
  // 0     1     2     3     4     5     6     7
  0xa5, 0xbb, 0x9f, 0xce, 0xc2, 0xe4, 0x4b, 0x91,
  0xa8, 0xc9, 0x59, 0x44, 0x62, 0x55, 0x90, 0x24
};

vds::dht::network::client::chunk_info vds::dht::network::client::save(
  const service_provider& sp,
  database_transaction& t,
  const const_data_buffer& data) {

  auto key_data = hash::signature(hash::sha256(), data);

  if (key_data.size() != symmetric_crypto::aes_256_cbc().key_size()
    || sizeof(pack_block_iv) != symmetric_crypto::aes_256_cbc().iv_size()) {
    throw std::runtime_error("Design error");
  }

  auto key = symmetric_key::create(
    symmetric_crypto::aes_256_cbc(),
    key_data.data(),
    pack_block_iv);

  auto key_data2 = hash::signature(
    hash::sha256(),
    symmetric_encrypt::encrypt(key, data));

  auto key2 = symmetric_key::create(
    symmetric_crypto::aes_256_cbc(),
    key_data2.data(),
    pack_block_iv);

  auto zipped = deflate::compress(data);

  auto crypted_data = symmetric_encrypt::encrypt(key2, zipped);
  return chunk_info
  {
    key_data,
    key_data2,
    this->impl_->save(sp, t, crypted_data)
  };
}

void vds::dht::network::client::save(const service_provider& sp, database_transaction& t, const std::string& key,
  const const_data_buffer& value) {
  this->impl_->save(sp, t, key, value);
}

vds::async_task<vds::const_data_buffer> vds::dht::network::client::restore(
    const vds::service_provider &sp,
    const vds::dht::network::client::chunk_info &block_id) {
  auto result = std::make_shared<const_data_buffer>();
  return this->impl_->restore(sp, block_id.replica_hashes, result, std::chrono::steady_clock::now())
      .then([result, block_id]() {

    auto key2 = symmetric_key::create(
      symmetric_crypto::aes_256_cbc(),
      block_id.key.data(),
      pack_block_iv);

    auto zipped = symmetric_decrypt::decrypt(key2, *result);
    auto original_data = inflate::decompress(zipped.data(), zipped.size());

    vds_assert(block_id.id == hash::signature(hash::sha256(), original_data));

    return original_data;
  });
}

vds::async_task<vds::const_data_buffer> vds::dht::network::client::restore(const service_provider& sp,
  const std::string& key) {
  auto result = std::make_shared<const_data_buffer>();
  return this->impl_->restore(sp, key, result, std::chrono::steady_clock::now())
  .then([result]() {
    return *result;
  });
}

vds::async_task<uint8_t, vds::const_data_buffer> vds::dht::network::client::restore_async(
  const service_provider& sp, const std::string& key) {
  auto result = std::make_shared<const_data_buffer>();
  return this->impl_->restore_async(sp, key, result)
    .then([result](uint8_t percent) {
    return vds::async_task<uint8_t, vds::const_data_buffer>::result(percent, *result);
  });
}

const vds::const_data_buffer &vds::dht::network::client::current_node_id() const {
  return this->impl_->current_node_id();
}

void vds::dht::network::client::get_route_statistics(route_statistic& result) {
  this->impl_->get_route_statistics(result);
}

void vds::dht::network::client::get_session_statistics(session_statistic& session_statistic) {
  this->impl_->get_session_statistics(session_statistic);
}
