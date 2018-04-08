/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <messages/offer_replica.h>
#include <messages/replica_not_found.h>
#include "stdafx.h"
#include "private/dht_sync_process.h"
#include "channel_local_cache_dbo.h"
#include "dht_network_client.h"
#include "messages/channel_log_state.h"
#include "transaction_log_record_dbo.h"
#include "chunk_replica_map_dbo.h"
#include "private/dht_network_client_p.h"
#include "messages/offer_move_replica.h"
#include "chunk_replica_data_dbo.h"
#include "transaction_log_unknown_record_dbo.h"
#include "messages/channel_log_request.h"
#include "messages/channel_log_record.h"
#include "transaction_log.h"

vds::async_task<> vds::dht::network::sync_process::query_unknown_records(const service_provider& sp, database_transaction& t) {
  std::map<const_data_buffer, std::list<const_data_buffer>> record_ids;
  orm::transaction_log_unknown_record_dbo t1;
  auto st = t.get_reader(t1.select(t1.id, t1.channel_id));

  while (st.execute()) {
    record_ids[t1.channel_id.get(st)].push_back(base64::to_bytes(t1.id.get(st)));
  }

  auto result = async_task<>::empty();
  auto & client = *sp.get<dht::network::client>();
  for (auto p : record_ids) {

    std::string log_message;
    for (const auto & r : p.second) {
      log_message += base64::from_bytes(r);
      log_message += ' ';
    }

    sp.get<logger>()->trace(
      ThisModule,
      sp,
      "Query log records %s of channel %s",
      log_message.c_str(),
      base64::from_bytes(p.first).c_str());
    result = result.then([client, sp, channel_id = p.first, requests = p.second]() {
      client->send(
        sp,
        channel_id,
        messages::channel_log_request(
          channel_id,
          requests,
          client->current_node_id()));
    });
  }
  return result;
}

vds::async_task<> vds::dht::network::sync_process::do_sync(
  const service_provider& sp,
  database_transaction& t) {
  return async_series(
    this->sync_local_channels(sp, t),
    this->query_unknown_records(sp, t),
    this->sync_replicas(sp, t)
  );
}

vds::async_task<> vds::dht::network::sync_process::apply_message(
  const service_provider& sp,
  database_transaction& t,
  const messages::channel_log_state& message) {

  orm::transaction_log_record_dbo t1;
  std::list<const_data_buffer> requests;
  for (auto & p : message.leafs()) {
    auto st = t.get_reader(
      t1.select(t1.state)
      .where(t1.id == base64::from_bytes(p)));
    if (!st.execute()) {
      //Not found
      requests.push_back(p);
    }
  }
  auto result = async_task<>::empty();
  if (!requests.empty()) {
    std::string log_message;
    for (const auto & r : requests) {
      log_message += base64::from_bytes(r);
      log_message += ' ';
    }

    sp.get<logger>()->trace(
      ThisModule,
      sp,
      "Query log records %s of channel %s",
      log_message.c_str(),
      base64::from_bytes(message.channel_id()).c_str());

    result = result.then([sp, message, requests]() {
      auto & client = *sp.get<vds::dht::network::client>();
      return client->send(
        sp,
        message.source_node(),
        messages::channel_log_request(
          message.channel_id(),
          requests,
          client->current_node_id()));
    });
  }
  else {
    orm::transaction_log_record_dbo t2;
    auto st = t.get_reader(
      t2.select(t2.id)
      .where(t2.channel_id == base64::from_bytes(message.channel_id())
        && t2.state == (int)orm::transaction_log_record_dbo::state_t::leaf));

    std::list<const_data_buffer> current_state;
    while (st.execute()) {
      auto id = base64::to_bytes(t2.id.get(st));
      
      auto exist = false;
      for (auto & p : message.leafs()) {
        if (id == p) {
          exist = true;
          break;;
        }
      }

      if(!exist) {
        current_state.push_back(id);
      }
    }

    if(!current_state.empty()) {
      result = result.then([sp, message, current_state]() {
        auto & client = *sp.get<vds::dht::network::client>();
        return client->send(
          sp,
          message.source_node(),
          messages::channel_log_state(
            message.channel_id(),
            current_state,
            client->current_node_id()));
      });
    }

    std::string log_message;
    for (const auto & r : message.leafs()) {
      log_message += base64::from_bytes(r);
      log_message += ' ';
    }

    sp.get<logger>()->trace(
      ThisModule,
      sp,
      "log records %s of channel %s already exists",
      log_message.c_str(),
      base64::from_bytes(message.channel_id()).c_str());
  }

  return result;
}

vds::async_task<> vds::dht::network::sync_process::apply_message(const service_provider& sp, database_transaction& t,
  const messages::channel_log_request& message) {
  auto result = async_task<>::empty();
  orm::transaction_log_record_dbo t1;
  std::list<const_data_buffer> requests;
  for (auto & p : message.requests()) {
    auto st = t.get_reader(t1.select(t1.channel_id, t1.data).where(t1.id == base64::from_bytes(p)));
    if (st.execute()) {

      sp.get<logger>()->trace(
        ThisModule,
        sp,
        "Provide log record %s of channel %s",
        base64::from_bytes(p).c_str(),
        t1.channel_id.get(st).c_str());

      result = result.then([
          sp,
          request = p,
          message,
          channel_id = base64::to_bytes(t1.channel_id.get(st)),
          data = t1.data.get(st)]() {
        auto & client = *sp.get<vds::dht::network::client>();
        return client->send(
          sp,
          message.source_node(),
          messages::channel_log_record(
            channel_id,
            request,
            data));
      });
    }
  }

  return result;
}

void vds::dht::network::sync_process::apply_message(const service_provider& sp, database_transaction& t,
  const messages::channel_log_record& message) {

  sp.get<logger>()->trace(
    ThisModule,
    sp,
    "Save log record %s of channel %s",
    base64::from_bytes(message.record_id()).c_str(),
    base64::from_bytes(message.channel_id()).c_str());

  transaction_log::save(
    sp,
    t,
    message.channel_id(),
    message.record_id(),
    message.data());
}

vds::async_task<> vds::dht::network::sync_process::sync_local_channels(const service_provider& sp, database_transaction& t) {
  auto & client = *sp.get<dht::network::client>();
  
  orm::channel_local_cache_dbo t1;
  auto st = t.get_reader(
    t1.select(t1.channel_id)
        .where(t1.last_sync > std::chrono::system_clock::now() - std::chrono::hours(24 * 30)));

  std::map<const_data_buffer, std::list<const_data_buffer>> channels;
  while (st.execute()) {
    channels.emplace(base64::to_bytes(t1.channel_id.get(st)), std::list<const_data_buffer>());
  }

  orm::transaction_log_record_dbo t2;
  st = t.get_reader(
      t2.select(t2.channel_id, t2.id)
          .where(t2.state == (int)orm::transaction_log_record_dbo::state_t::leaf));

  while (st.execute()) {
    channels[base64::to_bytes(t2.channel_id.get(st))].push_back(base64::to_bytes(t2.id.get(st)));
  }
  auto result = async_task<>::empty();
  for(auto p : channels){

    std::string log_message;
    for (const auto & r : p.second) {
      log_message += base64::from_bytes(r);
      log_message += ' ';
    }

    sp.get<logger>()->trace(
        ThisModule,
        sp,
        "Channel %s state is %s",
        base64::from_bytes(p.first).c_str(),
        log_message.c_str());

    result = result.then([client, sp, target_node = p.first, state = p.second]() {
      return client->send(
        sp,
        target_node,
        messages::channel_log_state(
          target_node,
          state,
          client->current_node_id()));
    });
  }
  return result;
}

struct object_info{
  std::map<uint16_t /*replica*/,
      std::map<vds::const_data_buffer /*distance*/,
          std::list<vds::const_data_buffer/*node_id*/>>> replicas;
  std::map<vds::const_data_buffer/*node_id*/, std::list<uint16_t>/*replica*/> nodes;
};

vds::async_task<> vds::dht::network::sync_process::sync_replicas(
    const vds::service_provider &sp,
    vds::database_transaction &t) {
  auto & client = *sp.get<dht::network::client>();

  std::map<const_data_buffer /*object_id*/, object_info> objects;
  orm::chunk_replica_map_dbo t1;
  auto st = t.get_reader(t1.select(t1.id, t1.replica, t1.node));
  while(st.execute()){
    auto object_id = base64::to_bytes(t1.id.get(st));
    auto node_id = base64::to_bytes(t1.node.get(st));
    objects[object_id].replicas[t1.replica.get(st)]
    [dht_object_id::distance(object_id, node_id)].push_back(node_id);
    objects[base64::to_bytes(t1.id.get(st))].nodes[node_id].push_back(t1.replica.get(st));
  }

  //Move replicas closer to root
  auto result = async_task<>::empty();
  for(auto & p : objects){
    std::map<vds::const_data_buffer /*distance*/, std::list<vds::const_data_buffer/*node_id*/>> neighbors;
    client->neighbors(sp, p.first, neighbors, _client::GENERATE_HORCRUX);

    for(auto & pneighbor : neighbors){
      for(auto & pneighbor_node : pneighbor.second) {
        if (p.second.nodes.end() == p.second.nodes.find(pneighbor_node))
          for (auto &preplica : p.second.replicas) {
            for (auto &pnode : preplica.second) {
              if (pneighbor.first < pnode.first) {
                for(auto & node : pnode.second) {
                 result = result.then([
                      client,
                      sp,
                      neighbor_node = pneighbor_node,
                      object_id = p.first,
                      replica = preplica.first,
                        target_node = node](){
                        client->send(sp, neighbor_node, messages::offer_move_replica(
                          object_id,
                          replica,
                          target_node,
                          client->current_node_id()));
                      });
                }
              }
            }
          }
      }
    }
  }

  return result;
}

void vds::dht::network::sync_process::apply_message(
    const vds::service_provider &sp,
    vds::database_transaction &t,
    const vds::dht::messages::offer_move_replica &message) {

  auto & client = *sp.get<dht::network::client>();

  orm::chunk_replica_data_dbo t1;
  auto st = t.get_reader(
      t1
          .select(t1.replica_data)
          .where(t1.id == base64::from_bytes(message.object_id())
          && t1.replica == message.replica()));
  if(!st.execute()){
    client->send(sp, message.source_node(), messages::replica_not_found(
        message.object_id(),
        message.replica(),
        client->current_node_id()));
  }
  else {
    client->send(sp, message.target_node(), messages::offer_replica(
        message.object_id(),
        message.replica(),
        t1.replica_data.get(st),
        message.source_node(),
        client->current_node_id()));
  }
}

void vds::dht::network::sync_process::apply_message(const vds::service_provider &sp, vds::database_transaction &t,
                                                    const vds::dht::messages::replica_not_found &message) {
  orm::chunk_replica_map_dbo t1;
  t.execute(t1.delete_if(
      t1.id == base64::from_bytes(message.object_id())
      && t1.replica == message.replica()
      && t1.node == base64::from_bytes(message.source_node())));

}

void vds::dht::network::sync_process::apply_message(
    const vds::service_provider &sp,
    vds::database_transaction &t,
    const vds::dht::messages::offer_replica &message) {

  orm::chunk_replica_map_dbo t1;
  t.execute(t1.insert_or_ignore(
      t1.id = base64::from_bytes(message.object_id()),
      t1.replica = message.replica(),
      t1.node = base64::from_bytes(message.source_node())));

  orm::chunk_replica_data_dbo t2;
  auto st = t.get_reader(t2.select(t2.replica).where(t2.id == base64::from_bytes(message.object_id())));
  if(!st.execute()){
    t.execute(t2.insert(
        t2.id = base64::from_bytes(message.object_id()),
        t2.replica = message.replica(),
        t2.replica_data = message.replica_data()
    ));

    auto & client = *sp.get<vds::dht::network::client>();
    client->send(sp, message.source_node(), messages::got_replica(
       message.object_id(),
       message.replica(),
       client->current_node_id()));

    std::map<vds::const_data_buffer /*distance*/, std::list<vds::const_data_buffer/*node_id*/>> neighbors;
    client->neighbors(sp, message.object_id(), neighbors, _client::GENERATE_HORCRUX);

    for(auto & p : neighbors){
      for(auto & node : p.second) {
        client->send(sp, node, messages::got_replica(
            message.object_id(),
            message.replica(),
            client->current_node_id()));
      }
    }
  }

}

void vds::dht::network::sync_process::apply_message(
    const vds::service_provider &sp,
    vds::database_transaction &t,
    const vds::dht::messages::got_replica &message) {

  orm::chunk_replica_map_dbo t1;
  t.execute(t1.insert_or_ignore(
      t1.id = base64::from_bytes(message.object_id()),
      t1.replica = message.replica(),
      t1.node = base64::from_bytes(message.source_node())));

}
