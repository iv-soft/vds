/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "server_to_server_api.h"
#include "messages.h"
#include "server_log_sync_p.h"
#include "route_manager_p.h"
#include "connection_manager_p.h"
#include "object_transfer_protocol_p.h"
#include "storage_log.h"

void vds::server_to_server_api::process_message(
  const service_provider & sp,
  database_transaction & t,
  _connection_manager * con_man,
  const connection_session & session,
  uint32_t message_type_id,
  const const_data_buffer & binary_form)
{
  switch ((message_identification)message_type_id) {
  case message_identification::server_log_record_broadcast_message_id:
    sp.get<_server_log_sync>()->on_record_broadcast(
      sp,
      t,
      _server_log_sync::server_log_record_broadcast(sp, binary_form));
    break;

  case message_identification::server_log_get_records_broadcast_message_id:
    sp.get<_server_log_sync>()->on_server_log_get_records_broadcast(
      sp,
      t,
      session,
      _server_log_sync::server_log_get_records_broadcast(binary_form));
    break;

  case message_identification::object_request_message_id:
    sp.get<logger>()->debug("route", sp, "object_request_message_id");
    con_man->object_transfer_protocol_->on_object_request(
      sp,
      t,
      session,
      object_request(binary_form));
    break;
    
  case message_identification::object_offer_replicas_message_id:
  {
    con_man->object_transfer_protocol_->object_offer(
      sp,
      t,
      session,
      object_offer_replicas(binary_form));
    break;
  }

  case message_identification::route_message_message_id:
  {
    route_message msg(binary_form);
    sp.get<logger>()->debug("route", sp, "Route: route message to %s, this server is %s",
      msg.target_server_id().str().c_str(),
      sp.get<istorage_log>()->current_server_id().str().c_str());
    con_man->route_manager_->on_route_message(
      con_man,
      sp,
      t,
      session,
      msg);
    
    if(message_identification::object_offer_replicas_message_id == (message_identification)msg.msg_type_id()){
      this->process_message(
        sp,
        t,
        con_man,
        session,
        msg.msg_type_id(),
        msg.message_data());
    }

    break;
  }
  default:
    sp.get<logger>()->debug("route", sp, "Handler for message %d not found", message_type_id);
    break;
  }

}