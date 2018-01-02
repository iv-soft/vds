/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "server.h"
#include "private/server_p.h"
#include "node_manager.h"
#include "user_manager.h"
#include "server_http_api.h"
#include "private/server_http_api_p.h"
#include "server_connection.h"
#include "server_udp_api.h"
#include "node_manager.h"
#include "private/node_manager_p.h"
#include "private/storage_log_p.h"
#include "private/chunk_manager_p.h"
#include "private/server_database_p.h"
#include "private/local_cache_p.h"
#include "private/node_manager_p.h"
#include "private/server_connection_p.h"
#include "private/server_udp_api_p.h"
#include "server_certificate.h"
#include "private/storage_log_p.h"
#include "transaction_block.h"
#include "transaction_block.h"
#include "transaction_context.h"
#include "chunk_manager.h"
#include "transaction_log.h"
#include "db_model.h"
#include "certificate_dbo.h"
#include "certificate_private_key_dbo.h"
#include "p2p_network_client.h"
#include "run_configuration_dbo.h"
#include "cert_control.h"
#include "p2p_network.h"
#include "private/p2p_network_p.h"
#include "log_sync_service.h"

vds::server::server()
: impl_(new _server(this))
{
}

vds::server::~server()
{
  delete impl_;
}



void vds::server::register_services(service_registrator& registrator)
{
  registrator.add_service<iserver>(this->impl_);
  
  registrator.add_service<istorage_log>(this->impl_->storage_log_.get());

  registrator.add_service<principal_manager>(&(this->impl_->storage_log_->principal_manager_));
  
  registrator.add_service<ichunk_manager>(this->impl_->chunk_manager_.get());
  
  registrator.add_service<iserver_database>(this->impl_->server_database_.get());
  
  registrator.add_service<ilocal_cache>(this->impl_->local_cache_.get());

  registrator.add_service<node_manager>(this->impl_->node_manager_.get());

  registrator.add_service<user_manager>(this->impl_->user_manager_.get());

  registrator.add_service<db_model>(this->impl_->db_model_.get());

  registrator.add_service<ip2p_network_client>(this->impl_->network_client_.get());

  registrator.add_service<p2p_network>(this->impl_->p2p_network_.get());
}

void vds::server::start(const service_provider& sp)
{
  this->impl_->start(sp);
}

void vds::server::stop(const service_provider& sp)
{
  this->impl_->stop(sp);
}

vds::async_task<> vds::server::reset(const vds::service_provider &sp, const std::string &root_user_name, const std::string &root_password,
                                     const std::string &device_name, int port) {

  return sp.get<db_model>()->async_transaction(sp, [this, sp, root_user_name, root_password, device_name, port](
      database_transaction & t){
    auto usr_manager = sp.get<user_manager>();
    auto private_key = asymmetric_private_key::generate(asymmetric_crypto::rsa4096());
    auto block_data = usr_manager->reset(sp, t, root_user_name, root_password, private_key, device_name, port);
    auto block_id = chunk_manager::pack_block(t, block_data);
	  transaction_log::apply(sp, t, chunk_manager::get_block(t, block_id));
  });
}

vds::async_task<> vds::server::init_server(const vds::service_provider &sp, const std::string &user_login,
                                           const std::string &user_password, const std::string &device_name, int port) {
  return this->impl_->p2p_network_->init_server(sp, user_login, user_password, device_name, port);

}

vds::async_task<> vds::server::start_network(const vds::service_provider &sp) {
  return this->impl_->p2p_network_->start_network(sp);
}

vds::async_task<> vds::server::prepare_to_stop(const vds::service_provider &sp) {
  return this->impl_->prepare_to_stop(sp);
}

void vds::server::get_statistic(vds::server_statistic &result) {
  this->impl_->get_statistic(result);
}

void vds::transaction_log::apply(
    const service_provider &sp,
    database_transaction &t,
    const const_data_buffer &chunk) {
  auto scope = sp.create_scope("Apply record");

  auto data = transaction_block::unpack_block(
      scope,
      chunk,
    [&t](const guid & cert_id) -> certificate{
	  certificate_dbo t1;
	  auto st = t.get_reader(t1.select(t1.cert).where(t1.id == cert_id));
	  if (st.execute()) {
		  return certificate::parse_der(t1.cert.get(st));
	  }
	  else {
		  return certificate();
	  }
    },
    [&t](const guid & cert_id) -> asymmetric_private_key{
		certificate_private_key_dbo t1;
		auto st = t.get_reader(t1.select(t1.body).where(t1.id == cert_id));
		if (st.execute()) {
			return asymmetric_private_key::parse_der(t1.body.get(st), std::string());
		}
		else {
			return asymmetric_private_key();
		}
	});

  binary_deserializer s(data);

  while(0 < s.size()){
    uint8_t category_id;
    uint8_t message_id;
    s >> category_id >> message_id;
    switch (category_id){
      case transaction_log::user_manager_category_id:
      {
        scope.get<user_manager>()->apply_transaction_record(scope, t, message_id, s);
        break;
      }
      default:
        throw std::runtime_error("Invalid record category");
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////

vds::_server::_server(server * owner)
: owner_(owner),
  node_manager_(new _node_manager()),
  server_http_api_(new _server_http_api()),
  storage_log_(new _storage_log()),
  chunk_manager_(new _chunk_manager()),
  server_database_(new _server_database()),
  local_cache_(new _local_cache()),
	user_manager_(new user_manager()),
	db_model_(new db_model()),
  network_client_(new p2p_network_client()),
  p2p_network_(new p2p_network()),
  log_sync_service_(new log_sync_service())
{
  this->leak_detect_.name_ = "server";
  this->leak_detect_.dump_callback_ = [this](leak_detect_collector * collector){
    collector->add(*this->p2p_network_);
    //collector->add(this->network_client_);
  };
}

vds::_server::~_server()
{
}

void vds::_server::start(const service_provider& sp)
{
	this->db_model_->start(sp);
  this->log_sync_service_->start(sp);
}

void vds::_server::stop(const service_provider& sp)
{
  this->log_sync_service_->stop(sp);
  this->db_model_->stop(sp);
  this->p2p_network_->stop(sp);

  this->log_sync_service_.reset();
  this->db_model_.reset();
  this->p2p_network_.reset();
  this->network_client_.reset();
}

vds::async_task<> vds::_server::prepare_to_stop(const vds::service_provider &sp) {
  return async_series(
    this->log_sync_service_->prepare_to_stop(sp),
    this->db_model_->prepare_to_stop(sp),
    this->p2p_network_->prepare_to_stop(sp)
  );
}

void vds::_server::get_statistic(vds::server_statistic &result) {
  this->log_sync_service_->get_statistic(result.sync_statistic_);

}
