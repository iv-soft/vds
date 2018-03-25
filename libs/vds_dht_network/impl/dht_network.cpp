/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "dht_network.h"
#include "private/udp_transport.h"
#include "db_model.h"
#include "current_config_dbo.h"

void vds::dht::network::service::register_services(service_registrator& registrator) {
  registrator.add_service<client>(&this->client_);
}

void vds::dht::network::service::start(const service_provider& sp, uint16_t port) {
  barrier b;
  std::shared_ptr<std::exception> error;
  sp.get<db_model>()->async_transaction(sp, [sp, port, this](database_transaction & t){

    certificate node_cert;

    orm::current_config_dbo t1;
    auto st = t.get_reader(t1.select(t1.cert, t1.cert_key));
    if(st.execute()){
      node_cert = certificate::parse_der(t1.cert.get(st));
    }
    else {
      auto private_key = asymmetric_private_key::generate(asymmetric_crypto::rsa4096());
      asymmetric_public_key public_key(private_key);

      certificate::create_options options;
      options.name = "Node Cert";
      options.country = "RU";
      options.organization = "IVySoft";
      node_cert = certificate::create_new(public_key, private_key, options);
      t.execute(t1.insert(
          t1.cert = node_cert.der(),
          t1.cert_key = private_key.der(std::string())));
    }
    this->udp_transport_ = std::make_shared<udp_transport>();
    this->udp_transport_->start(sp, port, node_cert.fingerprint(hash::sha256()));

    this->client_.start(sp, node_cert.fingerprint(hash::sha256()));

    return true;
  }).execute([&b, &error](const std::shared_ptr<std::exception> & ex){
    if(ex){
      error = ex;
    }
    b.set();
  });

  b.wait();
  if(error){
    throw std::runtime_error(error->what());
  }

}

void vds::dht::network::service::stop(const service_provider& sp) {
  this->client_.stop(sp);
  this->udp_transport_->stop(sp);
}

vds::async_task<> vds::dht::network::service::prepare_to_stop(const service_provider& sp) {
  return vds::async_task<>::empty();
}
