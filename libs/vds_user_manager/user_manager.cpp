/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include <certificate_unknown_dbo.h>
#include "stdafx.h"
#include "user_manager.h"
#include "member_user.h"
#include "private/user_manager_p.h"
#include "private/member_user_p.h"
#include "database_orm.h"
#include "private/cert_control_p.h"
#include "vds_exceptions.h"
#include "channel_create_transaction.h"
#include "channel_add_reader_transaction.h"
#include "channel_add_writer_transaction.h"
#include "db_model.h"
#include "dht_object_id.h"
#include "dht_network_client.h"
#include "register_request.h"
#include "create_user_transaction.h"
#include "private/user_channel_p.h"
#include "control_message_transaction.h"
#include "create_user_transaction.h"
#include "user_storage.h"

vds::user_manager::user_manager(const service_provider * sp)
: sp_(sp) {
}

vds::user_manager::~user_manager() {
}

vds::user_manager::login_state_t vds::user_manager::get_login_state() const {
  return this->impl_->get_login_state();
}

vds::async_task<vds::expected<void>> vds::user_manager::update() {
  return this->sp_->get<db_model>()->async_read_transaction([pthis = this->shared_from_this()](database_read_transaction & t) -> expected<void> {
    return pthis->update(t);
  });
}

vds::expected<void> vds::user_manager::update(database_read_transaction & t) const {
  return this->impl_->update(t);
}

vds::expected<void> vds::user_manager::load(
  database_read_transaction & t,
  const std::string & user_login,
  const std::string & user_password)
{
	if (nullptr != this->impl_.get()) {
		return vds::make_unexpected<std::runtime_error>("Logic error");
	}

  this->impl_.reset(new _user_manager());
  CHECK_EXPECTED(this->impl_->create(
    this->sp_,
    user_login,
    user_password));

	return this->impl_->update(t);
}

vds::async_task<vds::expected<vds::user_channel>> vds::user_manager::create_channel(
  const std::string & channel_type,
  const std::string& name) const {
  return this->impl_->create_channel(
    channel_type,
    name);
}


vds::expected<void> vds::user_manager::reset(
    const std::string &root_user_name,
    const std::string &root_password,
    const cert_control::private_info_t & private_info) {
  return this->sp_->get<db_model>()->async_transaction([this, root_user_name, root_password, private_info](
    database_transaction & t) -> expected<void> {

    auto playback = transactions::transaction_block_builder::create_root_block(this->sp_);

    //Create root user
    GET_EXPECTED(root_private_key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));

    GET_EXPECTED(root_user, _member_user::create_user(
      playback,
      "root",
      root_user_name,
      root_password,
      std::make_shared<asymmetric_private_key>(std::move(root_private_key))));

    //common news
    this->sp_->get<logger>()->info(ThisModule, "Create channel %s(Common News)",
      base64::from_bytes(cert_control::get_common_news_channel_id()).c_str());

    GET_EXPECTED(pc, root_user->personal_channel());
    CHECK_EXPECTED(pc.add_log(
      playback,
      message_create<transactions::channel_create_transaction>(
        cert_control::get_common_news_channel_id(),
        user_channel::channel_type_t::news_channel,
        "Common news",
        cert_control::get_common_news_read_certificate(),
        cert_control::get_common_news_read_private_key(),
        cert_control::get_common_news_write_certificate(),
        private_info.common_news_write_private_key_)));

    //Auto update
    this->sp_->get<logger>()->info(ThisModule, "Create channel %s(Auto update)",
      base64::from_bytes(cert_control::get_autoupdate_channel_id()).c_str());

    CHECK_EXPECTED(pc.add_log(
      playback,
      message_create<transactions::channel_create_transaction>(
        cert_control::get_autoupdate_channel_id(),
        user_channel::channel_type_t::file_channel,
        "Auto update",
        cert_control::get_autoupdate_read_certificate(),
        cert_control::get_autoupdate_read_private_key(),
        cert_control::get_autoupdate_write_certificate(),
        private_info.autoupdate_write_private_key_)));

    //Create auto update user
    GET_EXPECTED(autoupdate_private_key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));

    GET_EXPECTED(
      auto_update_user,
      root_user->create_user(
        playback,
        "Auto Update",
        cert_control::auto_update_login(),
        cert_control::auto_update_password(),
        std::make_shared<asymmetric_private_key>(std::move(autoupdate_private_key))));

    GET_EXPECTED(auto_update_user_personal_channel, auto_update_user->personal_channel());
    CHECK_EXPECTED(auto_update_user_personal_channel.add_log(
      playback,
      message_create<transactions::channel_add_reader_transaction>(
        cert_control::get_autoupdate_channel_id(),
        user_channel::channel_type_t::file_channel,
        "Auto update",
        cert_control::get_autoupdate_read_certificate(),
        cert_control::get_autoupdate_read_private_key(),
        cert_control::get_autoupdate_write_certificate())));

    //Web
    this->sp_->get<logger>()->info(ThisModule, "Create channel %s(Web)",
      base64::from_bytes(cert_control::get_web_channel_id()).c_str());

    CHECK_EXPECTED(pc.add_log(
      playback,
      message_create<transactions::channel_create_transaction>(
        cert_control::get_web_channel_id(),
        user_channel::channel_type_t::file_channel,
        "Web",
        cert_control::get_web_read_certificate(),
        cert_control::get_web_read_private_key(),
        cert_control::get_web_write_certificate(),
        private_info.web_write_private_key_)));

    //Create web user
    GET_EXPECTED(web_private_key, asymmetric_private_key::generate(asymmetric_crypto::rsa4096()));
    GET_EXPECTED(
      web_user,
      root_user->create_user(
        playback,
        "Web",
        cert_control::web_login(),
        cert_control::web_password(),
        std::make_shared<asymmetric_private_key>(std::move(web_private_key))));

    GET_EXPECTED(web_user_personal_channel, web_user->personal_channel());
    CHECK_EXPECTED(web_user_personal_channel.add_log(
      playback,
      message_create<transactions::channel_add_reader_transaction>(
        cert_control::get_web_channel_id(),
        user_channel::channel_type_t::file_channel,
        "Web",
        cert_control::get_web_read_certificate(),
        cert_control::get_web_read_private_key(),
        cert_control::get_web_write_certificate())));

    CHECK_EXPECTED(this->sp_->get<dht::network::client>()->save(
      this->sp_,
      playback,
      t));

    return expected<void>();
  }).get();
}

std::shared_ptr<vds::user_channel> vds::user_manager::get_channel(
  const const_data_buffer & channel_id) const
{
  return this->impl_->get_channel(channel_id);
}

std::map<vds::const_data_buffer, std::shared_ptr<vds::user_channel>> vds::user_manager::get_channels() const {
  std::list<vds::user_channel> result;

  return this->impl_->channels();
}

vds::expected<uint64_t> vds::user_manager::get_device_storage_used() {
  GET_EXPECTED(result, user_storage::device_storages(
    this->sp_,
    this->shared_from_this()).get());

  for (const auto & device : result) {
    if (device.current) {
      return device.used_size;
    }
  }  

  return 0;
}

vds::expected<uint64_t> vds::user_manager::get_device_storage_size() {
  GET_EXPECTED(result, user_storage::device_storages(
    this->sp_,
    this->shared_from_this()).get());

  for (const auto & device : result) {
    if (device.current) {
      return device.reserved_size;
    }
  }

  return 0;
}

vds::expected<uint64_t> vds::user_manager::get_user_balance() {
  return 0;
}

//vds::expected<bool> vds::user_manager::validate_and_save(
//		
//		const std::list<std::shared_ptr<vds::certificate>> &cert_chain) {
//
//  certificate_store store;
//  for (const auto & p : cert_chain) {
//    auto cert = this->impl_->get_certificate(p->subject());
//    if (!cert) {
//      cert = p;
//
//      GET_EXPECTED(result, store.verify(*cert));
//      if (0 != result.error_code) {
//        this->sp_->get<logger>()->warning(ThisModule, "Invalid certificate %s %s",
//          result.error.c_str(),
//          result.issuer.c_str());
//        return false;
//      }
//    }
//
//    CHECK_EXPECTED(store.add(*cert));
//    CHECK_EXPECTED(this->save_certificate(cert));
//  }
//
//  return true;
//}
//
//vds::expected<void> vds::user_manager::save_certificate(
//    vds::database_transaction &t,
//    const vds::asymmetric_public_key &cert) {
//
//  orm::certificate_chain_dbo t1;
//  GET_EXPECTED(st, t.get_reader(t1.select(t1.id).where(t1.id == cert.subject())));
//  GET_EXPECTED(st_result, st.execute());
//  if (!st_result) {
//    GET_EXPECTED(der, cert.der());
//    CHECK_EXPECTED(t.execute(t1.insert(
//      t1.id = cert.subject(),
//      t1.cert = der,
//      t1.parent = cert.issuer())));
//  }
//
//  orm::certificate_unknown_dbo t2;
//  return t.execute(t2.delete_if(t2.id == cert.subject()));
//}

vds::member_user vds::user_manager::get_current_user() const {
  return this->impl_->get_current_user();
}

const std::shared_ptr<vds::asymmetric_private_key> & vds::user_manager::get_current_user_private_key() const {
  return this->impl_->get_current_user_private_key();
}

vds::async_task<vds::expected<void>> vds::user_manager::create_user(
  const service_provider * sp,
  const std::string& userName,
  const std::string& userEmail,
  const std::string& userPassword) {

  return sp->get<db_model>()->async_transaction(
    [
      sp,
      userName,
      userEmail,
      userPassword
    ](database_transaction & t)->expected<void> {

    GET_EXPECTED(user_id, dht::dht_object_id::user_credentials_to_key(userEmail, userPassword));
    GET_EXPECTED(user_private_key, vds::asymmetric_private_key::generate(
      vds::asymmetric_crypto::rsa4096()));
    GET_EXPECTED(user_private_key_der, user_private_key.der(userPassword));

    GET_EXPECTED(user_public_key, asymmetric_public_key::create(user_private_key));

    GET_EXPECTED(playback, transactions::transaction_block_builder::create(sp, t));

    CHECK_EXPECTED(playback.add(
      message_create<transactions::create_user_transaction>(
        user_id,
        std::make_shared<asymmetric_public_key>(std::move(user_public_key)),
        user_private_key_der,
        userName)));

    //auto channel_id = dht::dht_object_id::generate_random_id();

    //auto read_private_key = asymmetric_private_key::generate(asymmetric_crypto::rsa4096());
    //auto write_private_key = asymmetric_private_key::generate(asymmetric_crypto::rsa4096());
    //auto channel = member_user(pthis->user_cert_, pthis->user_private_key_).create_channel(
    //  playback,
    //  userName);

    //channel->add_writer(
    //  playback,
    //  pthis->user_name_,
    //  member_user(user_cert, std::shared_ptr<asymmetric_private_key>()),
    //  member_user(pthis->user_cert_, pthis->user_private_key_));
    auto client = sp->get<dht::network::client>();
    CHECK_EXPECTED(client->save(sp, playback, t));

    return expected<void>();
  });
}

const std::list<std::shared_ptr<vds::user_wallet>>& vds::user_manager::wallets() const
{
  return this->impl_->wallets();
}

/////////////////////////////////////////////////////////////////////
vds::expected<void> vds::_user_manager::create(
  const service_provider * sp,
		const std::string & user_login,
    const std::string & user_password) {
  this->sp_ = sp;
  this->login_state_ = user_manager::login_state_t::waiting;
  GET_EXPECTED_VALUE(this->user_credentials_key_, dht::dht_object_id::user_credentials_to_key(user_login, user_password));
  this->user_password_ = user_password;
  return expected<void>();
}

vds::expected<bool> vds::_user_manager::process_create_user_transaction(
  const transactions::create_user_transaction & message) {
  if (this->user_credentials_key_ == message.user_credentials_key) {
    this->user_cert_ = message.user_cert;
    this->user_name_ = message.user_name;

    GET_EXPECTED(user_private_key, asymmetric_private_key::parse_der(message.user_private_key, this->user_password_));
    this->user_private_key_ = std::make_shared<asymmetric_private_key>(std::move(user_private_key));

    this->login_state_ = user_manager::login_state_t::login_successful;

    GET_EXPECTED(cp, _user_channel::import_personal_channel(
      this->user_cert_,
      this->user_private_key_));
    this->channels_[cp->id()] = cp;
  }
  return true;
}

vds::expected<bool> vds::_user_manager::process_channel_message(
  const transactions::channel_message & message,
  std::set<const_data_buffer> & new_channels,
  std::chrono::system_clock::time_point tp) {
  const auto log = this->sp_->get<logger>();
  auto channel = this->get_channel(message.channel_id());
  if (channel) {
    auto channel_read_key = channel->read_cert_private_key(message.channel_read_cert_subject());
    if (channel_read_key) {
      CHECK_EXPECTED(message.walk_messages(this->sp_, *channel_read_key, transactions::message_environment_t{ tp, "???" },
        [this, channel_id = message.channel_id(), log](
          const transactions::channel_add_reader_transaction & message,
          const transactions::message_environment_t & /*message_environment*/)->expected<bool> {
        GET_EXPECTED(read_id, message.read_cert->hash(hash::sha256()));
        GET_EXPECTED(write_id, message.write_cert->hash(hash::sha256()));
        auto cp = std::make_shared<user_channel>(
          message.id,
          message.channel_type,
          message.name,
          read_id,
          message.read_cert,
          message.read_private_key,
          write_id,
          message.write_cert,
          std::shared_ptr<asymmetric_private_key>());

        this->channels_[cp->id()] = cp;
        log->debug(ThisModule, "Got channel %s reader public key",
          base64::from_bytes(cp->id()).c_str());

        return true;
      },
        [this, channel_id = message.channel_id(), log](
          const transactions::channel_add_writer_transaction & message,
          const transactions::message_environment_t & /*message_environment*/)->expected<bool> {
        GET_EXPECTED(read_id, message.read_cert->hash(hash::sha256()));
        GET_EXPECTED(write_id, message.write_cert->hash(hash::sha256()));
        auto cp = std::make_shared<user_channel>(
          message.id,
          message.channel_type,
          message.name,
          read_id,
          message.read_cert,
          message.read_private_key,
          write_id,
          message.write_cert,
          message.write_private_key);

        this->channels_[cp->id()] = cp;

        log->debug(ThisModule, "Got channel %s write public key",
          base64::from_bytes(cp->id()).c_str());

        return true;
      },
        [this, channel_id = message.channel_id(), log, &new_channels](
          const transactions::channel_create_transaction & message,
          const transactions::message_environment_t & /*message_environment*/)->expected<bool>{
        if (new_channels.end() == new_channels.find(channel_id)) {
          new_channels.emplace(message.channel_id);
        }
        GET_EXPECTED(read_id, message.read_cert->hash(hash::sha256()));
        GET_EXPECTED(write_id, message.write_cert->hash(hash::sha256()));
        auto cp = std::make_shared<user_channel>(
          message.channel_id,
          message.channel_type,
          message.name,
          read_id,
          message.read_cert,
          message.read_private_key,
          write_id,
          message.write_cert,
          message.write_private_key);

        this->channels_[cp->id()] = cp;

        return true;
      },
        [this, channel_id = message.channel_id(), log](
          const transactions::control_message_transaction & message,
          const transactions::message_environment_t & /*message_environment*/)->expected<bool> {
        auto msg = std::dynamic_pointer_cast<json_object>(message.message);
        std::string type;
        if (msg) {
          GET_EXPECTED(have_type, msg->get_property("$type", type));
          if (have_type) {
            if (transactions::control_message_transaction::create_wallet_type == type) {
              std::string name;
              CHECK_EXPECTED(msg->get_property("name", name));

              GET_EXPECTED(cert, asymmetric_public_key::parse_der(message.attachments.at("cert")));
              GET_EXPECTED(private_key, asymmetric_private_key::parse_der(message.attachments.at("key"), std::string()));
              auto wallet = std::make_shared<user_wallet>(
                name,
                std::move(cert),
                std::move(private_key));

              this->wallets_.push_back(wallet);

              log->debug(ThisModule, "Got wallet %s write public key",
                name.c_str());
            }
          }
        }

        return true;
      }));
    }
  }

  return true;
}

vds::expected<void> vds::_user_manager::update(
  database_read_transaction &t) {
  std::list<const_data_buffer> new_records;
  orm::transaction_log_record_dbo t1;
  GET_EXPECTED(st, t.get_reader(
    t1.select(t1.id)
    .order_by(t1.order_no)));
  WHILE_EXPECTED(st.execute())
    auto id = t1.id.get(st);
  if (this->processed_.end() != this->processed_.find(id)) {
    continue;
  }

  new_records.push_back(id);
  WHILE_EXPECTED_END()

    if (new_records.empty() && this->login_state_ == user_manager::login_state_t::waiting) {
      this->login_state_ = user_manager::login_state_t::login_failed;
    }

  std::set<const_data_buffer> new_channels;
  for (auto & id : new_records) {
    GET_EXPECTED_VALUE(st, t.get_reader(
      t1.select(t1.data)
      .where(t1.id == id)));

    GET_EXPECTED(st_result, st.execute());
    if (!st_result) {
      return vds::make_unexpected<std::runtime_error>("Invalid program");
    }

    const auto data = t1.data.get(st);
    GET_EXPECTED(block, transactions::transaction_block::create(data));

    CHECK_EXPECTED(block.walk_messages(
      [this](const transactions::create_user_transaction & message)->expected<bool> {
        return this->process_create_user_transaction(message);
      },
      [this, &new_channels, tp = block.time_point()](const transactions::channel_message  & message)->expected<bool>{
      return this->process_channel_message(message, new_channels, tp);
    }
    ));
  }
  
  return expected<void>();
}

vds::expected<void> vds::_user_manager::add_certificate(const std::shared_ptr<vds::asymmetric_public_key> &cert) {
  GET_EXPECTED(id, cert->hash(hash::sha256()));
	this->certificate_chain_[id] = cert;
  return expected<void>();
}

vds::member_user vds::_user_manager::get_current_user() const {
  return member_user(this->user_cert_, this->user_private_key_);
}

const std::string& vds::_user_manager::user_name() const {
  return this->user_name_;
}

vds::async_task<vds::expected<vds::user_channel>> vds::_user_manager::create_channel(
  const std::string & channel_type,
  const std::string& name) {
  auto result = std::make_shared<vds::user_channel>();
  CHECK_EXPECTED_ASYNC(co_await this->sp_->get<db_model>()->async_transaction(
    [pthis = this->shared_from_this(), channel_type, name, result](database_transaction & t)->expected<void> {

    GET_EXPECTED(log, vds::transactions::transaction_block_builder::create(pthis->sp_, t));

    vds::asymmetric_private_key channel_read_private_key;
    vds::asymmetric_private_key channel_write_private_key;
    GET_EXPECTED_VALUE(*result, member_user(pthis->user_cert_, pthis->user_private_key_).create_channel(
      log,
      channel_type,
      name));

    CHECK_EXPECTED(
      pthis->sp_->get<dht::network::client>()->save(
        pthis->sp_,
        log,
        t));

    CHECK_EXPECTED(pthis->update(t));

    return expected<void>();
  }));

  co_return  std::move(*result);
}

const std::string& vds::user_manager::user_name() const {
  return this->impl_->user_name();
}
