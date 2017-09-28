/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "client.h"
#include "client_p.h"
#include "client_connection.h"
#include "deflate.h"
#include "file_manager_p.h"
#include "certificate_authority.h"

vds::client::client(const std::string & server_address)
: server_address_(server_address),
  impl_(new _client(this)),
  logic_(new client_logic())
{
}

vds::client::~client()
{
}

void vds::client::register_services(service_registrator & registrator)
{
  registrator.add_service<iclient>(this->impl_.get());
}

void vds::client::start(const service_provider & sp)
{
  this->impl_->start(sp);
  //Load certificates
  certificate * client_certificate = nullptr;
  asymmetric_private_key * client_private_key = nullptr;

  filename cert_name(foldername(persistence::current_user(sp), ".vds"), "client.crt");
  filename pkey_name(foldername(persistence::current_user(sp), ".vds"), "client.pkey");

  if (file::exists(cert_name)) {
    this->client_certificate_.load(cert_name);
    this->client_private_key_.load(pkey_name);

    client_certificate = &this->client_certificate_;
    client_private_key = &this->client_private_key_;
  }

  this->logic_->start(sp, this->server_address_, client_certificate, client_private_key);

}

void vds::client::stop(const service_provider & sp)
{
  this->logic_->stop(sp);
  this->impl_->stop(sp);
}

void vds::client::connection_closed()
{
}

void vds::client::connection_error()
{
}

vds::async_task<
  const vds::certificate & /*server_certificate*/,
  const vds::asymmetric_private_key & /*private_key*/>
  vds::iclient::init_server(
  const service_provider & sp,
  const std::string & user_login,
  const std::string & user_password)
{
  return static_cast<_client *>(this)->init_server(sp, user_login, user_password);
}

vds::async_task<> vds::iclient::create_local_login(
  const service_provider & sp,
  const std::string & login,
  const std::string & password,
  const std::string & name)
{
  return static_cast<_client *>(this)->create_local_login(sp, login, password, name);
}

vds::async_task<const std::string& /*version_id*/> vds::iclient::upload_file(
  const service_provider & sp,
  const std::string & name,
  const filename & tmp_file)
{
  return static_cast<_client *>(this)->upload_file(sp, name, tmp_file);
}

vds::async_task<const vds::guid & /*version_id*/> vds::iclient::download_data(
  const service_provider & sp,
  const std::string & name,
  const filename & target_file)
{
  return static_cast<_client *>(this)->download_data(sp, name, target_file);
}

vds::_client::_client(vds::client* owner)
: owner_(owner), last_tmp_file_index_(0)
{
}

void vds::_client::start(const service_provider & sp)
{
  this->tmp_folder_ = foldername(foldername(persistence::current_user(sp), ".vds"), "tmp");
  this->tmp_folder_.create();
}

void vds::_client::stop(const service_provider & sp)
{
}


vds::async_task<
  const vds::certificate & /*server_certificate*/,
  const vds::asymmetric_private_key & /*private_key*/>
vds::_client::init_server(
  const service_provider & sp,
  const std::string& user_login,
  const std::string& user_password)
{
  return
    this->authenticate(sp, user_login, user_password)
    .then([this, user_password](
      const service_provider & sp,
      const client_messages::certificate_and_key_response & response) {
      
    sp.get<logger>()->trace("client", sp, "Register new server");
    
    return asymmetric_private_key::parse_der(
      sp,
      base64::to_bytes(response.private_key_body()),
      user_password)
    .then(
      [this, response, user_password](
      const service_provider & sp,
      const asymmetric_private_key & user_private_key) {

    asymmetric_private_key private_key(asymmetric_crypto::rsa4096());
    private_key.generate();

    asymmetric_public_key pkey(private_key);

    auto user_certificate = certificate::parse(response.certificate_body());

    auto server_id = guid::new_guid();
    certificate::create_options options;
    options.country = "RU";
    options.organization = "IVySoft";
    options.name = "Certificate " + server_id.str();
    options.ca_certificate = &user_certificate;
    options.ca_certificate_private_key = &user_private_key;

    certificate server_certificate = certificate::create_new(pkey, private_key, options);
    foldername root_folder(persistence::current_user(sp), ".vds");
    root_folder.create();
    
    user_certificate.save(filename(root_folder, "owner.crt"));

    hash ph(hash::sha256());
    ph.update(user_password.c_str(), user_password.length());
    ph.final();

    return this->owner_->logic_->send_request<client_messages::register_server_response>(
      sp,
      client_messages::register_server_request(
        server_id,
        response.id(),
        server_certificate.str(),
        base64::from_bytes(private_key.der(sp, user_password)),
        ph.signature()).serialize())
      .then([this, server_certificate, private_key](
        const std::function<void(
          const service_provider & sp,
          const vds::certificate & server_certificate,
          const vds::asymmetric_private_key & private_key)> & done,
        const error_handler & on_error,
        const service_provider & sp,
        const client_messages::register_server_response & response) {
        
          done(sp, server_certificate, private_key);
        
        });
    });
  });
}

vds::async_task<> vds::_client::create_local_login(
  const service_provider & sp,
  const std::string& user_login,
  const std::string& user_password,
  const std::string & name)
{
  return
    this->authenticate(sp, user_login, user_password)
    .then([this, name, user_password](
      const service_provider & sp,
      const client_messages::certificate_and_key_response & response) ->async_task<> {
    sp.get<logger>()->trace("client", sp, "Register new user");

    return asymmetric_private_key::parse_der(
      sp,
      base64::to_bytes(response.private_key_body()),
      user_password)
      .then(
        [this, response, user_password, name](
          const service_provider & sp,
          const asymmetric_private_key & user_private_key) ->async_task<> {

      auto user_certificate = certificate::parse(response.certificate_body());

      asymmetric_private_key local_user_private_key(asymmetric_crypto::rsa4096());
      local_user_private_key.generate();

      auto member_id = guid::new_guid();

      auto local_user_certificate = certificate_authority::create_local_user(
        member_id,
        user_certificate,
        user_private_key,
        local_user_private_key);

      foldername root_folder(persistence::current_user(sp), ".vds");
      root_folder.create();
      user_certificate.save(filename(root_folder, "owner.crt"));
      local_user_certificate.save(filename(root_folder, "user.crt"));
      local_user_private_key.save(filename(root_folder, "user.pkey"));

      //Form principal_log message
      auto msg = principal_log_record(
        guid::new_guid(),
        response.id(),
        response.id(),
        response.parents(),
        principal_log_new_local_user(
          member_id,
          response.id(),
          local_user_certificate,
          name).serialize(true),
        response.order_num() + 1).serialize(false);

      auto s = msg->str();
      const_data_buffer signature;
      return asymmetric_private_key::parse_der(sp, base64::to_bytes(response.private_key_body()), user_password)
        .then([this, &signature, &response, msg, member_id, local_user_certificate, local_user_private_key, user_password, s](
          const service_provider & sp,
          const asymmetric_private_key & private_key) -> async_task<> {
        return dataflow(
          dataflow_arguments<uint8_t>((const uint8_t *)s.c_str(), s.length()),
          asymmetric_sign(
            hash::sha256(),
            private_key,
            signature))
          .then(
            [this, &signature, &response, msg, member_id, local_user_certificate, local_user_private_key, user_password, s](
              const service_provider & sp) -> async_task<> {
          return this->owner_->logic_->send_request<client_messages::principal_log_add_record_response>(
            sp,
            client_messages::principal_log_add_record_request(
              s,
              signature).serialize(true))
            .then([this](
              const std::function<void(const service_provider & sp)> & done,
              const error_handler & on_error,
              const service_provider & sp,
              const client_messages::principal_log_add_record_response & response) {
            done(sp);
          });
        });
      });
    });
  });
}

vds::async_task<const std::string& /*version_id*/> vds::_client::upload_file(
  const service_provider & sp,
  const std::string & name,
  const filename & fn)
{
  return this->owner_->logic_->send_request<client_messages::server_log_state_response>(
    sp,
    client_messages::server_log_state_request().serialize())
    .then([this, name, fn](
      const service_provider & sp,
      const client_messages::server_log_state_response & response) {

    imt_service::disable_async(sp);

    sp.get<logger>()->trace("client", sp, "Crypting data");
    auto length = file::length(fn);

    //Generate key
    symmetric_key transaction_key(symmetric_crypto::aes_256_cbc());
    transaction_key.generate();

    //Crypt body
    auto version_id = guid::new_guid();
    filename tmp_file(this->tmp_folder_, version_id.str());

    return _file_manager::crypt_file(
      sp,
      length,
      transaction_key,
      fn,
      tmp_file)
      .then([this, name, length, version_id, transaction_key, tmp_file](
        const service_provider & sp,
        size_t body_size,
        size_t tail_size) {
      //Meta info
      binary_serializer file_info;
      file_info << name << length << body_size << tail_size;
      transaction_key.serialize(file_info);

      auto user_certificate = load_user_certificate(sp);

      auto meta_info = user_certificate.public_key().encrypt(file_info.data());

      std::list<principal_log_record::record_id> parents;
      parents.push_back(version_id);
      //Form principal_log message
      auto msg = principal_log_record(
        guid::new_guid(),
        certificate_authority::certificate_parent_id(user_certificate),
        certificate_authority::certificate_id(user_certificate),
        parents,
        principal_log_new_object(version_id, body_size + tail_size, meta_info).serialize(),
        response.order_num() + 1).serialize(false);

      auto s = msg->str();
      const_data_buffer signature;
      return asymmetric_private_key::parse_der(sp, base64::to_bytes(response.private_key_body()), user_password)
        .then([this, &signature, &response, msg, version_id, tmp_file, s](
          const service_provider & sp,
          const asymmetric_private_key & private_key) {

        return dataflow(
          dataflow_arguments<uint8_t>((const uint8_t *)s.c_str(), s.length()),
          asymmetric_sign(
            hash::sha256(),
            private_key,
            signature)
        )
          .then(
            [this, &signature, &response, msg, version_id, tmp_file](const service_provider & sp) {
          sp.get<logger>()->trace("client", sp, "Message [%s] signed [%s]", msg->str().c_str(), base64::from_bytes(signature).c_str());

          uint8_t file_buffer[1024];
          file f(tmp_file, file::file_mode::open_read);
          hash file_hash(hash::sha256());
          for (;;) {
            auto readed = f.read(file_buffer, sizeof(file_buffer));
            if (0 == readed) {
              break;
            }

            file_hash.update(file_buffer, readed);
          }
          file_hash.final();
          f.close();

          sp.get<logger>()->trace("client", sp, "Uploading file");

          imt_service::enable_async(sp);
          return this->owner_->logic_->send_request<client_messages::put_object_message_response>(
            sp,
            client_messages::put_object_message(
              response.id(),
              msg,
              signature,
              version_id,
              tmp_file,
              file_hash.signature()).serialize(),
            std::chrono::seconds(10 * 60))
            .then([](
              const std::function<void(const service_provider & /*sp*/)> & done,
              const error_handler & on_error,
              const service_provider & sp,
              const client_messages::put_object_message_response & response) {
            done(sp);
          });
        });
      });
    })
      .then([version_id](
        const std::function<void(const service_provider & sp, const std::string & version_id)> & done,
        const error_handler & on_error,
        const service_provider & sp) {
      done(sp, version_id.str());
    });
  });
}

vds::async_task<const vds::guid & /*version_id*/>
vds::_client::download_data(
  const service_provider & sp,
  const std::string & name,
  const filename & target_file)
{
  return this->owner_->logic_->send_request<client_messages::server_log_state_response>(
    sp,
    client_messages::server_log_state_request().serialize())
    .then([this, name, target_file](
      const service_provider & sp,
      const client_messages::server_log_state_response & response) {
        
        sp.get<logger>()->trace("client", sp, "Waiting file");
        return asymmetric_private_key::parse_der(sp, base64::to_bytes(response.private_key_body()), user_password)
          .then([this, response, name, target_file](
            const service_provider & sp,
            const asymmetric_private_key & user_private_key) {
          return this->looking_for_file(
            sp,
            user_private_key,
            response.id(),
            response.order_num(),
            name,
            target_file);
        });
      });
}

vds::async_task<const vds::guid & /*version_id*/>
  vds::_client::looking_for_file(
    const service_provider & sp,
    const asymmetric_private_key & user_private_key,
    const guid & principal_id,
    const size_t order_num,
    const std::string & looking_file_name,
    const filename & target_file)
{
  return create_async_task(
    [this, user_private_key, principal_id, order_num, looking_file_name, target_file](
      const std::function<void(const service_provider & sp, const guid & version_id)> & done,
      const error_handler & on_error,
      const service_provider & sp) {
      this->owner_->logic_->send_request<client_messages::principal_log_response>(
        sp,
        client_messages::principal_log_request(principal_id, order_num).serialize())
      .wait([this, done, on_error, user_private_key, principal_id, looking_file_name, target_file]
        (const service_provider & sp, const client_messages::principal_log_response & principal_log) {
          for(auto p = principal_log.records().rbegin(); p != principal_log.records().rend(); ++p){
            auto & log_record = *p;
            
            auto log_message = std::dynamic_pointer_cast<json_object>(log_record.message());
            if(!log_message) {
              continue;
            }
            
            std::cout << "id=" << log_record.id().str() << ", principal_id=" << log_record.principal_id().str()
            << "\n";
            
            std::string message_type;
            if(!log_message->get_property("$t", message_type, false)
              || principal_log_new_object::message_type != message_type) {
              continue;
            }
            
            principal_log_new_object record(log_message);
            
            auto decrypted_data = user_private_key.decrypt(record.meta_data());
            binary_deserializer file_info(decrypted_data);
            
            std::string name;
            file_info >> name;
            
            if(looking_file_name != name) {
              continue;
            }
            
            size_t length;
            size_t body_size;
            size_t tail_size;
            
            file_info >> length >> body_size >> tail_size;
            
            symmetric_key transaction_key(symmetric_crypto::aes_256_cbc(), file_info);
            
            sp.get<logger>()->trace("client", sp, "Waiting file");
            filename tmp_file(this->tmp_folder_, record.index().str());

            auto version_id = record.index();
            
            this->download_file(
              version_id,
              tmp_file)
            .wait(
              [this, done, on_error, transaction_key, tmp_file, version_id, target_file, body_size, tail_size]
              (const service_provider & sp) {
                _file_manager::decrypt_file(
                  sp,
                  transaction_key,
                  tmp_file,
                  target_file,
                  body_size,
                  tail_size)
                .wait(
                  [done, version_id](const service_provider & sp){
                    done(sp, version_id);
                  },
                  on_error,
                  sp);
              },
              on_error,
              sp);
            
            return;
          }
          
          if(!principal_log.records().empty()){
            this->looking_for_file(
              sp,
              user_private_key,
              principal_id,
              principal_log.last_order_num() - 1,
              looking_file_name,
              target_file)
            .wait(done, on_error, sp);
          }
          else {
            on_error(sp, std::make_shared<std::runtime_error>("File " + looking_file_name + " not found"));
          }
        },
        on_error,
        sp);
  });
}

vds::async_task<> vds::_client::download_file(
  const guid & version_id,
  const filename & tmp_file)
{
  return create_async_task(
    [this, version_id, tmp_file](
      const std::function<void(const service_provider & sp)> & done,
      const error_handler & on_error,
      const service_provider & sp) {
  this->owner_->logic_->send_request<client_messages::get_object_response>(
    sp,
    client_messages::get_object_request(version_id, tmp_file).serialize(),
    std::chrono::minutes(10))
  .wait(
    [this, version_id, tmp_file, done, on_error]
    (const service_provider & sp, const client_messages::get_object_response & response) {
      switch(response.state().status){
        case server_task_manager::task_status::DONE:
          done(sp);
          break;
          
        case server_task_manager::task_status::FAILED:
          on_error(sp, std::make_shared<std::runtime_error>(response.state().current_task));
          break;
          
        default:
          std::cout << response.state().current_task << "[" << response.state().progress_percent << "]\n";
          std::this_thread::sleep_for(std::chrono::seconds(5));
          this->download_file(version_id, tmp_file).wait(done, on_error, sp);
          break;
      }
    },
    on_error,
    sp);
    });
}

vds::certificate vds::_client::load_user_certificate(const service_provider & sp)
{
  certificate result;
  result.load(filename(foldername(persistence::current_user(sp), ".vds"), "user.crt"));
  return result;
}

vds::asymmetric_private_key vds::_client::load_user_private_key(const service_provider & sp)
{
  asymmetric_private_key result;
  result.load(filename(foldername(persistence::current_user(sp), ".vds"), "user.pkey"));
  return result;
}


vds::async_task<const vds::client_messages::certificate_and_key_response & /*response*/>
  vds::_client::authenticate(
    const service_provider & sp,
    const std::string & user_login,
    const std::string & user_password)
{
  sp.get<logger>()->trace("client", sp, "Authenticating user %s", user_login.c_str());

  hash ph(hash::sha256());
  ph.update(user_password.c_str(), user_password.length());
  ph.final();

  return this->owner_->logic_->send_request<client_messages::certificate_and_key_response>(
    sp,
    client_messages::certificate_and_key_request(
      user_login,
      ph.signature()).serialize())
    .then([user_password](const std::function<void(
      const service_provider & /*sp*/,
      const client_messages::certificate_and_key_response & /*response*/)> & done,
      const error_handler & on_error,
      const service_provider & sp,
      const client_messages::certificate_and_key_response & response) {
    done(sp, response);
  });
}

