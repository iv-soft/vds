/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "client_logic.h"

const char * vds::client_logic::endpoints_[] = {
  "https://127.0.0.1:8050",
  nullptr
};

vds::client_logic::client_logic(
  const service_provider & sp,
  certificate * client_certificate,
  asymmetric_private_key * client_private_key)
  : sp_(sp),
  log_(sp, "VDS Client logic"),
  client_certificate_(client_certificate),
  client_private_key_(client_private_key),
  filter_last_index_(0),
  connected_(0),
  update_connection_pool_task_(std::bind(&client_logic::update_connection_pool, this)),
  outgoing_queue_(sp)
{
  
}

vds::client_logic::~client_logic()
{
}

void vds::client_logic::start()
{
  for (auto p = endpoints_; nullptr != *p; ++p) {
    url_parser::parse_addresses(*p,
      [this](const std::string & protocol, const std::string & address)->bool {
      if ("https" == protocol) {
        auto url = url_parser::parse_network_address(address);
        if (protocol == url.protocol) {
          this->connection_queue_.push_back(
            std::unique_ptr<client_connection<client_logic>>(
              new client_connection<client_logic>(
                this->sp_,
                this,
                url.server,
                std::atoi(url.port.c_str()),
                this->client_certificate_,
                this->client_private_key_)));
        }
      }
      return true;
    });
  }

  this->update_connection_pool();
}

void vds::client_logic::stop()
{
  if (std::future_status::ready != this->update_connection_pool_feature_.wait_for(std::chrono::seconds(0))) {
    this->update_connection_pool_feature_.get();
  }
}

void vds::client_logic::connection_closed(client_connection<client_logic>& connection)
{
  this->log_.error("Connection %s:%d has been closed", connection.address().c_str(), connection.port());

  this->connection_mutex_.lock();
  this->connected_--;
  this->connection_mutex_.unlock();

  this->update_connection_pool();
}

void vds::client_logic::connection_error(client_connection<client_logic>& connection, std::exception * ex)
{
  this->log_.error("Connection %s:%d error %s", connection.address().c_str(), connection.port(), ex->what());

  this->connection_mutex_.lock();
  this->connected_--;
  this->connection_mutex_.unlock();

  this->update_connection_pool();
}

void vds::client_logic::process_response(client_connection<client_logic>& connection, const json_value * response)
{
  auto tasks = dynamic_cast<const json_array *>(response);
  if (nullptr != tasks) {
    for (size_t i = 0; i < tasks->size(); ++i) {
      auto task = dynamic_cast<const json_object *>(tasks->get(i));

      if (nullptr != task) {
        std::string task_type;
        if (task->get_property("$t", task_type, false)) {
          std::unique_lock<std::mutex> lock(this->filters_mutex_);
          for (auto & f : this->filters_[task_type]) {
            f.second(task);
          }
        }
      }
    }
  }
}
/*
void vds::client_logic::node_install(const std::string & login, const std::string & password)
{
  hash password_hash(hash::sha256());
  password_hash.update(password.c_str(), password.length());
  password_hash.final();

  this->install_node_prepare_message_.user_id = "login:" + login;
  this->install_node_prepare_message_.password_hash = base64::from_bytes(password_hash.signature(), password_hash.signature_length());
  this->install_node_prepare_message_.request_id = guid::new_guid_string();

  std::unique_ptr<json_object> request(this->install_node_prepare_message_.serialize());

  json_writer body;
  request->str(body);

  this->query_all(body.str());
}
*/

void vds::client_logic::get_commands(vds::client_connection<vds::client_logic> & connection)
{
  this->sp_.get<imt_service>().async(
    [this, &connection](){ this->outgoing_queue_.get(connection); });
}

void vds::client_logic::add_task(const std::string & message)
{
  this->outgoing_queue_.push(message);
}

/*
std::string vds::client_logic::get_messages()
{
  std::string result("[");

  bool bfirst = true;

  std::unique_lock<std::mutex> lock(this->outgoing_queue_mutex_);
  for (auto& message : this->outgoing_queue_) {
    if (bfirst) {
      bfirst = false;
    }
    else {
      result += ',';
    }

    result += message;
  }

  result += ']';

  return result;
}
*/
void vds::client_logic::update_connection_pool()
{
  if (!this->connection_queue_.empty()) {
    if (
      !this->update_connection_pool_feature_.valid()
      || std::future_status::ready == this->update_connection_pool_feature_.wait_for(std::chrono::seconds(0))) {
      this->update_connection_pool_feature_ = std::async(
        std::launch::async,
        [this]() {

        std::chrono::time_point<std::chrono::system_clock> border
          = std::chrono::system_clock::now() - std::chrono::seconds(60);

        size_t try_count = 0;
        this->connection_mutex_.lock();
        while (
          !this->sp_.get_shutdown_event().is_shuting_down()
          && this->connected_ < MAX_CONNECTIONS
          && try_count++ < MAX_CONNECTIONS
          && this->connected_ < this->connection_queue_.size()) {


          auto index = std::rand() % this->connection_queue_.size();
          auto& connection = this->connection_queue_[index];
          if (
            client_connection<client_logic>::NONE == connection->state()
            || (
              client_connection<client_logic>::CONNECT_ERROR == connection->state()
              && border > connection->connection_end()
              )
            ) {

            this->connection_mutex_.unlock();
            connection->connect();
            this->connection_mutex_.lock();

            this->connected_++;
          }
        }

        this->connection_mutex_.unlock();
      });
    }
  }
  
  this->sp_.get<itask_manager>().wait_for(std::chrono::seconds(5)) += this->update_connection_pool_task_;
}

/*
void vds::client_logic::process(client_connection<client_logic>* connection, const install_node_prepared & message)
{
  if (
    this->install_node_prepare_message_.user_id != message.user_id
    && this->install_node_prepare_message_.request_id != message.request_id) {
    return;
  }


}
*/

void vds::client_logic::put_file(
  const std::string & user_login,
  const data_buffer & data)
{
  this->add_task(
    client_messages::put_file_message(
      guid::new_guid().str(),
      user_login,
      base64::from_bytes(data)).serialize()->str());
}

vds::data_buffer vds::client_logic::download_file(const std::string & user_login)
{
  auto request_id = guid::new_guid().str();
  std::string error;
  std::string datagram;

  if (!this->add_task_and_wait<client_messages::get_file_message_response>(
    client_messages::get_file_message_request(
      request_id,
      user_login).serialize()->str(),
    [request_id, &error, &datagram](const client_messages::get_file_message_response & response)->bool {
    if (request_id != response.request_id()) {
      return false;
    }

    error = response.error();
    datagram = response.datagram();
    return true;

  })) {
    throw new std::runtime_error("Timeout at registering new server");
  }

  if (!error.empty()) {
    throw new std::runtime_error(error);
  }

  return base64::to_bytes(datagram);
}
