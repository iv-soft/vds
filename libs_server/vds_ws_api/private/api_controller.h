#ifndef __VDS_WEB_SERVER_API_CONTROLLER_H_
#define __VDS_WEB_SERVER_API_CONTROLLER_H_


/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/


#include "http_message.h"
#include "service_provider.h"
#include "ws_http_server_p.h"

namespace vds {
  class api_controller {
  public:
    //static async_task<expected<std::shared_ptr<json_value>>> get_channels(        
    //  const vds::service_provider * sp,
    //  const std::shared_ptr<user_manager> & user_mng,
    //  const http_message & message);

    //static async_task<expected<std::shared_ptr<json_value>>> get_login_state(
    //  const vds::service_provider * sp,
    //  const std::string & login,
    //  const std::string & password,
    //  const std::shared_ptr<_web_server>& owner,
    //  const http_message& request);

    //static async_task<expected<std::shared_ptr<json_value>>> login(
    //  const vds::service_provider * sp,
    //  const std::string & login,
    //  const std::string & password,
    //  const std::shared_ptr<_web_server>& owner,
    //  const http_message& request);

    //static vds::async_task<vds::expected<void>> create_channel(
    //  const std::shared_ptr<user_manager> & user_mng,
    //  const std::shared_ptr<http_async_serializer> & output_stream,
    //  const std::string & channel_type,
    //  const std::string & name);

    //static vds::async_task<vds::expected<std::shared_ptr<json_value>>> channel_feed(
    //  const vds::service_provider * sp,
    //  const std::shared_ptr<user_manager> & user_mng,
    //  const const_data_buffer & channel_id);

    //static vds::async_task<vds::expected<file_manager::file_operations::download_result_t>>
    //download_file(
    //  const vds::service_provider * sp,
    //  const std::shared_ptr<user_manager> & user_mng,
    //  const const_data_buffer& channel_id,
    //  const std::string& file_name,
    //  const const_data_buffer& file_hash);

    //static vds::async_task<vds::expected<std::shared_ptr<json_value>>>
    //  prepare_download_file(
    //    const vds::service_provider * sp,
    //    const std::shared_ptr<user_manager> & user_mng,
    //    const const_data_buffer& channel_id,
    //    const std::string& file_name,
    //    const const_data_buffer& file_hash);

    //static vds::async_task<vds::expected<void>> lock_device(
    //  const vds::service_provider * sp,
    //  const std::shared_ptr<vds::user_manager> &user_mng,
    //  const std::shared_ptr<vds::_web_server> &owner,
    //  const std::string &device_name,
    //  const std::string &local_path,
    //  uint64_t reserved_size);

    //static vds::async_task<vds::expected<std::shared_ptr<vds::json_value>>>
    //offer_device(
    //  const vds::service_provider * sp,
    //  const std::shared_ptr<user_manager> &user_mng,
    //  const std::shared_ptr<_web_server> &owner);

    static vds::async_task<vds::expected<std::shared_ptr<vds::json_value>>>
    get_statistics(
      const vds::service_provider * sp,
      const http_message& message);

    //static std::shared_ptr<json_value>
    //get_invite(
    //  user_manager& user_mng,
    //  const std::shared_ptr<_web_server>& owner,
    //  const http_message& message);

    //static async_task<expected<std::shared_ptr<json_value>>> get_session(
    //  const std::shared_ptr<auth_session> & session);

    //static async_task<expected<std::shared_ptr<stream_output_async<uint8_t>>>> create_message(
    //  const vds::service_provider * sp,
    //  const std::shared_ptr<http_async_serializer> & output_stream,
    //  const std::shared_ptr<user_manager> &user_mng,
    //  const http_message & request);

  private:
//    static std::shared_ptr<json_object> channel_serialize(const vds::user_channel & channel);

  };
}

#endif //__VDS_WEB_SERVER_API_CONTROLLER_H_
