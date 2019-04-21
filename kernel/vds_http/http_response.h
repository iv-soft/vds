#ifndef __VDS_HTTP_HTTP_RESPONSE_H_
#define __VDS_HTTP_HTTP_RESPONSE_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "http_message.h"

namespace vds {
  class http_response
  {
  public:
    static constexpr int HTTP_OK = 200;
    static constexpr int HTTP_Internal_Server_Error = 500;
    static constexpr int HTTP_Found = 302;

    static constexpr int HTTP_Unauthorized = 401;
    static constexpr int HTTP_Not_Found = 404;

    http_response(
      const http_message & comment);

    http_response(
      int code,
      const std::string & comment);

    void add_header(const std::string & name, const std::string & value) {
      this->headers_.push_back(name + ":" + value);
    }
    
    int code() const
    {
      return this->code_;
    }

    const std::string & comment() const
    {
      return this->comment_;
    }

    static http_message simple_text_response(
      const std::string & body,
      const std::string & content_type = "text/html; charset=utf-8",
      int result_code = HTTP_OK,
      const std::string & message = "OK");

    static http_message simple_text_response(
      const std::shared_ptr<stream_input_async<uint8_t>> & body,
      uint64_t body_size,
      const std::string & content_type = "application/octet-stream",
      int result_code = HTTP_OK,
      const std::string & message = "OK");

    static expected<http_message> file_response(
      const filename & body_file,
      const std::string & out_filename,
      const std::string & content_type = "application/octet-stream",
      int result_code = HTTP_OK,
      const std::string & message = "OK");

    static http_message file_response(
      const std::shared_ptr<stream_input_async<uint8_t>> & body,
      uint64_t body_size,
      const std::string & filename,
      const const_data_buffer & file_hash = const_data_buffer(),
      const std::string & content_type = "application/octet-stream",
      int result_code = HTTP_OK,
      const std::string & message = "OK");

    static http_message file_response(
      const const_data_buffer & body,
      const std::string & filename,
      const const_data_buffer & file_hash = const_data_buffer(),
      const std::string & content_type = "application/octet-stream",
      int result_code = HTTP_OK,
      const std::string & message = "OK");

    static http_message redirect(
      const std::string & location);

    static http_message status_response(
        int result_code,
        const std::string & message);

  private:
    std::string protocol_;
    int code_;
    std::string comment_;
    std::list<std::string> headers_;
  };
}

#endif // __VDS_HTTP_HTTP_RESPONSE_H_
