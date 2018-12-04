#ifndef __VDS_HTTP_HTTP_MULTIPART_REQUEST_H_
#define __VDS_HTTP_HTTP_MULTIPART_REQUEST_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "http_message.h"

namespace vds {
  class http_multipart_request {
  public:
    http_multipart_request(
      const std::string & method,
      const std::string & url,
      const std::string & agent = "HTTP/1.0");

    void add_string(
      const std::string & name,
      const std::string & value);

    void add_file(
      const filename & body_file,
      const std::string & filename,
      const std::string & content_type = "application/octet-stream");


    http_message get_message();

  private:
    std::list<std::string> headers_;
    size_t total_size_;

    class multipart_body : public stream_input_async<uint8_t> {
    public:
      vds::async_task<size_t> read_async(
        uint8_t * buffer,
        size_t len) override;

    private:

      
    };
  };
}

#endif // __VDS_HTTP_HTTP_MULTIPART_REQUEST_H_
