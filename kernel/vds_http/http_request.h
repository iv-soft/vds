//#ifndef __VDS_HTTP_HTTP_REQUEST_H_
//#define __VDS_HTTP_HTTP_REQUEST_H_
//
///*
//Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
//All rights reserved
//*/
//
//#include <memory>
//#include <list>
//#include "http_message.h"
//
//namespace vds {
//  class http_request
//  {
//  public:
//    http_request(const http_message & message);
//
//    http_request(
//        const std::list<std::string> & headers,
//        const std::string & method,
//        const std::string & url,
//        const std::string & agent = "HTTP/1.0",
//        const std::shared_ptr<stream_input_async<uint8_t>> & body = std::make_shared<buffer_stream_input_async>(const_data_buffer())
//      ): message_(headers, body), method_(method), url_(url), agent_(agent)
//    {
//      this->parse_parameters();
//    }
//
//    static http_request create(
//      const std::string & method,
//      const std::string & url,
//      const std::string & agent = "HTTP/1.0",
//      const std::shared_ptr<stream_input_async<uint8_t>> & body = std::make_shared<buffer_stream_input_async>(const_data_buffer()))
//    {
//      std::list<std::string> headers;
//      headers.push_back(method + " " + url + " " + agent);
//
//      return http_request(headers, method, url, agent, body);
//    }
//
//    const std::string & url() const {
//      return this->url_;
//    }
//
//    const std::string & method() const {
//      return this->method_;
//    }
//    const std::string & agent() const {
//      return this->agent_;
//    }
//
//    const std::list<std::string> & headers() {
//      return this->message_.headers();
//    }
//
//    bool get_header(const std::string & name, std::string & value) const {
//      return this->message_.get_header(name, value);
//    }
//    
//    const http_message & get_message() const {
//      return this->message_;
//    }
//
//    std::string get_parameter(const std::string & name) const;
//    
//    static http_message simple_request(
//      
//      const std::string & method,
//      const std::string & url,
//      const std::string & body);
//
//  private:
//    http_message message_;
//    std::string method_;
//    std::string url_;
//    std::list<std::string> parameters_;
//    std::string agent_;
//
//    void parse_parameters();
//  };
//}
//
//#endif // __VDS_HTTP_HTTP_REQUEST_H_
