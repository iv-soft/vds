#ifndef __VDS_HTTP_HTTP_RESPONSE_PARSER_H_
#define __VDS_HTTP_HTTP_RESPONSE_PARSER_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "http_incoming_stream.h"

namespace vds {
  class http_response_parser
  {
  public:
    http_response_parser()
    {
    }
    
    template<
      typename context_type
    >
    class handler : public sequence_step<
      context_type,
      void(
        http_response & response,
        http_incoming_stream & incoming_stream
      )
    >
    {
      using base_class = sequence_step<
        context_type,
        void(
          http_response & response,
          http_incoming_stream & incoming_stream
        )
      >;
    public:
      handler(
        const context_type & context,
        const http_response_parser & args
      )
      : base_class(context),
        state_(STATE_PARSE_HEADER)
      {
      }
      
      ~handler()
      {
        std::cout << "http_response_parser::handler::~handler\n";
      }
      
      void operator()(const void * data, size_t len)
      {
        if (0 == len) {
          this->next(
            this->response_,
            this->incoming_stream_);
        }
        else {
          this->data_ = data;
          this->len_ = len;

          this->processed();
        }
      }
      
      void processed()
      {
        while (0 < this->len_) {
          if (STATE_PARSE_HEADER == this->state_) {
            const char * p = (const char *)memchr(this->data_, '\n', this->len_);
            if (nullptr == p) {
              this->parse_buffer_ += std::string((const char *)this->data_, this->len_);
              this->prev();
              return;
            }

            auto size = p - (const char *)this->data_;

            if (size > 0) {
              if ('\r' == reinterpret_cast<const char *>(this->data_)[size - 1]) {
                this->parse_buffer_ += std::string((const char *)this->data_, size - 1);
              }
              else {
                this->parse_buffer_ += std::string((const char *)this->data_, size);
              }
            }

            if (0 == this->parse_buffer_.length()) {
              if (this->headers_.empty()) {
                throw new std::logic_error("Invalid request");
              }

              auto request = *this->headers_.begin();

              std::string items[3];

              size_t index = 0;
              for (auto ch : request) {
                if (isspace(ch)) {
                  ++index;
                  if (index > sizeof(items) / sizeof(items[0])) {
                    throw new std::logic_error("Invalid request");
                  }
                }
                else {
                  items[index] += ch;
                }
              }

              if ((index + 1) != sizeof(items) / sizeof(items[0])) {
                throw new std::logic_error("Invalid request");
              }

              this->headers_.pop_front();

              this->response_.reset(
                  items[0],
                  items[1],
                  items[2],
                  this->headers_);

              this->data_ = p + 1;
              this->len_ -= size + 1;
              this->headers_.clear();
              
              this->state_ = STATE_PARSE_BODY;
              
              std::string content_length_header;
              if(this->response_.get_header("Content-Length", content_length_header)){
                this->size_limit_ = std::stoul(content_length_header);
              }
              else {
                this->size_limit_ = (size_t)-1;
              }
              this->readed_ = 0;

              this->next(
                this->response_,
                this->incoming_stream_
              );
              
              return;
            }
            else {
              this->headers_.push_back(this->parse_buffer_);
              this->parse_buffer_.clear();
            }

            this->data_ = p + 1;
            this->len_ -= size + 1;
          }
          else {
#undef min
            size_t l = std::min(
              this->size_limit_ -this->readed_,
              this->len_);
            auto p = this->data_;
            this->data_ = reinterpret_cast<const char *>(this->data_) + l;
            this->len_ -= l;
            this->readed_ += l;
            this->incoming_stream_.push_data(
              p,
              l);
            return;
          }
        }
        
        if(STATE_PARSE_BODY == this->state_
          && this->readed_ == this->size_limit_) {
            this->state_ = STATE_PARSE_HEADER;
            this->incoming_stream_.push_data(
              nullptr,
              0);
        }
        else {
          this->prev();
        }
      }
      
    private:
      enum 
      {
        STATE_PARSE_HEADER,
        STATE_PARSE_BODY
      } state_;

      const void * data_;
      size_t len_;

      std::string parse_buffer_;
      std::list<std::string> headers_;
      
      http_response response_;
      http_incoming_stream incoming_stream_;
      
      size_t size_limit_;
      size_t readed_;
    };
  };
}

#endif // __VDS_HTTP_HTTP_RESPONSE_PARSER_H_