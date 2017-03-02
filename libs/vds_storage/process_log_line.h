#ifndef __VDS_STORAGE_PROCESS_LOG_LINE_H_
#define __VDS_STORAGE_PROCESS_LOG_LINE_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "log_records.h"

namespace vds {
  template <typename handler_type>
  class process_log_line
  {
  public:
    process_log_line(
      const std::string & stream_name,
      handler_type * owner)
      : stream_name_(stream_name),
      owner_(owner)
    {
    }

    template <typename context_type>
    class handler : public sequence_step<context_type, void(void)>
    {
      using base_class = sequence_step<context_type, void(void)>;
    public:
      handler(
        const context_type & context,
        const process_log_line & args
      ) : base_class(context),
        stream_name_(args.stream_name_),
        owner_(args.owner_)
      {
      }

      void operator()(json_value * log_record) {
        if (nullptr == log_record) {
          this->next();
          return;
        }

        server_log_record record(log_record);
        
        if (nullptr == record.message()) {
          throw new std::runtime_error("Invalid log record in the stream "
            + this->stream_name_
            + "(" + std::to_string(log_record->line()) + "," + std::to_string(log_record->column()) + ")");
        }

        auto message_body = record.message()->serialize()->str();
        hash h(hash::sha256());
        h.update(message_body.c_str(), message_body.length());
        h.final();

        size_t sign_count = 0;
        for (auto & s : record.signatures()) {
          certificate * sign_cert = nullptr;
          if (this->owner_->is_empty()) {
            sign_cert = this->owner_->parse_root_cert(record.message()->get_messages()->get(0));
          }
          else {
            sign_cert = this->owner_->get_cert(s.fingerprint());
          }

          if (s.fingerprint() != sign_cert->fingerprint()) {
            throw new std::runtime_error("Invalid certificate in the stream "
              + this->stream_name_
              + "(" + std::to_string(log_record->line()) + "," + std::to_string(log_record->column()) + ")");
          }

          auto cert_public_key = sign_cert->public_key();
          asymmetric_sign_verify verifier(hash::sha256(), cert_public_key);
          verifier.update(h.signature(), h.signature_length());

          std::vector<uint8_t> sig_data;
          base64::to_bytes(s.signature(), sig_data);
          if (!verifier.verify((const unsigned char *)sig_data.data(), sig_data.size())) {
            throw new std::runtime_error("Invalid sign record in the stream "
              + this->stream_name_
              + "(" + std::to_string(log_record->line()) + "," + std::to_string(log_record->column()) + ")");
          }

          ++sign_count;
        }
        if (0 < sign_count) {
          for (size_t i = 0; i < record.message()->get_messages()->size(); ++i) {
            this->owner_->apply_record(record.message()->get_messages()->get(i));
          }
        }

        this->prev();
      }

    private:
      std::string stream_name_;
      handler_type * owner_;
    };

  private:
    std::string stream_name_;
    handler_type * owner_;
  };
}

#endif // __VDS_STORAGE_PROCESS_LOG_LINE_H_