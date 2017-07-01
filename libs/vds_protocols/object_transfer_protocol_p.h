#ifndef __VDS_PROTOCOLS_OBJECT_TRANSFER_PROTOCOL_P_H_
#define __VDS_PROTOCOLS_OBJECT_TRANSFER_PROTOCOL_P_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "object_transfer_protocol.h"
#include "messages.h"

namespace vds {
  class route_hop;
  class object_request;

  class _object_transfer_protocol
  {
  public:
    _object_transfer_protocol();
    ~_object_transfer_protocol();
    

    void on_object_request(
      const service_provider & sp,
      const guid & from_server_id,
      const object_request & message);
    
  private:
    
  };

  class route_hop
  {
  public:
    route_hop(
      const guid & server_id,
      const std::string & return_address);
    
    const guid & server_id() const { return this->server_id_; }
    const std::string & return_address() const { return this->return_address_; }
    
  private:
    guid server_id_;
    std::string return_address_;
  };
  
  class object_request
  {
  public:
    static const uint32_t message_type_id = (uint32_t)message_identification::object_request_message_id;
    
    void serialize(binary_serializer & b) const;
    std::shared_ptr<json_value> serialize() const;
    
    object_request(const const_data_buffer & binary_form);

    object_request(
      const guid & server_id,
      uint64_t index)
    : server_id_(server_id),
      index_(index)
    {
    }
    
    const guid & server_id() const { return this->server_id_; }
    uint64_t index() const { return this->index_; }
    
  private:
    guid server_id_;
    uint64_t index_;
  };
  
}

#endif // __VDS_PROTOCOLS_OBJECT_TRANSFER_PROTOCOL_P_H_