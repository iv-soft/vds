#ifndef __VDS_PROTOCOLS_PRINCIPAL_RECORD_H_
#define __VDS_PROTOCOLS_PRINCIPAL_RECORD_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "guid.h"
#include "const_data_buffer.h"
#include "asymmetriccrypto.h"

namespace vds {
  class principal_record
  {
  public:
    principal_record(
      const guid & parent_principal,
      const guid & id,
      const certificate & cert_body,
      const const_data_buffer & cert_key,
      const const_data_buffer & password_hash);

    const guid & parent_principal() const { return this->parent_principal_; }
    const guid & id() const { return this->id_; }
    const certificate & cert_body() const { return this->cert_body_; }
    const const_data_buffer & cert_key() const { return this->cert_key_; }
    const const_data_buffer & password_hash() const { return this->password_hash_; }

  private:
    guid parent_principal_;
    guid id_;
    certificate cert_body_;
    const_data_buffer cert_key_;
    const_data_buffer password_hash_;
  };
}

#endif // __VDS_PROTOCOLS_PRINCIPAL_RECORD_H_
