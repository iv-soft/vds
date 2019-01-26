/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "private/stdafx.h"
#include "include/transaction_block.h"
#include "transaction_log_record_dbo.h"
#include "encoding.h"

vds::expected<bool> vds::transactions::transaction_block::validate(const certificate& write_cert) {
  binary_serializer block_data;
  CHECK_EXPECTED(serialize(block_data, this->version_));
  CHECK_EXPECTED(serialize(block_data, (uint64_t)std::chrono::system_clock::to_time_t(this->time_point_)));
  CHECK_EXPECTED(serialize(block_data, this->order_no_));
  CHECK_EXPECTED(serialize(block_data, this->write_cert_id_));
  CHECK_EXPECTED(serialize(block_data, this->ancestors_));
  CHECK_EXPECTED(serialize(block_data, this->block_messages_));

  GET_EXPECTED(write_cert_public_key, write_cert.public_key());

  return asymmetric_sign_verify::verify(
    hash::sha256(),
    write_cert_public_key,
    this->signature_, 
    block_data.get_buffer(),
    block_data.size());
}

vds::expected<bool> vds::transactions::transaction_block::exists(database_transaction& t) {
  orm::transaction_log_record_dbo t1;
  GET_EXPECTED(st, t.get_reader(
    t1
    .select(t1.state)
    .where(t1.id == this->id())));

  return st.execute();
}
