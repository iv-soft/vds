/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "private/stdafx.h"
#include "include/transaction_block.h"
#include "transaction_log_record_dbo.h"
#include "encoding.h"

vds::expected<vds::transactions::transaction_block> vds::transactions::transaction_block::create(const const_data_buffer& data) {

  GET_EXPECTED(id, hash::signature(hash::sha256(), data));

  binary_deserializer s(data);
  uint32_t version;
  CHECK_EXPECTED(s >> version);

  if (version != CURRENT_VERSION) {
    return vds::make_unexpected<std::runtime_error>("Invalid block version");
  }

  uint64_t time_point;
  CHECK_EXPECTED(s >> time_point);

  uint64_t order_no;
  CHECK_EXPECTED(s >> order_no);

  const_data_buffer write_public_key_id;
  CHECK_EXPECTED(s >> write_public_key_id);

  std::set<const_data_buffer> ancestors;
  CHECK_EXPECTED(s >> ancestors);

  const_data_buffer block_messages;
  CHECK_EXPECTED(s >> block_messages);

  const_data_buffer signature;
  CHECK_EXPECTED(s >> signature);

  return transaction_block(
    version,
    std::chrono::system_clock::from_time_t(time_point),
    std::move(id),
    order_no,
    std::move(write_public_key_id),
    std::move(ancestors),
    std::move(block_messages),
    std::move(signature));
}

vds::expected<vds::const_data_buffer> vds::transactions::transaction_block::build(
  database_transaction& t,
  const const_data_buffer& messages,
  const std::shared_ptr<asymmetric_public_key>& node_public_key,
  const std::shared_ptr<asymmetric_private_key>& node_key)
{
  vds_assert(0 != messages.size());

  //Load
  orm::transaction_log_record_dbo t1;
  GET_EXPECTED(st, t.get_reader(
    t1.select(t1.id, t1.order_no)
    .where(t1.state == orm::transaction_log_record_dbo::state_t::leaf)));
  std::set<const_data_buffer> ancestors;
  uint64_t order_no = 0;
  WHILE_EXPECTED(st.execute())
    ancestors.emplace(t1.id.get(st));
  auto order = safe_cast<uint64_t>(t1.order_no.get(st));
  if (order_no < order) {
    order_no = order;
  }
  WHILE_EXPECTED_END()

    if (ancestors.empty()) {
      //TODO: Check what messages create root user
      //return make_unexpected<std::runtime_error>("transaction logs are empty");
    }

  ++order_no;

  //Sign
  binary_serializer block_data;
  CHECK_EXPECTED(block_data << transaction_block::CURRENT_VERSION);
  CHECK_EXPECTED(block_data << static_cast<uint64_t>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())));
  CHECK_EXPECTED(block_data << order_no);
  CHECK_EXPECTED(block_data << node_public_key->fingerprint());
  CHECK_EXPECTED(block_data << ancestors);
  CHECK_EXPECTED(block_data << messages);

  GET_EXPECTED(sig_data, asymmetric_sign::signature(
    hash::sha256(),
    *node_key,
    block_data.get_buffer(),
    block_data.size()));

  CHECK_EXPECTED(block_data << sig_data);

  return block_data.move_data();
}

vds::expected<bool> vds::transactions::transaction_block::validate(const asymmetric_public_key& write_public_key) {
  binary_serializer block_data;
  CHECK_EXPECTED(block_data << this->version_);
  CHECK_EXPECTED(block_data << (uint64_t)std::chrono::system_clock::to_time_t(this->time_point_));
  CHECK_EXPECTED(block_data << this->order_no_);
  CHECK_EXPECTED(block_data << this->write_public_key_id_);
  CHECK_EXPECTED(block_data << this->ancestors_);
  CHECK_EXPECTED(block_data << this->block_messages_);

  return asymmetric_sign_verify::verify(
    hash::sha256(),
    write_public_key,
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
/////////////////////////
//vds::expected<vds::const_data_buffer> vds::transactions::transaction_block_builder::save(
//  const service_provider* sp,
//  class vds::database_transaction& t,
//  const std::shared_ptr<asymmetric_public_key>& write_public_key,
//  const std::shared_ptr<asymmetric_private_key>& write_private_key) {
//
//  GET_EXPECTED(data, sign(
//    sp,
//    write_public_key,
//    write_private_key));
//
//  CHECK_EXPECTED(transaction_log::save(sp, t, data));
//
//  return hash::signature(hash::sha256(), data);
//}
//
//vds::expected<vds::const_data_buffer> vds::transactions::transaction_block_builder::sign(
//  const service_provider* /*sp*/,
//  const std::shared_ptr<asymmetric_public_key>& write_public_key,
//  const std::shared_ptr<asymmetric_private_key>& write_private_key) {
//  vds_assert(0 != this->data_.size());
//  binary_serializer block_data;
//  CHECK_EXPECTED(block_data << transaction_block::CURRENT_VERSION);
//  CHECK_EXPECTED(block_data << static_cast<uint64_t>(std::chrono::system_clock::to_time_t(this->time_point_)));
//  CHECK_EXPECTED(block_data << this->order_no_);
//  CHECK_EXPECTED(block_data << write_public_key->fingerprint());
//  CHECK_EXPECTED(block_data << this->ancestors_);
//  CHECK_EXPECTED(block_data << this->data_.move_data());
//
//  GET_EXPECTED(sig_data, asymmetric_sign::signature(
//    hash::sha256(),
//    *write_private_key,
//    block_data.get_buffer(),
//    block_data.size()));
//
//  CHECK_EXPECTED(block_data << sig_data);
//
//  return block_data.move_data();
//}
//
//vds::transactions::transaction_block_builder::transaction_block_builder(const service_provider* sp)
//  : sp_(sp), time_point_(std::chrono::system_clock::now()), order_no_(1) {
//}
//
//vds::expected<vds::transactions::transaction_block_builder> vds::transactions::transaction_block_builder::create(
//  const service_provider* sp,
//  vds::database_read_transaction& t) {
//
//  orm::transaction_log_record_dbo t1;
//  GET_EXPECTED(st, t.get_reader(
//    t1.select(t1.id, t1.order_no)
//    .where(t1.state == orm::transaction_log_record_dbo::state_t::leaf)));
//  std::set<const_data_buffer> ancestors;
//  uint64_t order_no = 0;
//  WHILE_EXPECTED(st.execute())
//    ancestors.emplace(t1.id.get(st));
//  auto order = safe_cast<uint64_t>(t1.order_no.get(st));
//  if (order_no < order) {
//    order_no = order;
//  }
//  WHILE_EXPECTED_END()
//
//    if (ancestors.empty()) {
//      return make_unexpected<std::runtime_error>("transaction logs are empty");
//    }
//
//  ++order_no;
//  return expected<transaction_block_builder>(
//    sp,
//    std::chrono::system_clock::now(),
//    ancestors,
//    order_no);
//}
//
//vds::expected<vds::transactions::transaction_block_builder> vds::transactions::transaction_block_builder::create(
//  const service_provider* sp,
//  vds::database_read_transaction& t,
//  const const_data_buffer& data) {
//  GET_EXPECTED(result, create(sp, t));
//  CHECK_EXPECTED(result.data_.push_data(data.data(), data.size(), false));
//  return result;
//}
//
//vds::expected<vds::transactions::transaction_block_builder> vds::transactions::transaction_block_builder::create(
//  const service_provider* sp,
//  vds::database_read_transaction& t,
//  const std::set<const_data_buffer>& ancestors) {
//
//  uint64_t order_no = 0;
//
//  for (const auto& ancestor : ancestors) {
//    orm::transaction_log_record_dbo t1;
//    GET_EXPECTED(st, t.get_reader(
//      t1.select(t1.order_no)
//      .where(t1.id == ancestor)));
//    WHILE_EXPECTED(st.execute())
//      auto order = safe_cast<uint64_t>(t1.order_no.get(st));
//    if (order_no < order) {
//      order_no = order;
//    }
//    WHILE_EXPECTED_END()
//  }
//
//  if (ancestors.empty()) {
//    return make_unexpected<std::runtime_error>("transaction logs are empty");
//  }
//
//  ++order_no;
//  return expected<transaction_block_builder>(sp,
//    std::chrono::system_clock::now(),
//    ancestors,
//    order_no);
//}
