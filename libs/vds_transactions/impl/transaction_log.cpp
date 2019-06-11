/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "private/stdafx.h"
#include <set>
#include "include/transaction_log.h"
#include "asymmetriccrypto.h"
#include "database_orm.h"
#include "db_model.h"
#include "transaction_block_builder.h"
#include "transaction_log_hierarchy_dbo.h"
#include "transaction_log_record_dbo.h"
#include "encoding.h"
#include "user_manager.h"
#include "vds_exceptions.h"
#include "logger.h"
#include "channel_message_dbo.h"
#include "include/transaction_state_calculator.h"
#include "transaction_log_vote_request_dbo.h"
#include "member_user_dbo.h"
#include "transaction_log_balance_dbo.h"
#include "database.h"
#include "datacoin_balance_dbo.h"

vds::expected<vds::const_data_buffer> vds::transactions::transaction_log::save(
	const service_provider * sp,
	database_transaction & t,
	const const_data_buffer & block_data)
{
  GET_EXPECTED(block, transaction_block::create(block_data));

  GET_EXPECTED(block_exists, block.exists(t));
  vds_assert(!block_exists);

  orm::transaction_log_record_dbo t1;
  CHECK_EXPECTED(
  t.execute(
    t1.insert(
      t1.id = block.id(),
      t1.data = block_data,
      t1.state = orm::transaction_log_record_dbo::state_t::validated,
      t1.new_member = false,
      t1.consensus = block.ancestors().empty(),
      t1.order_no = block.order_no(),
      t1.time_point = block.time_point())));

  orm::transaction_log_hierarchy_dbo t2;
  for (const auto & ancestor : block.ancestors()) {
    CHECK_EXPECTED(t.execute(t2.insert(
      t2.id = ancestor,
      t2.follower_id = block.id()
    )));
  }

  CHECK_EXPECTED(process_block(sp, t, block_data));

  return block.id();
}

vds::expected<void> vds::transactions::transaction_log::process_block(
  const service_provider* sp,
  database_transaction& t,
  const const_data_buffer& block_data) {
  GET_EXPECTED(block, transaction_block::create(block_data));

  orm::transaction_log_record_dbo t1;

  //Check ancestors
  std::set<const_data_buffer> remove_leaf;
  auto state = orm::transaction_log_record_dbo::state_t::leaf;
  for (const auto & ancestor : block.ancestors()) {
    GET_EXPECTED(st, t.get_reader(t1.select(t1.state,t1.order_no,t1.time_point).where(t1.id == ancestor)));
    GET_EXPECTED(st_execute, st.execute());
    if (!st_execute){
      return expected<void>();
    }
    else {
      if(safe_cast<uint64_t>(t1.order_no.get(st)) >= block.order_no() || t1.time_point.get(st) > block.time_point()) {
        state = orm::transaction_log_record_dbo::state_t::invalid;
        vds_assert(false);
      }
      else {
        switch (static_cast<orm::transaction_log_record_dbo::state_t>(t1.state.get(st))) {
        case orm::transaction_log_record_dbo::state_t::leaf: {
          remove_leaf.emplace(ancestor);
          break;
        }

        case orm::transaction_log_record_dbo::state_t::processed:
          break;

        case orm::transaction_log_record_dbo::state_t::validated: {
          sp->get<logger>()->trace(
            ThisModule,
            "Ancestor %s is not processed. So stop processing this block.",
            base64::from_bytes(ancestor).c_str());
          return expected<void>();
        }

        case orm::transaction_log_record_dbo::state_t::invalid:
          state = orm::transaction_log_record_dbo::state_t::invalid;
          //TODO: And?
          break;

        default:
          return vds::make_unexpected<std::runtime_error>("Invalid program");
        }
      }
    }
  }

  GET_EXPECTED(check_consensus_result, check_consensus(t, block.id()));
  if(check_consensus_result) {
    CHECK_EXPECTED(t.execute(t1.update(t1.consensus = true).where(t1.id == block.id())));
  }


  CHECK_EXPECTED(update_consensus(sp, t, block_data));
  CHECK_EXPECTED(t.execute(t1.update(t1.state = state).where(t1.id == block.id())));

  if (orm::transaction_log_record_dbo::state_t::leaf == state) {
    for (const auto & p : remove_leaf) {
      CHECK_EXPECTED(t.execute(
        t1.update(
          t1.state = orm::transaction_log_record_dbo::state_t::processed)
        .where(t1.id == p)));
    }
  }

  if (orm::transaction_log_record_dbo::state_t::leaf == state
    || orm::transaction_log_record_dbo::state_t::processed == state) {
    CHECK_EXPECTED(process_records(sp, t, block));
  }

  //process followers
  std::set<const_data_buffer> followers;
  orm::transaction_log_hierarchy_dbo t4;
  GET_EXPECTED(st, t.get_reader(t4.select(t4.follower_id).where(t4.id == block.id())));
  WHILE_EXPECTED (st.execute()) {
    const auto follower_id = t4.follower_id.get(st);
    if (follower_id) {
      followers.emplace(follower_id);
    }
  }
  WHILE_EXPECTED_END()

  for (const auto &p : followers) {
    GET_EXPECTED_VALUE(st, t.get_reader(t1.select(t1.data).where(t1.id == p)));
    GET_EXPECTED(st_execute_result, st.execute());
    if (!st_execute_result) {
      return vds::make_unexpected<std::runtime_error>("Invalid data");
    }

    if(state == orm::transaction_log_record_dbo::state_t::leaf) {
      CHECK_EXPECTED(process_block(sp, t, t1.data.get(st)));
    }
    else {
      CHECK_EXPECTED(invalid_block(sp, t, p));
    }
  }

  return expected<void>();
}

vds::expected<void> vds::transactions::transaction_log::update_consensus(
  const service_provider* sp,
  database_transaction& t,
  const const_data_buffer& block_data) {
  
  GET_EXPECTED(block, transaction_block::create(block_data));

  auto leaf_owner = block.write_cert_id();

  std::map<const_data_buffer, const_data_buffer> not_processed;
  std::map<const_data_buffer, const_data_buffer> processed;
  std::set<const_data_buffer> consensus_candidate;

  not_processed[block.id()] = block_data;

  while (!not_processed.empty()) {
    auto pbegin = not_processed.begin();
    auto data = pbegin->second;
    not_processed.erase(pbegin);

    GET_EXPECTED_VALUE(block, transaction_block::create(data));

    orm::transaction_log_record_dbo t1;
    for (const auto & ancestor : block.ancestors()) {
      if(processed.end() != processed.find(ancestor) || not_processed.end() != not_processed.find(ancestor)) {
        continue;
      }

      GET_EXPECTED(st, t.get_reader(t1.select(t1.state, t1.data).where(t1.id == ancestor)));
      GET_EXPECTED(st_execute, st.execute());
      if (!st_execute) {
        return vds::make_unexpected<std::runtime_error>("Invalid data");
      }

      const auto ancestor_data = t1.data.get(st);

      orm::transaction_log_vote_request_dbo t2;
      CHECK_EXPECTED(t.execute(
        t2.update(t2.approved = true)
        .where(t2.id == ancestor && t2.owner == leaf_owner)));

      GET_EXPECTED(check_consensus_result, check_consensus(t, ancestor));
      if(check_consensus_result) {
        consensus_candidate.emplace(ancestor);
      }

      if(block.write_cert_id() != leaf_owner){
        not_processed[ancestor] = ancestor_data;
      }
    }
  }
  for(auto & candidate : consensus_candidate) {
    CHECK_EXPECTED(make_consensus(sp, t, candidate));
  }

  return expected<void>();
}

vds::expected<bool> vds::transactions::transaction_log::process_records(const service_provider * sp, database_transaction & t, const transaction_block & block)
{
  std::stack<std::function<expected<void>()>> undo_actions;
  GET_EXPECTED(completed, block.walk_messages(
    [sp, &t, &undo_actions, id = block.id()](const payment_transaction & message) -> expected<bool> {
      GET_EXPECTED(result, apply_record(sp, t, message, id));
      if (result) {
        undo_actions.push([sp, &t, message, id]() {return undo_record(sp, t, message, id); });
      }
      return result;
    },
    [sp, &t, &undo_actions, id = block.id()](const channel_message & message) -> expected<bool> {
      GET_EXPECTED(result, apply_record(sp, t, message, id));
      if (result) {
        undo_actions.push([sp, &t, message, id]() {return undo_record(sp, t, message, id); });
      }
      return result;
    },
    [sp, &t, &undo_actions, id = block.id()](const create_user_transaction & message) -> expected<bool> {
      GET_EXPECTED(result, apply_record(sp, t, message, id));
      if (result) {
        undo_actions.push([sp, &t, message, id]() {return undo_record(sp, t, message, id); });
      }
      return result;
    },
    [sp, &t, &undo_actions, id = block.id()](const node_add_transaction & message) -> expected<bool> {
      GET_EXPECTED(result, apply_record(sp, t, message, id));
      if (result) {
        undo_actions.push([sp, &t, message, id]() {return undo_record(sp, t, message, id); });
      }
      return result;
    }
  ));

  if (!completed) {
    while (!undo_actions.empty()) {
      CHECK_EXPECTED(undo_actions.top()());
      undo_actions.pop();
    }
  }

  return completed;
}

vds::expected<void> vds::transactions::transaction_log::invalid_block(
  const service_provider * sp,
  class database_transaction &t,
  const const_data_buffer & block_id) {

  orm::transaction_log_record_dbo t1;
  CHECK_EXPECTED(t.execute(t1.update(t1.state = orm::transaction_log_record_dbo::state_t::invalid).where(t1.id == block_id)));

  std::set<const_data_buffer> followers;
  orm::transaction_log_hierarchy_dbo t4;
  GET_EXPECTED(st, t.get_reader(t4.select(t4.follower_id).where(t4.id == block_id)));
  WHILE_EXPECTED (st.execute()) {
    const auto follower_id = t4.follower_id.get(st);
    if (follower_id) {
      followers.emplace(follower_id);
    }
  }
  WHILE_EXPECTED_END()

  for (const auto &p : followers) {

    GET_EXPECTED_VALUE(st, t.get_reader(t1.select(t1.data).where(t1.id == p)));
    GET_EXPECTED(st_execute, st.execute());
    if (!st_execute) {
      return vds::make_unexpected<std::runtime_error>("Invalid data");
    }

    CHECK_EXPECTED(invalid_block(sp, t, p));
  }

  return expected<void>();
}

vds::expected<void> vds::transactions::transaction_log::invalid_become_consensus(const service_provider* sp,
  const database_transaction& t, const const_data_buffer& log_id) {
  return vds::make_unexpected<std::runtime_error>("Not implemented");

  //std::set<const_data_buffer> not_processed;
  //std::set<const_data_buffer> processed;

  //orm::transaction_log_record_dbo t1;
  //auto st = t.get_reader(t1.select(t1.order_no, t1.consensus).where(t1.state == orm::transaction_log_record_dbo::state_t::leaf));
  //while(st.execute()) {
  //  
  //}
}

vds::expected<void> vds::transactions::transaction_log::make_consensus(const service_provider* sp, database_transaction& t,
  const const_data_buffer& start_log_id) {

  std::set<const_data_buffer> not_processed;
  std::set<const_data_buffer> processed;
  not_processed.emplace(start_log_id);

  while (!not_processed.empty()) {
    auto log_id = *not_processed.begin();
    not_processed.erase(not_processed.begin());
    processed.emplace(log_id);


    orm::transaction_log_record_dbo t1;
    GET_EXPECTED(st, t.get_reader(t1.select(t1.state, t1.data, t1.consensus).where(t1.id == log_id)));
    GET_EXPECTED(st_execute, st.execute());
    if (!st_execute) {
      return vds::make_unexpected<std::runtime_error>("Invalid data");
    }

    if (t1.consensus.get(st)) {
      continue;
    }

    const auto state = t1.state.get(st);

    //check all ancestors in consensus
    auto all_ancestors_in_consensus = true;
    GET_EXPECTED(block, transaction_block::create(t1.data.get(st)));

    for (const auto & ancestor : block.ancestors()) {
      GET_EXPECTED_VALUE(st, t.get_reader(t1.select(t1.consensus).where(t1.id == ancestor)));
      GET_EXPECTED_VALUE(st_execute, st.execute());
      if (!st_execute) {
        return vds::make_unexpected<std::runtime_error>("Invalid data");
      }
      if (!t1.consensus.get(st)) {
        all_ancestors_in_consensus = false;
        break;
      }
    }

    if(!all_ancestors_in_consensus) {
      continue;
    }

    CHECK_EXPECTED(t.execute(
      t1.update(
        t1.consensus = true)
      .where(t1.id == log_id)));

    switch (state) {
    case orm::transaction_log_record_dbo::state_t::invalid:
      CHECK_EXPECTED(invalid_become_consensus(sp, t, log_id));
      break;

    case orm::transaction_log_record_dbo::state_t::processed: {
      std::set<const_data_buffer> followers;
      orm::transaction_log_hierarchy_dbo t3;
      GET_EXPECTED_VALUE(st, t.get_reader(t3.select(t3.follower_id).where(t3.id == log_id)));
      WHILE_EXPECTED(st.execute()) {
        auto follower_id = t3.follower_id.get(st);
        if(not_processed.end() == not_processed.find(follower_id)
          && processed.end() == processed.find(follower_id)) {
          followers.emplace(follower_id);
        }
      }
      WHILE_EXPECTED_END()

      for(const auto & follower_id : followers) {
        GET_EXPECTED(check_consensus_result, check_consensus(t, follower_id));
        if (check_consensus_result) {
          not_processed.emplace(follower_id);
        }
      }

      break;
    }

    case orm::transaction_log_record_dbo::state_t::leaf: {
      break;
    }

    default:
      return vds::make_unexpected<std::runtime_error>("Invalid program");
    }
  }

  return expected<void>();
}

vds::expected<bool> vds::transactions::transaction_log::check_consensus(
  database_read_transaction& t,
  const const_data_buffer & log_id) {

  orm::transaction_log_vote_request_dbo t2;

  db_value<int> appoved_count;
  GET_EXPECTED(st, t.get_reader(
    t2.select(db_count(t2.owner).as(appoved_count))
    .where(t2.id == log_id && t2.approved == true)));
  GET_EXPECTED(st_execute, st.execute());
  if (!st_execute) {
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }

  auto ac = appoved_count.get(st);

  db_value<int> total_count;
  GET_EXPECTED_VALUE(st, t.get_reader(
    t2.select(db_count(t2.owner).as(total_count))
    .where(t2.id == log_id)));
  GET_EXPECTED_VALUE(st_execute, st.execute());
  if (!st_execute) {
    return vds::make_unexpected<std::runtime_error>("Invalid data");
  }
  const auto tc = total_count.get(st);

  return (ac > tc / 2);
}


vds::expected<bool> vds::transactions::transaction_log::apply_record(
  const service_provider * sp,
  database_transaction & t,
  const payment_transaction & message,
  const const_data_buffer & block_id)
{
  vds::orm::datacoin_balance_dbo t1;

  GET_EXPECTED(st, t.get_reader(
    t1
    .select(t1.proposed_balance)
    .where(
      t1.owner == message.source_user
      && t1.issuer == message.issuer
      && t1.currency == message.currency
      && t1.source_transaction == message.source_transaction
      )));
  
  GET_EXPECTED(st_execute, st.execute());
  if (!st_execute) {
    return false;
  }

  auto balance = t1.proposed_balance.get(st);
  if (balance < message.value) {
    return false;
  }

  if (balance == message.value) {
    CHECK_EXPECTED(t.execute(
      t1.delete_if(
        t1.owner == message.source_user
        && t1.issuer == message.issuer
        && t1.currency == message.currency
        && t1.source_transaction == message.source_transaction
        )));
  }
  else{
    CHECK_EXPECTED(t.execute(
      t1.update(t1.proposed_balance = balance - message.value)
      .where(
        t1.owner == message.source_user
        && t1.issuer == message.issuer
        && t1.currency == message.currency
        && t1.source_transaction == message.source_transaction
        )));

  }

  CHECK_EXPECTED(t.execute(
    t1.insert(
      t1.proposed_balance = message.value,
      t1.owner = message.target_user,
      t1.issuer = message.issuer,
      t1.currency = message.currency,
      t1.source_transaction = block_id
      )));

  return true;
}

vds::expected<void> vds::transactions::transaction_log::undo_record(
  const service_provider * sp,
  database_transaction & t,
  const payment_transaction & message,
  const const_data_buffer & block_id)
{
  vds::orm::datacoin_balance_dbo t1;

  GET_EXPECTED(st, t.get_reader(
    t1
    .select(t1.proposed_balance)
    .where(
      t1.owner == message.source_user
      && t1.issuer == message.issuer
      && t1.currency == message.currency
      && t1.source_transaction == message.source_transaction
      )));

  GET_EXPECTED(st_execute, st.execute());
  if (st_execute) {
    auto balance = t1.proposed_balance.get(st);
    CHECK_EXPECTED(t.execute(
      t1.update(t1.proposed_balance = balance + message.value)
      .where(
        t1.owner == message.source_user
        && t1.issuer == message.issuer
        && t1.currency == message.currency
        && t1.source_transaction == message.source_transaction
        )));
  }
  else {
    CHECK_EXPECTED(t.execute(
      t1.insert(
        t1.proposed_balance = message.value,
        t1.owner = message.source_user,
        t1.issuer = message.issuer,
        t1.currency = message.currency,
        t1.source_transaction = message.source_transaction
        )));
  }

  CHECK_EXPECTED(t.execute(
    t1.delete_if(
      t1.owner == message.target_user
      && t1.issuer == message.issuer
      && t1.currency == message.currency
      && t1.source_transaction == block_id
    )));

  return expected<void>();
}

vds::expected<bool> vds::transactions::transaction_log::apply_record(
  const service_provider * sp,
  database_transaction & t,
  const channel_message & message,
  const const_data_buffer & block_id)
{
  orm::channel_message_dbo t1;
  CHECK_EXPECTED(t.execute(
    t1.insert(
      t1.block_id = block_id,
      t1.channel_id = message.channel_id(),
      t1.channel_read_cert_subject = message.channel_read_cert_subject(),
      t1.write_cert_subject = message.write_cert_subject(),
      t1.crypted_key = message.crypted_key(),
      t1.crypted_data = message.crypted_data(),
      t1.signature = message.signature()
    )));
  return true;
}

vds::expected<void> vds::transactions::transaction_log::undo_record(const service_provider * sp, database_transaction & t, const channel_message & message, const const_data_buffer & block_id)
{
  orm::channel_message_dbo t1;
  CHECK_EXPECTED(t.execute(
    t1.delete_if(
      t1.block_id == block_id
      && t1.channel_id == message.channel_id()
      && t1.channel_read_cert_subject == message.channel_read_cert_subject()
      && t1.write_cert_subject == message.write_cert_subject()
      && t1.crypted_key == message.crypted_key()
      && t1.signature == message.signature()
    )));
  return expected<void>();
}

vds::expected<bool> vds::transactions::transaction_log::apply_record(const service_provider * sp, database_transaction & t, const create_user_transaction & message, const const_data_buffer & block_id)
{
  return expected<bool>();
}

vds::expected<void> vds::transactions::transaction_log::undo_record(const service_provider * sp, database_transaction & t, const create_user_transaction & message, const const_data_buffer & block_id)
{
  return expected<void>();
}

vds::expected<bool> vds::transactions::transaction_log::apply_record(const service_provider * sp, database_transaction & t, const node_add_transaction & message, const const_data_buffer & block_id)
{
  return expected<bool>();
}

vds::expected<void> vds::transactions::transaction_log::undo_record(const service_provider * sp, database_transaction & t, const node_add_transaction & message, const const_data_buffer & block_id)
{
  return expected<void>();
}

