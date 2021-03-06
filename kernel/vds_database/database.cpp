/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/
#include "stdafx.h"
#include "database.h"
#include "private/database_p.h"

vds::database::database()
{
}

vds::database::~database()
{
}

vds::expected<void> vds::database::open(const service_provider * sp, const filename & fn)
{
  this->impl_ = std::make_shared<_database>(sp);
  return this->impl_->open(fn);
}

vds::expected<void> vds::database::close()
{
  return this->impl_->close();
}

vds::async_task<vds::expected<void>> vds::database::async_transaction(
  lambda_holder_t<expected<bool>, class database_transaction &> callback)
{
  return this->impl_->async_transaction(std::move(callback));
}

vds::async_task<vds::expected<void>> vds::database::async_read_transaction(
  lambda_holder_t<expected<void>, class database_read_transaction &> callback) {
  return this->impl_->async_read_transaction(std::move(callback));
}

vds::async_task<vds::expected<void>> vds::database::prepare_to_stop() {
  if (nullptr == this->impl_) {
    return vds::async_task<vds::expected<void>>(vds::expected<void>());
  }

  return this->impl_->prepare_to_stop();
}

size_t vds::database::queue_length() const {
  return this->impl_->queue_length();
}

vds::expected<void> vds::database_transaction::execute(const char * sql)
{
   return this->impl_->execute(sql);
}

vds::expected<int> vds::database_transaction::rows_modified() const {
  return this->impl_->rows_modified();
}

vds::expected<int> vds::database_transaction::last_insert_rowid() const {
  return this->impl_->last_insert_rowid();
}

vds::expected<vds::sql_statement> vds::database_read_transaction::parse(const char * sql) const
{
  return this->impl_->parse(sql);
}

vds::sql_statement::sql_statement()
  : impl_(nullptr)
{
}

vds::sql_statement::sql_statement(_sql_statement * impl)
  : impl_(impl)
{
}

vds::sql_statement::sql_statement(sql_statement && original)
  : impl_(original.impl_)
{
  original.impl_ = nullptr;
}

vds::sql_statement::~sql_statement()
{
  delete this->impl_;
}

void vds::sql_statement::set_parameter(int index, int value)
{
  this->impl_->set_parameter(index, value);
}

void vds::sql_statement::set_parameter(int index, int64_t value)
{
  this->impl_->set_parameter(index, value);
}

void vds::sql_statement::set_parameter(int index, const std::string & value)
{
  this->impl_->set_parameter(index, value);
}

void vds::sql_statement::set_parameter(int index, const const_data_buffer & value)
{
  this->impl_->set_parameter(index, value);
}

void vds::sql_statement::set_parameter(int index, const std::chrono::system_clock::time_point &value) {
  this->impl_->set_parameter(index, value);
}

vds::expected<bool> vds::sql_statement::execute()
{
  return this->impl_->execute();
}

bool vds::sql_statement::get_value(int index, int & value)
{
  return this->impl_->get_value(index, value);
}

bool vds::sql_statement::get_value(int index, int64_t & value)
{
  return this->impl_->get_value(index, value);
}

bool vds::sql_statement::get_value(int index, std::string & value)
{
  return this->impl_->get_value(index, value);
}

bool vds::sql_statement::get_value(int index, const_data_buffer & value)
{
  return this->impl_->get_value(index, value);
}

bool vds::sql_statement::get_value(int index, double & value)
{
  return this->impl_->get_value(index, value);
}

bool vds::sql_statement::get_value(int index, std::chrono::system_clock::time_point &value) {
  return this->impl_->get_value(index, value);
}

vds::sql_statement& vds::sql_statement::operator= (vds::sql_statement&& original)
{
  delete this->impl_;
  this->impl_ = original.impl_;
  original.impl_ = nullptr;
  
  return *this;
}

bool vds::sql_statement::is_null(int index) const {
  return this->impl_->is_null(index);
}


//////////////////////////////////////
