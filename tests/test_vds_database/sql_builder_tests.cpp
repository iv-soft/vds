/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "sql_builder_tests.h"
#include "service_provider.h"
#include "string_utils.h"
#include "test_config.h"

vds::mock_database::mock_database()
: impl_(nullptr)
{
}

vds::mock_database::~mock_database()
{
}

vds::async_task<vds::expected<void>> vds::mock_database::async_transaction(
  lambda_holder_t<expected<bool>, class database_transaction &> callback)
{
  mock_database_transaction t{ std::shared_ptr<_database>() };
  CHECK_EXPECTED_ASYNC(callback(t));
  co_return expected<void>();
}

vds::async_task<vds::expected<void>>  vds::mock_database::async_read_transaction(
  lambda_holder_t<expected<void>, class database_read_transaction &> callback)
{
  mock_database_read_transaction t{ std::shared_ptr<_database>() };
  co_return callback(t);
}

vds::mock_sql_statement::mock_sql_statement() {
}

vds::mock_sql_statement::mock_sql_statement(_sql_statement * ) {
}

vds::mock_sql_statement::mock_sql_statement(mock_sql_statement && )
{
}

vds::mock_sql_statement::~mock_sql_statement()
{
}

vds::mock_sql_statement & vds::mock_sql_statement::operator=(vds::mock_sql_statement && /*original*/) {
  return *this;
}

static int int_parameter_value;

void vds::mock_sql_statement::set_parameter(int , int value)
{
  int_parameter_value = value;
}

void vds::mock_sql_statement::set_parameter(int , int64_t )
{
}

static std::string string_parameter_value;

void vds::mock_sql_statement::set_parameter(int , const std::string & value)
{
  string_parameter_value = value;
}

void vds::mock_sql_statement::set_parameter(int , const const_data_buffer & )
{
}

vds::expected<bool> vds::mock_sql_statement::execute()
{
  return false;
}

static std::string result_sql;

vds::expected<vds::mock_sql_statement> vds::mock_database_read_transaction::parse(const char * sql) const
{
  result_sql = sql;
  return mock_sql_statement(nullptr);
}


TEST(sql_builder_tests, test_select) {

  vds::database db;
  CHECK_EXPECTED_GTEST(db.async_transaction([](vds::database_transaction & trans) -> vds::expected<bool> {

    test_table1 t1;
    test_table2 t2;

    CHECK_EXPECTED(trans.get_reader(
      t1
      .select(vds::db_max(t1.column1), t1.column2, t2.column1)
      .inner_join(t2, t1.column1 == t2.column1)
      .where(t1.column1 == 10 && t2.column2 == "test")
      .order_by(t1.column1, vds::db_desc_order(t1.column1))));

    return true;
  }).get());

  vds::replace_string(result_sql, "?1", "?");
  vds::replace_string(result_sql, "?2", "?");

  ASSERT_EQ(result_sql,
    "SELECT MAX(t0.column1),t0.column2,t1.column1 FROM test_table1 t0 INNER JOIN test_table2 t1 ON t0.column1=t1.column1 WHERE (t0.column1=?) AND (t1.column2=?) ORDER BY t0.column1,t0.column1 DESC");

  ASSERT_EQ(int_parameter_value, 10);
  ASSERT_EQ(string_parameter_value, "test");
}


TEST(sql_builder_tests, test_insert) {

  vds::database db;
  CHECK_EXPECTED_GTEST(db.async_transaction([](vds::database_transaction & trans) -> vds::expected<bool> {
    test_table1 t1;

    CHECK_EXPECTED(trans.execute(
      t1.insert(t1.column1 = 10, t1.column2 = "test")));
    return true;
  }).get());

  vds::replace_string(result_sql, "?1", "?");
  vds::replace_string(result_sql, "?2", "?");

  ASSERT_EQ(result_sql,
    "INSERT INTO test_table1(column1,column2) VALUES (?,?)");

  ASSERT_EQ(int_parameter_value, 10);
  ASSERT_EQ(string_parameter_value, "test");
}

TEST(sql_builder_tests, test_update) {

  vds::database db;
  CHECK_EXPECTED_GTEST(db.async_transaction([](vds::database_transaction & trans) -> vds::expected<bool> {
  test_table1 t1;

  CHECK_EXPECTED(trans.execute(
    t1.update(t1.column1 = 10, t1.column2 = "test").where(t1.column1 == 20)));
  return true;
  }).get());

  vds::replace_string(result_sql, "?1", "?");
  vds::replace_string(result_sql, "?2", "?");
  vds::replace_string(result_sql, "?3", "?");

  ASSERT_EQ(result_sql,
    "UPDATE test_table1 SET column1=?,column2=? WHERE column1=?");

  ASSERT_EQ(string_parameter_value, "test");
}

TEST(sql_builder_tests, test_insert_from) {

  vds::database db;
  CHECK_EXPECTED_GTEST(db.async_transaction([](vds::database_transaction & trans) -> vds::expected<bool> {
  test_table1 t1;
  test_table2 t2;

  CHECK_EXPECTED(trans.execute(
    t1.insert_into(t1.column1, t1.column2)
    .from(t2, vds::db_max(t2.column1), t2.column1, vds::db_max(vds::db_length(t2.column2)))
    .where(t2.column2 == "test")));
  return true;
  }).get());

  ASSERT_EQ(result_sql,
     "INSERT INTO test_table1(column1,column2) SELECT MAX(t0.column1),t0.column1,MAX(LENGTH(t0.column2)) FROM test_table2 t0 WHERE t0.column2=?1");

  ASSERT_EQ(string_parameter_value, "test");
}


TEST(sql_builder_tests, test_delete) {

  vds::database db;
  CHECK_EXPECTED_GTEST(db.async_transaction([](vds::database_transaction & trans) -> vds::expected<bool> {
  test_table1 t1;

  CHECK_EXPECTED(
  trans.execute(
    t1.delete_if(t1.column1 == 10)));
  return true;
  }).get());

  vds::replace_string(result_sql, "?1", "?");

  ASSERT_EQ(result_sql,
    "DELETE FROM test_table1 WHERE test_table1.column1=?");
  ASSERT_EQ(int_parameter_value, 10);
}
