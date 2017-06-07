/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#include "stdafx.h"
#include "principal_record.h"
#include "storage_log.h"
#include "storage_log_p.h"

vds::principal_record::principal_record(
  const guid & object_name,
  const std::string & cert_body,
  const std::string & cert_key,
  const const_data_buffer & password_hash)
  : object_name_(object_name),
  cert_body_(cert_body),
  cert_key_(cert_key),
  password_hash_(password_hash)
{
}